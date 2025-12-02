#include "ds18b20_node.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ds18b20";

// Стан стейт-машини
typedef enum {
	DS_STATE_IDLE = 0,
	DS_STATE_CONVERTING,
} ds_state_t;

typedef struct {
	gpio_num_t pin;
	bool       inited;
	bool       present;        // є пристрій на шині
	ds_state_t state;

	uint32_t   last_ms;        // коли останній раз успішно прочитали температуру
	uint32_t   conv_start_ms;  // коли стартонуло перетворення

	float      last_temp;      // остання температура (або NAN)
} ds_ctx_t;

static ds_ctx_t s_ds = {
	.pin       = GPIO_NUM_NC,
	.inited    = false,
	.present   = false,
	.state     = DS_STATE_IDLE,
	.last_ms   = 0,
	.conv_start_ms = 0,
	.last_temp = NAN,
};

// Параметри часу
#define DS_PERIOD_MS       5000   // як часто хочемо нове значення
#define DS_CONVERT_MS       800   // час конверсії DS18B20 (750ms + запас)

// -----------------------------------------------------------------------------
//  Хелпери
// -----------------------------------------------------------------------------

static inline uint32_t now_ms(void)
{
	return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// Лінія низько (0)
static inline void ow_drive_low(void)
{
	gpio_set_direction(s_ds.pin, GPIO_MODE_OUTPUT);
	gpio_set_level(s_ds.pin, 0);
}

// Відпускаємо лінію (input) — підтягується резистором до 3.3V
static inline void ow_release_line(void)
{
	gpio_set_direction(s_ds.pin, GPIO_MODE_INPUT);
}

// Read level
static inline int ow_read_line(void)
{
	return gpio_get_level(s_ds.pin);
}

// 1-Wire reset. Повертає true, якщо є presence-pulse від пристрою.
static bool ow_reset(void)
{
	ow_drive_low();
	esp_rom_delay_us(480);          // тримаємо 480µs
	ow_release_line();
	esp_rom_delay_us(70);           // чекаємо presence

	int presence = ow_read_line();  // 0 = девайс відповів
	esp_rom_delay_us(410);          // добиваємо до 960µs
	return (presence == 0);
}

// Запис одного біта
static void ow_write_bit(int bit)
{
	ow_drive_low();
	if (bit) {
		// '1' — короткий імпульс
		esp_rom_delay_us(6);
		ow_release_line();
		esp_rom_delay_us(64);
	} else {
		// '0' — тримаємо низько майже весь слот
		esp_rom_delay_us(60);
		ow_release_line();
		esp_rom_delay_us(10);
	}
}

// Читання одного біта
static int ow_read_bit(void)
{
	int r;

	ow_drive_low();
	esp_rom_delay_us(6);       // стартуємо слот
	ow_release_line();
	esp_rom_delay_us(9);       // чекаємо, поки девайс виставить біт
	r = ow_read_line();
	esp_rom_delay_us(55);      // добиваємо слот
	return r;
}

// Запис байта (LSB first)
static void ow_write_byte(uint8_t v)
{
	for (int i = 0; i < 8; i++) {
		ow_write_bit(v & 0x01);
		v >>= 1;
	}
}

// Читання байта (LSB first)
static uint8_t ow_read_byte(void)
{
	uint8_t v = 0;
	for (int i = 0; i < 8; i++) {
		int b = ow_read_bit();
		if (b) {
			v |= (1U << i);
		}
	}
	return v;
}

// -----------------------------------------------------------------------------
//  Публічний API
// -----------------------------------------------------------------------------

esp_err_t ds18b20_node_init(gpio_num_t pin)
{
	if (s_ds.inited) {
		return ESP_OK;
	}

	s_ds.pin       = pin;
	s_ds.inited    = true;
	s_ds.present   = false;
	s_ds.state     = DS_STATE_IDLE;
	s_ds.last_temp = NAN;

	// Налаштовуємо GPIO (input + pull-up, далі будемо міняти режим)
	gpio_config_t io_conf = {
		.pin_bit_mask = 1ULL << pin,
		.mode         = GPIO_MODE_INPUT,
		.pull_up_en   = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&io_conf));

	// Перша спроба знайти девайс
	if (ow_reset()) {
		ESP_LOGI(TAG, "device present on GPIO %d", pin);
		s_ds.present = true;
	} else {
		ESP_LOGW(TAG, "no DS18B20 response on GPIO %d", pin);
		s_ds.present = false;
	}

	return ESP_OK;
}

bool ds18b20_node_has_value(void)
{
	return !isnan(s_ds.last_temp);
}

float ds18b20_node_get_last(void)
{
	return s_ds.last_temp;
}

// Один крок стейт-машини (можна викликати з будь-якої таски)
void ds18b20_node_update(void)
{
	if (!s_ds.inited) return;

	uint32_t now = now_ms();

	// Якщо девайса немає — іноді пробуємо знайти знову
	if (!s_ds.present) {
		static uint32_t last_try = 0;
		if (now - last_try < 2000) {
			return;
		}
		last_try = now;
		if (ow_reset()) {
			ESP_LOGI(TAG, "device re-appeared");
			s_ds.present = true;
			s_ds.state   = DS_STATE_IDLE;
		}
		return;
	}

	switch (s_ds.state) {
	case DS_STATE_IDLE:
		// Час запустити нову конверсію?
		if (now - s_ds.last_ms >= DS_PERIOD_MS) {
			if (!ow_reset()) {
				ESP_LOGW(TAG, "lost device during start conversion");
				s_ds.present = false;
				s_ds.last_temp = NAN;
				return;
			}
			ow_write_byte(0xCC);  // SKIP ROM (припускаємо один девайс на шині)
			ow_write_byte(0x44);  // CONVERT T

			s_ds.conv_start_ms = now;
			s_ds.state = DS_STATE_CONVERTING;
		}
		break;

	case DS_STATE_CONVERTING:
		// Чекаємо завершення конверсії
		if (now - s_ds.conv_start_ms >= DS_CONVERT_MS) {
			uint8_t data[9];

			if (!ow_reset()) {
				ESP_LOGW(TAG, "lost device during read scratchpad");
				s_ds.present = false;
				s_ds.last_temp = NAN;
				s_ds.state = DS_STATE_IDLE;
				return;
			}
			ow_write_byte(0xCC);  // SKIP ROM
			ow_write_byte(0xBE);  // READ SCRATCHPAD

			for (int i = 0; i < 9; i++) {
				data[i] = ow_read_byte();
			}

			int16_t raw = (int16_t)((data[1] << 8) | data[0]);
			float   t   = (float)raw / 16.0f;

			s_ds.last_temp = t;
			s_ds.last_ms   = now;
			s_ds.state     = DS_STATE_IDLE;

			ESP_LOGI(TAG, "temperature = %.2f C", t);
		}
		break;
	}
}

// Окрема таска, яка сама крутить update()
void ds18b20_node_task(void *arg)
{
	(void)arg;
	while (1) {
		ds18b20_node_update();
		vTaskDelay(pdMS_TO_TICKS(50));   // ≈20 разів на секунду — більш ніж достатньо
	}
}
