#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


#include "ds18b20_node.h"   // ds18b20_node_has_value(), ds18b20_node_get_last()
#include "tds_node.h"

/* -------------------------------------------------------------------------- */
/*  Налаштування TDS / ADC                                                    */
/* -------------------------------------------------------------------------- */

static const char *TAG = "tds_node";

/* TDS підключений до GPIO34 -> це ADC1, канал 6 */
#define TDS_GPIO         GPIO_NUM_34
#define TDS_ADC_UNIT     ADC_UNIT_1
#define TDS_ADC_CHANNEL  ADC_CHANNEL_6   // відповідає GPIO34 для ADC1

/* Константи з Arduino-коду */
static const float VREF     = 3.3f;     // опорна напруга
static const float TDS_K    = 595.0f;   // коеф для ppm
static const float TDS_BASE = 180.0f;   // базова вода
static const float ALPHA_T  = 0.02f;    // 2%/°C для температурної компенсації

/* Скільки семплів на одне вимірювання та інтервали між семплами */
#define TDS_SAMPLES             20
#define TDS_SAMPLE_INTERVAL_MS  50

/* Період повного вимірювання (для робочого режиму)
 * Для дебагу можна ставити 10000 (10 c), потім зробиш 3600000 (1 година)
 */
#define TDS_MEASURE_PERIOD_MS   60000UL

/* -------------------------------------------------------------------------- */
/*  Стани / глобали                                                           */
/* -------------------------------------------------------------------------- */

static bool  s_inited           = false;
static bool  s_has_values       = false;

static int   s_last_raw         = 0;
static float s_last_voltage     = 0.0f;
static float s_last_tds_raw     = 0.0f;
static float s_last_tds25       = 0.0f;
static float s_last_tds_broth   = 0.0f;
static float s_last_tds_broth25 = 0.0f;
static float s_last_temp_used   = NAN;

/* хендл нового oneshot-драйвера */
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t	s_adc_cali = NULL;
static bool					s_adc_cal_ok = false;

/* -------------------------------------------------------------------------- */
/*  Геттери                                                                    */
/* -------------------------------------------------------------------------- */

bool tds_node_has_last_values(void)
{
    return s_has_values;
}

float tds_node_get_last_tds25(void)
{
    return s_last_tds25;
}

float tds_node_get_last_tds_broth25(void)
{
    return s_last_tds_broth25;
}

float tds_node_get_last_temp_used(void)
{
    return s_last_temp_used;
}

/* -------------------------------------------------------------------------- */
/*  Ініціалізація ADC oneshot                                                 */
/* -------------------------------------------------------------------------- */

static void tds_adc_try_init_calibration(adc_atten_t atten, adc_bitwidth_t bitwidth)
{
	s_adc_cali = NULL;
	s_adc_cal_ok = false;

	#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
	{
		adc_cali_curve_fitting_config_t cal_cfg =
		{
			.unit_id = TDS_ADC_UNIT,
			.atten = atten,
			.bitwidth = bitwidth,
		};

		if(adc_cali_create_scheme_curve_fitting(&cal_cfg, &s_adc_cali) == ESP_OK)
		{
			s_adc_cal_ok = true;
			ESP_LOGI(TAG, "ADC calibration: curve fitting OK");
			return;
		}
	}
	#endif

	#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
	{
		adc_cali_line_fitting_config_t cal_cfg =
		{
			.unit_id = TDS_ADC_UNIT,
			.atten = atten,
			.bitwidth = bitwidth,
		};

		if(adc_cali_create_scheme_line_fitting(&cal_cfg, &s_adc_cali) == ESP_OK)
		{
			s_adc_cal_ok = true;
			ESP_LOGI(TAG, "ADC calibration: line fitting OK");
			return;
		}
	}
	#endif

	ESP_LOGW(TAG, "ADC calibration not available -> rough raw->V mapping");
}


void tds_node_init(void)
{
    if (s_inited) {
        return;
    }

    /* 1) Створюємо "юніт" ADC (oneshot) */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = TDS_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    /* 2) Налаштовуємо канал (бітність і атенюацію) */
	adc_oneshot_chan_cfg_t chan_cfg =
	{
		.bitwidth = ADC_BITWIDTH_12,
		.atten    = ADC_ATTEN_DB_11,
	};

	ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, TDS_ADC_CHANNEL, &chan_cfg));

	tds_adc_try_init_calibration(chan_cfg.atten, chan_cfg.bitwidth);
	ESP_LOGI(TAG, "ADC cal=%s", s_adc_cal_ok ? "YES" : "NO");


    s_inited = true;
    ESP_LOGI(TAG, "TDS node init done (GPIO%d, unit=%d, ch=%d)",
             (int)TDS_GPIO, (int)TDS_ADC_UNIT, (int)TDS_ADC_CHANNEL);
}

/* -------------------------------------------------------------------------- */
/*  Основна таска TDS                                                         */
/* -------------------------------------------------------------------------- */

static void tds_node_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {

        /* Чекаємо до наступного “слоту” вимірювання */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TDS_MEASURE_PERIOD_MS));

        if (!s_inited || s_adc_handle == NULL) {
            continue;
        }

        int sum_raw = 0;
        int raw_cnt = 0;

        int sum_mv = 0;
        int mv_cnt = 0;

        for (int i = 0; i < TDS_SAMPLES; ++i) {
            int raw = 0;
            esp_err_t err = adc_oneshot_read(s_adc_handle, TDS_ADC_CHANNEL, &raw);

            if (err == ESP_OK) {
                sum_raw += raw;
                raw_cnt++;

                if (s_adc_cal_ok && s_adc_cali) {
                    int mv = 0;
                    if (adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) == ESP_OK) {
                        sum_mv += mv;
                        mv_cnt++;
                    }
                }
            } else {
                ESP_LOGE(TAG, "adc_oneshot_read failed: 0x%x", err);
            }

            vTaskDelay(pdMS_TO_TICKS(TDS_SAMPLE_INTERVAL_MS));
        }

        int avg_raw = (raw_cnt > 0) ? (sum_raw / raw_cnt) : 0;

        float voltage = 0.0f;
        if ((s_adc_cal_ok && s_adc_cali) && mv_cnt > 0) {
            float avg_mv = (float)sum_mv / (float)mv_cnt;
            voltage = avg_mv / 1000.0f;
        } else {
            voltage = (avg_raw / 4095.0f) * VREF; // fallback
        }

        float tds_raw = voltage * TDS_K;

        float tds_broth = tds_raw - TDS_BASE;
        if (tds_broth < 0.0f) {
            tds_broth = 0.0f;
        }

        float temp_c      = NAN;
        float tds25       = tds_raw;
        float tds_broth25 = tds_broth;

        /* Питаємо температуру у ds18b20_node.* (наш сенсор температури) */
        if (ds18b20_node_has_value()) {
            temp_c = ds18b20_node_get_last();

            float factor = 1.0f + ALPHA_T * (temp_c - 25.0f);
            if (factor <= 0.0f) {
                factor = 1.0f;
            }
            tds25       = tds_raw   / factor;
            tds_broth25 = tds_broth / factor;
        }

        /* Зберігаємо в глобальні змінні для геттерів */
        s_last_raw         = avg_raw;
        s_last_voltage     = voltage;
        s_last_tds_raw     = tds_raw;
        s_last_tds25       = tds25;
        s_last_tds_broth   = tds_broth;
        s_last_tds_broth25 = tds_broth25;
        s_last_temp_used   = temp_c;
        s_has_values       = true;

        /* Лог — одна жирна строка, як ти хотів */
        if (!isnan(temp_c)) {
            ESP_LOGI(TAG,
                     "RAW=%d U=%.3fV  TDS_raw=%.1fppm  TDS25=%.1fppm  "
                     "BROTH_raw=%.1fppm  BROTH25=%.1fppm  temp=%.2fC",
                     avg_raw, voltage,
                     tds_raw, tds25,
                     tds_broth, tds_broth25,
                     temp_c);
        } else {
            ESP_LOGI(TAG,
                     "RAW=%d U=%.3fV  TDS_raw=%.1fppm  TDS25=%.1fppm  "
                     "BROTH_raw=%.1fppm  BROTH25=%.1fppm  temp=N/A",
                     avg_raw, voltage,
                     tds_raw, tds25,
                     tds_broth, tds_broth25);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  Старт таски                                                               */
/* -------------------------------------------------------------------------- */

void tds_node_start_task(UBaseType_t prio)
{
    static bool started = false;

    if (started) {
        return;
    }
    started = true;

    BaseType_t ok = xTaskCreate(
        tds_node_task,
        "tds_node",
        4096,          // потім глянеш у stack_mon і підправиш
        NULL,
        prio,
        NULL);

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create tds_node task");
    }
}
