#include "legacy_root_sender.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_wifi.h"
#include "esp_mesh.h"
#include "esp_log.h"

// -----------------------------------------------------------------------------
//  Локальна копія формату mesh_packet_t (має збігатися з тим, що в mesh_main.c)
// -----------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
	uint8_t  magic;
	uint8_t  version;
	uint8_t  type;
	uint8_t  reserved;
	uint32_t counter;
	uint8_t  src_mac[6];
	char     payload[32];
} mesh_packet_t;

#define MESH_PKT_MAGIC        0xA5
#define MESH_PKT_VERSION      1
#define MESH_PKT_TYPE_TEXT    1    // той самий тип, що й "Hello N"

// -----------------------------------------------------------------------------
//  Черга
// -----------------------------------------------------------------------------

typedef struct {
	char text[LEGACY_ROOT_MSG_MAX_LEN];
} legacy_msg_t;

static const char   *TAG    = "legacy_root_tx";
static QueueHandle_t s_q    = NULL;
static TaskHandle_t  s_task = NULL;
static uint32_t      s_cnt  = 0;

#define LEGACY_ROOT_QUEUE_LEN  16

// -----------------------------------------------------------------------------
//  Таска, яка реально шле на root
// -----------------------------------------------------------------------------

static void legacy_root_sender_task(void *arg)
{
	mesh_data_t   data;
	mesh_addr_t   dest;
	mesh_packet_t pkt;
	esp_err_t     err;

	data.data  = (uint8_t *)&pkt;
	data.proto = MESH_PROTO_BIN;
	data.tos   = MESH_TOS_P2P;

	// 00:00:00:00:00:00 -> root
	memset(&dest, 0, sizeof(dest));

	while (1) {
		legacy_msg_t msg;

		if (xQueueReceive(s_q, &msg, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		// Збираємо mesh_packet
		memset(&pkt, 0, sizeof(pkt));
		pkt.magic   = MESH_PKT_MAGIC;
		pkt.version = MESH_PKT_VERSION;
		pkt.type    = MESH_PKT_TYPE_TEXT;
		pkt.counter = ++s_cnt;

		esp_wifi_get_mac(WIFI_IF_STA, pkt.src_mac);
		strncpy(pkt.payload, msg.text, sizeof(pkt.payload) - 1);

		data.size = sizeof(pkt);

		// Поки не відправиться / не приймемо рішення дропнути
		for (;;) {
			err = esp_mesh_send(&dest, &data, MESH_DATA_P2P, NULL, 0);
			if (err == ESP_OK) {
				ESP_LOGI(TAG, "TX -> ROOT legacy: \"%s\"", msg.text);
				break;
			}

			ESP_LOGW(TAG,
			         "esp_mesh_send failed: 0x%x (%s), retry later",
			         err, esp_err_to_name(err));

			// Пробуємо покласти повідомлення назад у початок черги
			if (xQueueSendToFront(s_q, &msg, 0) != pdPASS) {
				ESP_LOGW(TAG,
				         "queue full on retry, drop message \"%s\"",
				         msg.text);
				break;  // щоб не зациклитися
			}

			// Чекаємо секунду, даємо mesh'у шанс відновитися
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}
}

// -----------------------------------------------------------------------------
//  Публічні функції
// -----------------------------------------------------------------------------

void legacy_root_sender_start(UBaseType_t task_prio)
{
    if (s_q) {
        return;  // вже запущено
    }

    // Якщо передали 0 – підставляємо дефолтне значення
    if (task_prio == 0) {
        task_prio = 5;
    }

    s_q = xQueueCreate(LEGACY_ROOT_QUEUE_LEN, sizeof(legacy_msg_t));
    if (!s_q) {
        ESP_LOGE(TAG, "failed to create queue");
        return;
    }

    BaseType_t ok = xTaskCreate(
        legacy_root_sender_task,
        "legacy_root_tx",
        4096,
        NULL,
        task_prio,
        &s_task
    );

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create task");
        vQueueDelete(s_q);
        s_q = NULL;
    } else {
        ESP_LOGI(TAG, "legacy_root_sender started, prio=%u, queue len=%d",
                 (unsigned)task_prio, LEGACY_ROOT_QUEUE_LEN);
    }
}


bool legacy_send_to_root(const char *text)
{
	if (!s_q || !text || !text[0]) {
		return false;
	}

	legacy_msg_t msg = {0};
	strncpy(msg.text, text, sizeof(msg.text) - 1);

	BaseType_t ok = xQueueSend(s_q, &msg, 0);
	if (ok != pdPASS) {
		ESP_LOGW(TAG, "queue full, drop \"%s\"", msg.text);
		return false;
	}
	return true;
}
