#include "legacy_root_sender.h"

#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_mesh.h"

static const char *TAG = "legacy_tx";

/* ---------- mesh-пакет того ж формату, що й у mesh_main.c ---------- */

typedef struct __attribute__((packed)) {
	uint8_t  magic;
	uint8_t  version;
	uint8_t  type;
	uint8_t  reserved;
	uint32_t counter;
	uint8_t  src_mac[6];
	char     payload[32];
} mesh_legacy_packet_t;

#define MESH_PKT_MAGIC      0xA5
#define MESH_PKT_VERSION    1
#define MESH_PKT_TYPE_TEXT  1

/* ---------- Черга текстів ---------- */

#define LEGACY_MSG_MAX_LEN  31
#define LEGACY_QUEUE_LENGTH 16

typedef struct {
	char text[LEGACY_MSG_MAX_LEN + 1];
} legacy_msg_t;

static QueueHandle_t s_queue        = NULL;
static bool          s_task_started = false;
static uint32_t      s_pkt_counter  = 0;

/* ---------- Внутрішня таска, яка реально шле в mesh ---------- */

static void legacy_root_sender_task(void *arg)
{
	(void)arg;

	legacy_msg_t msg;
	mesh_legacy_packet_t pkt;
	mesh_data_t data;
	mesh_addr_t dest;
	esp_err_t err;

	data.data  = (uint8_t *)&pkt;
	data.proto = MESH_PROTO_BIN;
	data.tos   = MESH_TOS_P2P;

	while (1) {
		/* Чекаємо нове повідомлення з черги */
		if (xQueueReceive(s_queue, &msg, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		/* Якщо ми раптом root – лог і пропускаємо */
		if (esp_mesh_is_root()) {
			ESP_LOGW(TAG, "I'm ROOT, skip send_to_root: \"%s\"", msg.text);
			continue;
		}

		memset(&pkt, 0, sizeof(pkt));
		pkt.magic   = MESH_PKT_MAGIC;
		pkt.version = MESH_PKT_VERSION;
		pkt.type    = MESH_PKT_TYPE_TEXT;
		pkt.counter = ++s_pkt_counter;

		esp_wifi_get_mac(WIFI_IF_STA, pkt.src_mac);

		strncpy(pkt.payload, msg.text, sizeof(pkt.payload) - 1);
		pkt.payload[sizeof(pkt.payload) - 1] = '\0';

		data.size = sizeof(pkt);

		/* 00:00:00:00:00:00 -> send to ROOT */
		memset(&dest, 0, sizeof(dest));

		err = esp_mesh_send(&dest, &data, MESH_DATA_P2P, NULL, 0);
		if (err == ESP_OK) {
			ESP_LOGI(TAG, "TX->ROOT legacy: \"%s\"", msg.text);
		} else {
			ESP_LOGE(TAG, "esp_mesh_send failed: 0x%x (%s)",
				err, esp_err_to_name(err));
		}
	}
}

/* ---------- Публічний API ---------- */

esp_err_t legacy_root_sender_init(UBaseType_t prio)
{
	if (!s_queue) {
		s_queue = xQueueCreate(LEGACY_QUEUE_LENGTH, sizeof(legacy_msg_t));
		if (!s_queue) {
			ESP_LOGE(TAG, "failed to create queue");
			return ESP_ERR_NO_MEM;
		}
	}

	if (!s_task_started) {
		BaseType_t ok = xTaskCreate(
			legacy_root_sender_task,
			"legacy_tx",
			4096,
			NULL,
			prio,
			NULL
		);
		if (ok != pdPASS) {
			ESP_LOGE(TAG, "failed to create legacy_tx task");
			return ESP_FAIL;
		}
		s_task_started = true;
	}

	return ESP_OK;
}

bool legacy_send_to_root(const char *text)
{
	if (!s_queue || !text) {
		return false;
	}

	legacy_msg_t msg;
	if (!text[0]) {
		return false;	// пустий рядок
	}

	strncpy(msg.text, text, LEGACY_MSG_MAX_LEN);
	msg.text[LEGACY_MSG_MAX_LEN] = '\0';

	BaseType_t ok = xQueueSendToBack(s_queue, &msg, 0);	// без блокування
	return (ok == pdTRUE);
}
