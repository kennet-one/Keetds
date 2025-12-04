#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Ініціалізація черги + старт таски.
 * prio – пріоритет FreeRTOS-таски (наприклад 5).
 */
esp_err_t legacy_root_sender_init(UBaseType_t prio);

/**
 * Кинути текстове legacy-повідомлення в чергу на відправку до ROOT.
 * Повертає true, якщо поклали в чергу, false – якщо текст пустий або черга забита.
 * Рядок копіюється у внутрішній буфер (до 31 символа + '\0').
 */
bool legacy_send_to_root(const char *text);

#ifdef __cplusplus
}
#endif
