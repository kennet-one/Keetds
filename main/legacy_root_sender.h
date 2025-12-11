#pragma once

#include <stdbool.h>
#include "freertos/FreeRTOS.h"   // для UBaseType_t

#ifdef __cplusplus
extern "C" {
#endif

#define LEGACY_ROOT_MSG_MAX_LEN  32

// prio – пріоритет таски (як у xTaskCreate).
// Якщо передати 0 – всередині підставимо дефолт (5).
void legacy_root_sender_start(UBaseType_t prio);

bool legacy_send_to_root(const char *text);

#ifdef __cplusplus
}
#endif
