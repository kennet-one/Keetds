#pragma once

#include <stdbool.h>
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Ініціалізація TDS-вузла (ADC, стани).
 *
 * Викликаєш ОДИН раз з app_main(), до start_task().
 */
void tds_node_init(void);

/**
 * @brief Запуск FreeRTOS-таски вимірювання TDS.
 *
 * @param prio Пріоритет таски (наприклад 5).
 */
void tds_node_start_task(UBaseType_t prio);

/* ----- Геттери останніх виміряних значень ----- */

/**
 * @return true якщо вже є хоч одне валідне вимірювання.
 */
bool  tds_node_has_last_values(void);

/**
 * @return останній TDS, приведений до 25°C (ppm).
 */
float tds_node_get_last_tds25(void);

/**
 * @return останній “бульйонний” TDS (поверх базової води) при 25°C (ppm).
 */
float tds_node_get_last_tds_broth25(void);

/**
 * @return яку температуру (°C) реально використали при компенсації.
 *         Може бути NAN, якщо DS18B20 ще не давав коректних даних.
 */
float tds_node_get_last_temp_used(void);

#ifdef __cplusplus
}
#endif
