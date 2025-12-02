#pragma once

#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Ініціалізація 1-Wire на вказаному піні (наприклад GPIO_NUM_4)
esp_err_t ds18b20_node_init(gpio_num_t pin);

// ФріРТОС-таска: всередині сама періодично читає датчик
void ds18b20_node_task(void *arg);

// Якщо не хочеш окрему таску — можна викликати цю функцію
// з якоїсь існуючої таски (вона всередині стейт-машина)
void ds18b20_node_update(void);

// Чи є вже валідне останнє значення
bool  ds18b20_node_has_value(void);

// Остання температура в °C (якщо ще нічого немає — NAN)
float ds18b20_node_get_last(void);

#ifdef __cplusplus
}
#endif
