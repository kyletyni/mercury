#ifndef EMITTER_H
#define EMITTER_H

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"

#define EMIT_DR GPIO_NUM_13
#define EMIT_DL GPIO_NUM_21
#define EMIT_FR GPIO_NUM_3
#define EMIT_FL GPIO_NUM_2

void init_emit_pin(gpio_num_t gpio_num) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(gpio_num, 0);
}

void emit_pin_on(gpio_num_t gpio_num) {
    gpio_set_level(gpio_num, 1);
}

void emit_pin_off(gpio_num_t gpio_num) {
    gpio_set_level(gpio_num, 0);
}

void set_emit_pin(gpio_num_t gpio_num, int state) {
    gpio_set_level(gpio_num, state);
}

#endif