#ifndef PIN_H
#define PIN_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

/******************* Emitter + Receiver Pins *******************/
// Front Left IR Pins
#define FL_RECV_PIN GPIO_NUM_1
#define FL_EMIT_PIN GPIO_NUM_2

// Front Right IR Pins
#define FR_RECV_PIN GPIO_NUM_9
#define FR_EMIT_PIN GPIO_NUM_3

// Diagonal Right IR Pins
#define DL_RECV_PIN GPIO_NUM_14
#define DL_EMIT_PIN GPIO_NUM_21

// Diagonal Left IR Pins
#define DR_RECV_PIN GPIO_NUM_12
#define DR_EMIT_PIN GPIO_NUM_13

/******************* Motor PWM Pins *******************/
// Right Motor
#define MR_PWMA_PIN GPIO_NUM_38
#define MR_PWMB_PIN GPIO_NUM_8

// Left Motor
#define ML_PWMA_PIN GPIO_NUM_36
#define ML_PWMB_PIN GPIO_NUM_37

// Fan Motor
#define FAN_PWM_PIN GPIO_NUM_16

/******************* Button Pins *******************/
#define BTN1 GPIO_NUM_5
#define BTN2 GPIO_NUM_6

/******************* LED Pins *******************/
#define RED_LED GPIO_NUM_35
#define BLUE_LED GPIO_NUM_48
#define GREEN_LED GPIO_NUM_47


void initialize_emitter_output_pin(gpio_num_t gpio_num) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

#endif