#ifndef USER_H
#define USER_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BTN1 GPIO_NUM_5
#define BTN2 GPIO_NUM_6

#define RED_LED GPIO_NUM_35
#define BLUE_LED GPIO_NUM_48
#define GREEN_LED GPIO_NUM_47

const char *user_tag = "user";

void init_user_io() {

    // Configure BTN1
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Configure BTN2
    io_conf.pin_bit_mask = (1ULL << BTN2);
    ESP_ERROR_CHECK(gpio_config(&io_conf));


    // configure the RED led
    io_conf.pin_bit_mask = (1ULL << RED_LED);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(RED_LED, 1);
    
    // configure the BLUE led
    io_conf.pin_bit_mask = (1ULL << BLUE_LED);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(BLUE_LED, 1);
    
    // configure the GREEN led
    io_conf.pin_bit_mask = (1ULL << GREEN_LED);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(GREEN_LED, 0);
}

    
void button_task(void *pvParameter) {
    while (1) {
        // Check if the button is pressed (logic low)
        if (gpio_get_level(BTN1) == 0) {
            ESP_LOGI(user_tag, "Button1 pressed!");
            ESP_LOGI(user_tag, "LED on!");
            gpio_set_level(GREEN_LED, 0);
            gpio_set_level(RED_LED, 0);

            // Optional: Debounce the button press to avoid multiple triggers
            vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100ms
        }

        if (gpio_get_level(BTN2) == 0) {
            ESP_LOGI(user_tag, "Button2 pressed!");
            ESP_LOGI(user_tag, "LED off");
            gpio_set_level(GREEN_LED, 1);
            gpio_set_level(RED_LED, 1);

            // Optional: Debounce the button press to avoid multiple triggers
            vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100ms
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Check the button state every 10ms
    }
}

#endif