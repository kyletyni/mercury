#ifndef USER_H
#define USER_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define BTN1 GPIO_NUM_5
#define BTN2 GPIO_NUM_6

#define RED_LED GPIO_NUM_35
#define BLUE_LED GPIO_NUM_48
#define GREEN_LED GPIO_NUM_47

#define UART_NUM UART_NUM_0
#define TXD_PIN GPIO_NUM_44
#define RXD_PIN GPIO_NUM_43
#define BUF_SIZE (1024)

const char *user_tag = "user";
QueueHandle_t uart_queue;

typedef enum {
    SEARCH_MODE_SELECT = 0,
    SPEED_MODE_SELECT = 1,
    SELECT_MENU_SIZE = 2
} UserSelectMode;

typedef enum {
    SELECT_MODE_ACTIVE = 0,
    RUNNING_MODE_ACTIVE = 1,
    ACTIVE_MODE_SIZE = 2
} UserActiveMode;

UserSelectMode user_select_mode = SEARCH_MODE_SELECT;
UserActiveMode user_active_mode = SELECT_MODE_ACTIVE;

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
    gpio_set_level(GREEN_LED, 1);
}


void init_uart() 
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
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