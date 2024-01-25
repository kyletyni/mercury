#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "encoder.h"
#include "emitter.h"
#include "motors.h"
#include "receiver.h"
#include "maze.h"


#define BLUE_LED GPIO_NUM_48
#define GREEN_LED GPIO_NUM_47
#define RED_LED GPIO_NUM_35

const char *TAG = "main";

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    initialize_gpio(EMIT_DR);
    initialize_gpio(EMIT_DL);
    initialize_gpio(EMIT_FR);
    initialize_gpio(EMIT_FL);
    ESP_LOGI(TAG, "EMIT pins initialized successfully");

    initialize_gpio(BLUE_LED);
    initialize_gpio(GREEN_LED);
    initialize_gpio(RED_LED);
    ESP_LOGI(TAG, "LED pins initialized successfully");
    
    ESP_LOGI(TAG, "Initializing ADC");
    initialize_adc();
    ESP_LOGI(TAG, "ADC initialized successfully");
    
    // ESP_ERROR_CHECK(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP_FIT));

    while (true)
    {
        for(int i = 0; i < 100; i++)
        {
            perform_adc_conversion();
        }
        ESP_LOGI(TAG, "adc: %d, conv: %d", recv_adc_raw[0], recv_voltage[0]);

        vTaskDelay(100);
    }   
}
