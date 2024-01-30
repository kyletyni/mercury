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
#include "user.h"

#include "gyro.h"

#define BLUE_LED GPIO_NUM_48
#define GREEN_LED GPIO_NUM_47
#define RED_LED GPIO_NUM_35

const char *TAG = "main";

MessageBufferHandle_t xMessageBufferToClient;
QueueHandle_t xQueueTrans;


extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    init_emit_pin(EMIT_DR);
    init_emit_pin(EMIT_DL);
    init_emit_pin(EMIT_FR);
    init_emit_pin(EMIT_FL);
    ESP_LOGI(TAG, "EMIT pins initialized successfully");

    init_user_io();
    ESP_LOGI(TAG, "LED pins initialized successfully");
    
    ESP_LOGI(TAG, "Initializing ADC");
    initialize_adc();
    ESP_LOGI(TAG, "ADC initialized successfully");
    
    init_motors();
    ESP_LOGI(TAG, "Motor PWM initialized successfully");

    
	// Create Queue
	xQueueTrans = xQueueCreate(10, sizeof(POSE_t));
	configASSERT( xQueueTrans );

	// Create Message Buffer
	xMessageBufferToClient = xMessageBufferCreate(1024);
	configASSERT( xMessageBufferToClient );

    // xTaskCreate(&button_task, "button_task", 4096, NULL, 10, NULL);

    // ESP_LOGI(TAG, "creating Gyro task");
    // xTaskCreate(&mpu6500, "IMU", 1024*8, NULL, 5, NULL);

    while (true)
    {

        /******************* Continuously Poll Encoder Counts *******************/
        get_raw_encoder_count();
        ESP_LOGI(TAG, "R: %4d, L: %4d", raw_count_left, raw_count_right);

        // set_pwm_dc(ML_PWMB_CHANNEL, 0.f);

        /******************* Continuously Poll IR Sensors *******************/
        // ir_sensor_poll(FRONT_LEFT);
        // vTaskDelay(1);
        // ir_sensor_poll(DIAGONAL_LEFT);
        // vTaskDelay(1);
        // ir_sensor_poll(DIAGONAL_RIGHT);
        // vTaskDelay(1);
        // ir_sensor_poll(FRONT_RIGHT);
        // vTaskDelay(1);
        // ESP_LOGI(TAG, "FR: %4d, DR: %4d, DL: %4d, FL: %4d", recv_adc_raw[FRONT_RIGHT], recv_adc_raw[DIAGONAL_RIGHT], recv_adc_raw[DIAGONAL_LEFT], recv_adc_raw[FRONT_LEFT]);

        vTaskDelay(10);
    }   
}
