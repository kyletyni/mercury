#include <stdio.h>

#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <freertos/queue.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "encoder.h"
#include "emitter.h"
#include "profile.h"
#include "motors.h"
#include "receiver.h"
#include "maze.h"
#include "user.h"

#include "gyro.h"

#define BLUE_LED GPIO_NUM_48
#define GREEN_LED GPIO_NUM_47
#define RED_LED GPIO_NUM_35

const char *TAG = "main";

void sensorPollTask(void *pvParameters);
void motorControlTask(void *pvParameters);
void operatorTask(void *pvParameters);

MessageBufferHandle_t xMessageBufferToClient;
QueueHandle_t xQueueTrans;


extern "C" void app_main(void)
{
    // esp_set_cpu_affinity(xTaskGetCurrentTask(), esp_cpu_pin_to_core(CPU_0)); // Core 0 for current task
    // vPortSetTickFrequency(1000);

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

    // xTaskCreate(&button_task, "button_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Spawning Tasks for Core 0");
    xTaskCreatePinnedToCore(sensorPollTask, "sensorPollTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(&mpu6500, "gyroTask", 4096, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "Creating Tasks for Core 1");
    xTaskCreatePinnedToCore(motorControlTask, "motorControlTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(operatorTask, "operatorTask", 4096, NULL, 1, NULL, 1);
}


void sensorPollTask(void *pvParameters)
{
    TickType_t xFrequency = pdMS_TO_TICKS(1); // convert to ms
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    reset_encoder_count();
    int count = 0;

    while (true)
    {   
        xLastWakeTime = xTaskGetTickCount();
        /******************* Continuously Poll Encoder Counts *******************/
        update_encoders();
        update_distances();

        // NOTE: RIGHT Encoder Counts DOWN when dir is FORWARD, counts UP when dir is BACKWARD
        // NOTE: LEFT Encoder Counts UP when dir is FORWARD, counts DOWN when dir is BACKWARD
        // ESP_LOGI(TAG, "core0 L: %4d, L_total: %6ld, R: %4d, R_total: %6ld", raw_count_left, total_count_left, raw_count_right, total_count_right);
        
        /******************* Continuously Poll IR Sensors *******************/
        ir_sensor_poll(FRONT_LEFT);
        ir_sensor_poll(DIAGONAL_LEFT);
        ir_sensor_poll(DIAGONAL_RIGHT);
        ir_sensor_poll(FRONT_RIGHT);

        // calculate cross track error


        // if no walls, we use gyro


        // ESP_LOGI(TAG, "FR: %4d, DR: %4d, DL: %4d, FL: %4d", recv_avg_val[FRONT_RIGHT], recv_avg_val[DIAGONAL_RIGHT], recv_avg_val[DIAGONAL_LEFT], recv_avg_val[FRONT_LEFT]);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        TickType_t xElapsedTime = (xTaskGetTickCount() - xLastWakeTime);
        if (xElapsedTime > xFrequency) {
            ESP_LOGW(TAG, "WARNING: 'sensorPollTask' exceeded deadline: %ld ms!", xElapsedTime);
        }
    }
}


void motorControlTask(void *pvParameters)
{
    TickType_t xFrequency = pdMS_TO_TICKS(2); // convert to ms
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    float fwd_output;
    float rot_output;

    float left_output_v;
    float right_output_v;

    mouse_distance = 0.f;
    Profile_Reset(&forward);
    Profile_Reset(&rotation);

    int count = 0;
    controller_output_enabled = true;

    while (true)
    {
        xLastWakeTime = xTaskGetTickCount();
        
        // Updates Profiles
        Profile_Update(&forward);
        Profile_Update(&rotation);

        fwd_output = forward_controller();
        rot_output = rotation_controller(0.f);

        left_output_v = 0.f;
        right_output_v = 0.f;

        left_output_v = fwd_output + rot_output;
        right_output_v = fwd_output - rot_output;

        float tangent_velocity = rotation.velocity * MOUSE_RADIUS * RADIANS_PER_DEGREE;
        float left_speed = forward.velocity - tangent_velocity;
        float right_speed = forward.velocity + tangent_velocity;

        float left_ff = left_ff_voltage(left_speed);
        float right_ff = right_ff_voltage(right_speed);
        
        // print logic
        if (count % 60 == 0) {
            count = 1;
            ESP_LOGI(TAG, "a %.2f r %.2f y %.2f", mouse_angle, rotation.position, rot_output);
        } else {
            count++;
        }

        // Output motor voltages
        left_output_v += left_ff;
        right_output_v += right_ff;
        
        if (controller_output_enabled) {
            set_motor_voltage(MOTOR_RIGHT, right_output_v);
            set_motor_voltage(MOTOR_LEFT, left_output_v);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        TickType_t xElapsedTime = (xTaskGetTickCount() - xLastWakeTime);
        if (xElapsedTime > xFrequency) {
            ESP_LOGW(TAG, "WARNING: 'motorControlTask' exceeded deadline: %ld ms!", xElapsedTime);
        }
    }
}

void operatorTask(void *pvParameters)
{
    TickType_t xFrequency = pdMS_TO_TICKS(1); // convert to ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    ESP_LOGI(TAG, "rot_kp %.2f rot_kd: %.2f", ROT_KP, ROT_KD);

    bool test_fwd = true;

    while (true) 
    {
        vTaskDelay(500);
        if (test_fwd) {
            // Profile_Start(&forward, 300.f, 300.f, 0.f, 1000.f);
            // set_motor_dir(MOTOR_RIGHT, BACKWARD);
            
            Profile_Start(&forward, 0.f, 0.f, 0.f, 0.f);
            
            Profile_Start(&rotation, 90.f, 300.f, 0.f, 2000.f);
            while(rotation.state != PS_DONE) {
                vTaskDelay(2);
            }

            // test_fwd = false;
        }
    }
}