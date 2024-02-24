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
#include "config.h"
#include "motion.h"

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

    init_uart();
    //ESP_LOGI(TAG, "UART initialized successfully");
    
    initialize_adc();
    ESP_LOGI(TAG, "ADC initialized successfully");
    
    init_motors();
    ESP_LOGI(TAG, "Motor PWM initialized successfully");
    

    // xTaskCreate(&button_task, "button_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Spawning Tasks for Core 0");
    xTaskCreatePinnedToCore(&mpu6500, "gyroTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sensorPollTask, "sensorPollTask", 4096, NULL, 2, NULL, 0);

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
        
        /******************* Continuously Poll IR Sensors *******************/
        ir_sensor_poll(FRONT_LEFT);
        ir_sensor_poll(DIAGONAL_LEFT);
        ir_sensor_poll(DIAGONAL_RIGHT);
        ir_sensor_poll(FRONT_RIGHT);
        // ESP_LOGI(TAG, "L: %4d, L_total: %6ld, R: %4d, R_total: %6ld", raw_count_left, total_count_left, raw_count_right, total_count_right);

        // calculate cross track error
        calc_cross_track_error();

        // if no walls, we use gyro

        if (count % 50 == 0) {
            // ESP_LOGI(TAG, "FR: %4d, DR: %4d, DL: %4d, FL: %4d, FRONT: %.2f", recv_avg_val[FRONT_RIGHT], recv_avg_val[DIAGONAL_RIGHT], recv_avg_val[DIAGONAL_LEFT], recv_avg_val[FRONT_LEFT], front_sensor_sum);
            count = 1;
        } else {
            count++;
        }

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
    steering_enabled = false;

    while (true)
    {
        xLastWakeTime = xTaskGetTickCount();
        
        // Updates Profiles
        Profile_Update(&forward);
        Profile_Update(&rotation);

        fwd_output = forward_controller();

        // calculate rotational adjustment
        static float rot_adjust = 0.f;

        if (steering_enabled) {
            rot_adjust = (STEERING_KP * cross_track_error) + (STEERING_KD * cross_track_error - cross_track_error_prev);
            rot_adjust *= LOOP_INTERVAL;
            if (rot_adjust < -STEERING_ADJUSTMENT_LIMIT)
                rot_adjust = -STEERING_ADJUSTMENT_LIMIT;
            else if (rot_adjust > STEERING_ADJUSTMENT_LIMIT)
                rot_adjust = STEERING_ADJUSTMENT_LIMIT;
        } else {
            rot_adjust = 0.f;
        }

        rot_output = rotation_controller(rot_adjust);

        left_output_v = fwd_output + rot_output;
        right_output_v = fwd_output - rot_output;

        float tangent_velocity = rotation.velocity * MOUSE_RADIUS * RADIANS_PER_DEGREE;
        float left_speed = forward.velocity + tangent_velocity;
        float right_speed = forward.velocity - tangent_velocity;

        float left_ff = left_ff_voltage(left_speed);
        float right_ff = right_ff_voltage(right_speed);

        // print logic
        if (count % 40 == 0) {
            count = 1;
            // ESP_LOGI(TAG, "mous %.2f fwd %.2f gyr %.2f", mouse_distance, forward.position, mouse_yaw);
            // ESP_LOGI(TAG, "a %.2f r %.2f g %.2f", mouse_angle, rotation.position, mouse_yaw);
            // ESP_LOGI(TAG, "a %.2f y %.2f, R: %d, F: %d, L: %d", rot_adjust, rot_output, right_wall_present, front_wall_present, left_wall_present);
        } 
        else {
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
    // Maze maze;

    while (!gyro_init) {
        vTaskDelay(2);
    }

    // reset_maze(&maze);
    // maze.m_goal = (Pos){7, 7};
    maze.m_mask = MASK_OPEN;
    flood(&maze);
    maze.m_mouse_heading = NORTH;

    Heading bestHeading = NORTH;

    while (true) 
    {
        vTaskDelay(2);

        /*
        if (maze.m_mouse_pos.x != maze.m_goal.x || maze.m_mouse_pos.y != maze.m_goal.y)
        {
            scan_new_walls(&maze);
            ESP_LOGI(TAG, "L %d F %d R %d", left_wall_present, front_wall_present, right_wall_present);

            flood(&maze);
            bestHeading = heading_to_smallest(&maze, maze.m_mouse_pos, maze.m_mouse_heading);

            if (bestHeading == NORTH) {
                maze.m_mouse_pos.y++;
            } else if (bestHeading == EAST) {
                maze.m_mouse_pos.x++;
            } else if (bestHeading == WEST) {
                maze.m_mouse_pos.x--;
            } else if (bestHeading == SOUTH) {
                maze.m_mouse_pos.y--;
            } else {
                ESP_LOGI(TAG, "bad heading");
            }
            maze.m_mouse_heading = bestHeading;
        }
        */

        // print_maze_state(&maze);
        if (test_fwd) {
            // if (left_wall_present) {
            // gpio_set_level(RED_LED, 0);
            // } else {
            // gpio_set_level(RED_LED, 1);
            // }

            // if (right_wall_present) {
            // gpio_set_level(GREEN_LED, 0);
            // } else {
            // gpio_set_level(GREEN_LED, 1);
            // }

            // if (front_wall_present) {
            // gpio_set_level(BLUE_LED, 0);
            // } else {
            // gpio_set_level(BLUE_LED, 1);
            // }
            // move_forward()
            // move_forward(FULL_CELL, SEARCH_VELOCITY, 0, SEARCH_ACCELERATION);
            // turn_smooth(SS90EL);
            search((Pos){0, 2}, true);
            turn_IP180();
            maze.m_mouse_heading = behind_from(maze.m_mouse_heading);
            
            search((Pos){0, 0}, false);
            test_fwd = false;
            // controller_output_enabled = true;

            // reset_maze(&maze);
            // flood(&maze);
            // ESP_LOGI(TAG, "controler output is off");
            // search((Pos){2, 0});
            // gpio_set_level(BLUE_LED, 0);
            // // print_maze_state(&maze);

            // turn_IP180();
            // maze.m_mouse_heading = behind_from(maze.m_mouse_heading);

            // // print_maze_state(&maze);
            // ESP_LOGI(TAG, "search finished");
            // vTaskDelay(500);
            // gpio_set_level(BLUE_LED, 1);
            // ESP_LOGI(TAG, "heading back");   
            // search((Pos){0, 0});    
            // gpio_set_level(BLUE_LED, 0);
            // ESP_LOGI(TAG, "back at start");
            // test_fwd = false;
        }


        if (false) 
        {
            steering_enabled = false;
            Profile_Start(&forward, -BACK_TO_CENTER_DIST, SEARCH_VELOCITY / 4, 0.f, SEARCH_ACCELERATION / 3);
            while(!Profile_Is_Finished(&forward)) { vTaskDelay(1); }
            vTaskDelay(200);

            Profile_Start(&forward, BACK_TO_CENTER_DIST + HALF_CELL, SEARCH_VELOCITY / 2, SEARCH_VELOCITY / 2, SEARCH_ACCELERATION);
            while(!Profile_Is_Finished(&forward)) { vTaskDelay(1); }
            
            ESP_LOGI(TAG, "stopped at center");

            steering_enabled = false;

            // Profile_Start(&forward, FULL_CELL, SEARCH_VELOCITY, 0.f, SEARCH_ACCELERATION);
            // while(!Profile_Is_Finished(&forward)) { vTaskDelay(1); }
            // mouse_distance -= FULL_CELL;

            // Profile_Start(&forward, 300.f, 300.f, 0.f, 1000.f);
            // set_motor_dir(MOTOR_RIGHT, BACKWARD);
            
            // Profile_Start(&forward, 180.f * 2, 400.f, 0.f, 1500.f);

            // Profile_Start(&rotation, 90.f, 300.f, 0.f, 2000.f);
            // while(rotation.state != PS_DONE || forward.state != PS_DONE) 
            // {
            //     vTaskDelay(2);
            // }

            set_target_velocity(&forward, 0);
            test_fwd = false;
        }
    }
}