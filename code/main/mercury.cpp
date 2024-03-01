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
    // xTaskCreatePinnedToCore(&mpu6500, "gyroTask", 4096, NULL, 1, NULL, 0);
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
        
        // calculate cross track error
        calc_cross_track_error();

        // if no walls, we should use gyro

        if (count % 40 == 0) {
            // if(!Profile_Is_Finished(&forward)) {
                // ESP_LOGI(TAG, "DL: %4d, DR: %4d, POS: %.2f", recv_avg_val[DIAGONAL_LEFT], recv_avg_val[DIAGONAL_RIGHT], forward.position    );
            // }
            // ESP_LOGI(TAG, "FR: %4d, DR: %4d, DL: %4d, FL: %4d, FRONT: %.2f, DE: %.2f", recv_avg_val[FRONT_RIGHT], recv_avg_val[DIAGONAL_RIGHT], recv_avg_val[DIAGONAL_LEFT], recv_avg_val[FRONT_LEFT], front_sensor_sum, front_dist_error);
            // ESP_LOGI(TAG, "FR: %.2f, DR: %.2f, DL: %.2f, FL: %.2f, FRONT: %.2f", rec_adc_raw_sum[FRONT_RIGHT], rec_adc_raw_sum[DIAGONAL_RIGHT], rec_adc_raw_sum[DIAGONAL_LEFT], rec_adc_raw_sum[FRONT_LEFT], front_sensor_sum);
            // ESP_LOGI(TAG, "FR: %4d, DR: %4d, DL: %4d, FL: %4d, FRONT: %.2f", recv_adc_raw[FRONT_RIGHT], recv_adc_raw[DIAGONAL_RIGHT], recv_adc_raw[DIAGONAL_LEFT], recv_adc_raw[FRONT_LEFT], front_sensor_sum);

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


        if (steering_mode == SM_FRONT_BASED) {
            fwd_output = front_dist_controller();
        } else {
            fwd_output = forward_controller();
        }

        // calculate rotational adjustment
        static float rot_adjust = 0.f;

        if (steering_enabled) {
            if (steering_mode == SM_SIDE_BASED) {
                rot_adjust = (STEERING_KP * cross_track_error) + (STEERING_KD * cross_track_error - cross_track_error_prev);
            } else {
                rot_adjust = (FRONT_STEERING_KP * front_align_error);

            }
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
            // ESP_LOGI(TAG, "f_err %.2f f %.2f", front_dist_error, front_sensor_sum);
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

    bool test_fwd = true;

    // while (!gyro_init) {
    //     vTaskDelay(2);
    // }

    user_select_mode = SEARCH_MODE_SELECT;

    maze.m_mask = MASK_OPEN;
    reset_maze(&maze);
    flood(&maze);
    maze.m_mouse_heading = NORTH;
    maze.m_mouse_pos = (Pos){0, 0};
    Heading bestHeading = NORTH;

    while (true) 
    {
        if (user_active_mode == SELECT_MODE_ACTIVE) {
            gpio_set_level(RED_LED, 1);


            if (gpio_get_level(BTN2) == 0) {
                user_select_mode = (UserSelectMode)((user_select_mode + 1) % SELECT_MENU_SIZE);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }

            if (gpio_get_level(BTN1) == 0) {
                user_active_mode = (UserActiveMode)((user_active_mode + 1) % ACTIVE_MODE_SIZE);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }

            switch (user_select_mode)
            {
                case SEARCH_MODE_SELECT:
                    gpio_set_level(GREEN_LED, 0);
                    gpio_set_level(BLUE_LED, 1);
                    break;
                case SPEED_MODE_SELECT:
                    gpio_set_level(GREEN_LED, 1);
                    gpio_set_level(BLUE_LED, 0);
                    break;
                default:
                    break;
            }
        } else {
            gpio_set_level(RED_LED, 0);
            
            if (gpio_get_level(BTN1) == 0) {
                user_active_mode = (UserActiveMode)((user_active_mode + 1) % ACTIVE_MODE_SIZE);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                continue;
            }

            // check for arming based on front right and diag right sensor vals
            if (recv_avg_val[DIAGONAL_RIGHT] > 150 && recv_avg_val[FRONT_RIGHT] > 250)
            {
                ESP_LOGI(TAG, "armed and starting");
                while(true)
                {
                    gpio_set_level(RED_LED, 1);
                    gpio_set_level(BLUE_LED, 1);
                    gpio_set_level(GREEN_LED, 1);

                    if (user_select_mode == SEARCH_MODE_SELECT) {
                        ESP_LOGI(TAG, "starting search");
                        vTaskDelay(2000);

                        // while (true)
                        // {
                        //     if (left_wall_present) {
                        //         gpio_set_level(RED_LED, 0);
                        //     } else {
                        //         gpio_set_level(RED_LED, 1);
                        //     }

                        //     if (right_wall_present) {
                        //         gpio_set_level(GREEN_LED, 0);
                        //     } else {
                        //         gpio_set_level(GREEN_LED, 1);
                        //     }

                        //     if (front_wall_present) {
                        //         gpio_set_level(BLUE_LED, 0);
                        //     } else {
                        //         gpio_set_level(BLUE_LED, 1);
                        //     }
                        //   vTaskDelay(2);
                        // }

                        // align_with_front_wall();
                        maze.m_mask = MASK_OPEN;
                        search((Pos){7, 7}, true);

                        // print_maze_state(&maze);

                        vTaskDelay(1000);

                        maze.m_goal = (Pos){0, 0};
                        flood(&maze);
                        Heading bestHeading = heading_to_smallest(&maze, maze.m_mouse_pos, maze.m_mouse_heading);
                        uint8_t headingChange = ((bestHeading - maze.m_mouse_heading) >> 1) & 0x3;
                        maze.m_mouse_heading = bestHeading;

                        
                        // print_maze_state(&maze);

                        switch (headingChange) {
                            case RIGHT:
                                turn_IP(90.f, 287.f, 2850.f);
                                break;

                            case LEFT:
                                turn_IP(-90.f, 287.f, 2850.f);
                                break;

                            case BACK:
                                turn_IP(180.f, 287.f, 2850.f);
                                break;

                            default:
                                break;
                        }
                        search((Pos){0, 0}, false);


                        maze.m_mouse_pos = (Pos){0, 0};
                        maze.m_mouse_heading = NORTH;
                        user_active_mode = SELECT_MODE_ACTIVE;
                        break;
                    } else if (user_select_mode == SPEED_MODE_SELECT) {
                        // speed mode, must perform search before utilizing this mode
                        gpio_set_level(RED_LED, 1);
                        gpio_set_level(BLUE_LED, 1);
                        gpio_set_level(GREEN_LED, 1);

                        ESP_LOGI(TAG, "starting search");
                        vTaskDelay(2000);

                        // align_with_front_wall();
                        maze.m_mask = MASK_OPEN;
                        basic_search((Pos){7, 7}, true);

                        print_maze_state(&maze);
                        vTaskDelay(1000);

                        maze.m_goal = (Pos){0, 0};
                        flood(&maze);
                        Heading bestHeading = heading_to_smallest(&maze, maze.m_mouse_pos, maze.m_mouse_heading);
                        uint8_t headingChange = ((bestHeading - maze.m_mouse_heading) >> 1) & 0x3;
                        maze.m_mouse_heading = bestHeading;

                        
                        // print_maze_state(&maze);

                        switch (headingChange) {
                            case RIGHT:
                                turn_IP(90.f, 287.f, 2850.f);
                                break;

                            case LEFT:
                                turn_IP(-90.f, 287.f, 2850.f);
                                break;

                            case BACK:
                                turn_IP(180.f, 287.f, 2850.f);
                                break;

                            default:
                                break;
                        }
                        search((Pos){0, 0}, false);


                        maze.m_mouse_pos = (Pos){0, 0};
                        maze.m_mouse_heading = NORTH;
                        user_active_mode = SELECT_MODE_ACTIVE;
                        break;
                    }
                        

                        // ESP_LOGI(TAG, "starting speed");
                        // maze.m_mask = MASK_CLOSED;

                        // int num_moves = generate_speed_moves((Pos){7, 7});
                        // float end_speed = 0.f;
                        
                        // Profile_Start(&forward, -HALF_CELL, SEARCH_VELOCITY / 4, 0.f, SEARCH_ACCELERATION / 3);
                        // while(!Profile_Is_Finished(&forward)) { vTaskDelay(1); }
                        // vTaskDelay(200);

                        // for(uint8_t i = 0; i < num_moves; i++)
                        // {
                        //     // check next movement for end speed
                        //     if (i + 1 < num_moves) {
                        //         if (speed_movements[i + 1].type != SS_FWD) {
                        //             end_speed = SEARCH_TURN_SPEED;   
                        //         }
                        //     }

                        //     if (speed_movements[i].type == SS_FWD)
                        //     {
                        //         if (i == 0) {
                        //             move_forward(speed_movements[i].num * FULL_CELL - BACK_TO_CENTER_DIST, SPEED_VELOCITY, SEARCH_TURN_SPEED, SPEED_ACCELERATION);
                        //         } else {
                        //             move_forward(speed_movements[i].num * FULL_CELL, SPEED_VELOCITY, SEARCH_TURN_SPEED, SPEED_ACCELERATION);
                        //         }
                        //     } else if (speed_movements[i].type == SS90EL) {
                        //         for (uint8_t j = 0; j < speed_movements[i].num; j++) {
                        //             turn_smooth(SS90EL);
                        //         }
                        //     } else if (speed_movements[i].type == SS90ER) {
                        //         for (uint8_t j = 0; j < speed_movements[i].num; j++) {
                        //             turn_smooth(SS90ER);
                        //         }
                        //     } else {
                        //         // shouldn't get here
                        //     }
                        // }

                        // stop_at_center();
                    
                    // vTaskDelay(1500);

                    // search((Pos){0, 0}, false);

                    vTaskDelay(2);
                    // break;
                }
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);

        // search((Pos){0, 2}, true);
        // vTaskDelay(300);
        // search((Pos){0, 0}, false);


    }
}