#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "profile.h"
#include "config.h"
#include "gyro.h"

#define MR_PWMA_GPIO GPIO_NUM_38
#define MR_PWMA_CHANNEL LEDC_CHANNEL_0

#define MR_PWMB_GPIO GPIO_NUM_8
#define MR_PWMB_CHANNEL LEDC_CHANNEL_1

#define ML_PWMA_GPIO GPIO_NUM_36
#define ML_PWMA_CHANNEL LEDC_CHANNEL_2

#define ML_PWMB_GPIO GPIO_NUM_37
#define ML_PWMB_CHANNEL LEDC_CHANNEL_3   

#define PWM_FREQUENCY 15000

bool controller_enabled;

float fwd_error;
float fwd_error_prev;

float rot_error;
float rot_error_prev;

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} MotorSide;

typedef enum {
    FORWARD = 0,
    BACKWARD
} MotorDir;

MotorDir left_motor_dir_cur;
MotorDir right_motor_dir_cur;
MotorDir left_motor_dir_target;
MotorDir right_motor_dir_target;

float left_motor_voltage;
float right_motor_voltage;

bool controller_output_enabled;
bool steering_enabled = false;

static void set_motor_pwm_dc(ledc_channel_t channel, float duty);
void set_motor_dir(MotorSide side, MotorDir dir);

void init_motors(void) {
    fwd_error = 0.f;
    rot_error = 0.f;
    fwd_error_prev = 0.f;
    rot_error_prev = 0.f;

    ledc_fade_func_install(0);

    // PWM Timer Configuration
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;      // timer mode
    ledc_timer.duty_resolution = LEDC_TIMER_12_BIT;   // resolution of PWM duty
    ledc_timer.timer_num = LEDC_TIMER_0;             // timer index
    ledc_timer.freq_hz = PWM_FREQUENCY;               // frequency of PWM signal
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;               // Auto select the source clock
    ledc_timer_config(&ledc_timer);

    // PWM Channel Configuration
    ledc_channel_config_t ledc_channel;
    ledc_channel.channel    = LEDC_CHANNEL_0;
    ledc_channel.duty       = 0;
    ledc_channel.gpio_num   = MR_PWMA_GPIO;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.hpoint     = 0;
    ledc_channel.timer_sel  = LEDC_TIMER_0;
    ledc_channel_config(&ledc_channel);

    set_motor_pwm_dc(MR_PWMA_CHANNEL, 0.f);

    ledc_channel.gpio_num = MR_PWMB_GPIO;
    ledc_channel.channel  = LEDC_CHANNEL_1;
    ledc_channel_config(&ledc_channel);
    set_motor_pwm_dc(MR_PWMB_CHANNEL, 0.f);
    
    ledc_channel.gpio_num = ML_PWMA_GPIO;
    ledc_channel.channel  = LEDC_CHANNEL_2;
    ledc_channel_config(&ledc_channel);
    set_motor_pwm_dc(ML_PWMA_CHANNEL, 0.f);
    
    ledc_channel.gpio_num = ML_PWMB_GPIO;
    ledc_channel.channel  = LEDC_CHANNEL_3;
    ledc_channel_config(&ledc_channel);
    set_motor_pwm_dc(ML_PWMB_CHANNEL, 0.f);

    controller_output_enabled = false;

    return;
};

/// @brief sets the pwm duty cycle of a certain signal, must make sure duty is btwn 0 and 100 
static void set_motor_pwm_dc(ledc_channel_t channel, float duty)
{
    float normalized_value = duty / 100.f;
    uint32_t duty_val = (uint16_t)((1 << 12) - normalized_value * (1 << 12));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty_val));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));

    return;
};

void set_motor_voltage(MotorSide side, float voltage)
{
    if (voltage >= 0) {
        set_motor_dir(side, FORWARD);
    }
    else {
        set_motor_dir(side, BACKWARD);
    }

	voltage = fabsf(voltage);
    if (voltage > 5.5f) {
        voltage = 5.5f;
    }


    float duty_cycle = voltage / 6.f * 100.f;
    if (side == MOTOR_LEFT) {
        left_motor_voltage = voltage;
        // check to see if the current direction matches the target direction
        if(left_motor_dir_target != left_motor_dir_cur) {
            set_motor_pwm_dc(ML_PWMA_CHANNEL, 0.f);
            set_motor_pwm_dc(ML_PWMB_CHANNEL, 0.f);
            left_motor_dir_cur = left_motor_dir_target;
        } 
        // Set the correct PWM channel for direction
        if (left_motor_dir_cur == FORWARD) {
            set_motor_pwm_dc(ML_PWMA_CHANNEL, 0.f);
            set_motor_pwm_dc(ML_PWMB_CHANNEL, duty_cycle);
        } else {
            set_motor_pwm_dc(ML_PWMB_CHANNEL, 0.f);
            set_motor_pwm_dc(ML_PWMA_CHANNEL, duty_cycle);
        }
    }
    
    // Handle logic for the right motor
    else {
        right_motor_voltage = voltage;
        if(right_motor_dir_target != right_motor_dir_cur) {
            set_motor_pwm_dc(MR_PWMA_CHANNEL, 0.f);
            set_motor_pwm_dc(MR_PWMB_CHANNEL, 0.f);
            right_motor_dir_cur = right_motor_dir_target;
        } 
        // Set the correct PWM channel for direction
        if (right_motor_dir_cur == FORWARD) {
            set_motor_pwm_dc(MR_PWMA_CHANNEL, 0.f);
            set_motor_pwm_dc(MR_PWMB_CHANNEL, duty_cycle);
        } else {
            set_motor_pwm_dc(MR_PWMB_CHANNEL, 0.f);
            set_motor_pwm_dc(MR_PWMA_CHANNEL, duty_cycle);
        }
    }

    return;
};

void set_motor_dir(MotorSide side, MotorDir dir)
{
    if (side == MOTOR_RIGHT) 
        right_motor_dir_target = dir;
    else
        left_motor_dir_target = dir;
};

float left_ff_voltage(float speed) {
    static float oldSpeed = 0;
    float leftFF = speed * FF_VOLTAGE + BIAS_VOLTAGE;
    float acc = (speed - oldSpeed) * LOOP_FREQUENCY;
    oldSpeed = speed;
    float accFF = FF_ACC_VOLTAGE * acc;
    leftFF += accFF;
    return leftFF;
}

float right_ff_voltage(float speed) {
    static float oldSpeed = 0;
    float rightFF = speed * FF_VOLTAGE + BIAS_VOLTAGE;
    float acc = (speed - oldSpeed) * LOOP_FREQUENCY;
    oldSpeed = speed;
    float accFF = FF_ACC_VOLTAGE * acc;
    rightFF += accFF;
    return rightFF;
}

float forward_controller(void) 
{
    static float output_prev = 0.f;
    float increment = forward.velocity * LOOP_INTERVAL;
    fwd_error += increment - fwd_change;
    fwd_change = 0;
    // fwd_change_prev = fwd_change;
    float diff = fwd_error - fwd_error_prev;
    fwd_error_prev = fwd_error;
    float output = (FWD_KP * fwd_error + FWD_KD * diff) * 0.8f + 0.2f * output_prev;
    output_prev = output;
    
    return output;
};

float rotation_controller(float adjustment) 
{
    static float output_prev = 0.f;
    float increment = rotation.velocity * LOOP_INTERVAL;
    // if (xSemaphoreTake(encoderCountMutex, portMAX_DELAY)) {
    rot_error += increment - enc_rot_change;
    enc_rot_change = 0;
        // xSemaphoreGive(encoderCountMutex);
    // }
    // rot_error += increment - yaw_change * 1.5;
    rot_error += adjustment;
    float diff = rot_error - rot_error_prev;
    rot_error_prev = rot_error;
    float output = (ROT_KP * rot_error + ROT_KD * diff) * 0.9f + 0.1f * output_prev;;
    return output;
};

#endif