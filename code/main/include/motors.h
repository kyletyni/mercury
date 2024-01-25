#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"


#define MR_PWMA_GPIO GPIO_NUM_38
#define MR_PWMB_GPIO GPIO_NUM_8
#define ML_PWMA_GPIO GPIO_NUM_36
#define ML_PWMB_GPIO GPIO_NUM_37   

#define PWM_FREQUENCY 20000

bool controller_enabled;

float fwd_error;
float rot_error;
float prev_fwd_error;
float prev_rot_error;

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} MotorSide;

typedef enum {
    FORWARD = 0,
    BACKWARD
} MotorDir;

MotorDir left_dir;
MotorDir right_dir;

float left_voltage;
float right_voltage;

void init_motors(void) {
    fwd_error = 0.f;
    rot_error = 0.f;
    prev_fwd_error = 0.f;
    prev_rot_error = 0.f;

    // PWM Timer Configuration
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_14_BIT,   // resolution of PWM duty
        .freq_hz = PWM_FREQUENCY,               // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,      // timer mode
        .timer_num = LEDC_TIMER_0,             // timer index
        .clk_cfg = LEDC_AUTO_CLK,               // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);

    // PWM Channel Configuration
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = MR_PWMA_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);

    ledc_channel.gpio_num = MR_PWMB_GPIO;
    ledc_channel.channel  = LEDC_CHANNEL_1;
    ledc_channel_config(&ledc_channel);
    
    ledc_channel.gpio_num = ML_PWMA_GPIO;
    ledc_channel.channel  = LEDC_CHANNEL_2;
    ledc_channel_config(&ledc_channel);
    
    ledc_channel.gpio_num = ML_PWMB_GPIO;
    ledc_channel.channel  = LEDC_CHANNEL_3;
    ledc_channel_config(&ledc_channel);
};


void set_pwm_dc(gpio_num_t pin, uint16_t duty)
{

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 16383 / 2));
};

void set_dir(MotorSide side, MotorDir dir)
{
    if (side == MOTOR_RIGHT) 
        right_dir = dir;
    else
        left_dir = dir;
};

#endif