#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"


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

void set_pwm_dc(ledc_channel_t channel, float duty);

void init_motors(void) {
    fwd_error = 0.f;
    rot_error = 0.f;
    prev_fwd_error = 0.f;
    prev_rot_error = 0.f;

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

    set_pwm_dc(MR_PWMA_CHANNEL, 0.f);

    ledc_channel.gpio_num = MR_PWMB_GPIO;
    ledc_channel.channel  = LEDC_CHANNEL_1;
    ledc_channel_config(&ledc_channel);
    set_pwm_dc(MR_PWMB_CHANNEL, 0.f);
    
    ledc_channel.gpio_num = ML_PWMA_GPIO;
    ledc_channel.channel  = LEDC_CHANNEL_2;
    ledc_channel_config(&ledc_channel);
    set_pwm_dc(ML_PWMA_CHANNEL, 0.f);
    
    ledc_channel.gpio_num = ML_PWMB_GPIO;
    ledc_channel.channel  = LEDC_CHANNEL_3;
    ledc_channel_config(&ledc_channel);
    set_pwm_dc(ML_PWMB_CHANNEL, 0.f);
};

/// @brief sets the pwm duty cycle of a certain signal, must make sure duty is btwn 0 and 100 
void set_pwm_dc(ledc_channel_t channel, float duty)
{

    float normalized_value = duty / 100.f;
    uint32_t duty_val = (uint16_t)((1 << 12) - normalized_value * (1 << 12));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty_val));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
};

void set_dir(MotorSide side, MotorDir dir)
{
    if (side == MOTOR_RIGHT) 
        right_dir = dir;
    else
        left_dir = dir;
};

#endif