#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "emitter.h"

#define FL_RECV_CHANNEL ADC_CHANNEL_0  
#define FR_RECV_CHANNEL ADC_CHANNEL_8  
#define DL_RECV_CHANNEL ADC_CHANNEL_3
#define DR_RECV_CHANNEL ADC_CHANNEL_1

#define NUM_OF_POLLS 10

int recv_adc_raw_val[4][NUM_OF_POLLS];
int recv_adc_raw[4];
uint16_t recv_adc_raw_idx[4];
float rec_adc_raw_sum[4];
int recv_avg_val[4];

float front_sensor_sum;
float front_sensor_diff;

typedef enum {
    SM_SIDE_BASED = 0,
    SM_FRONT_BASED
} SteeringMode;

SteeringMode steering_mode = SM_SIDE_BASED;

const char *TAG1 = "receiver";

adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t adc_cali_handle_FL_RECV = NULL;
adc_cali_handle_t adc_cali_handle_FR_RECV = NULL;
adc_cali_handle_t adc_cali_handle_DL_RECV = NULL;
adc_cali_handle_t adc_cali_handle_DR_RECV = NULL;

bool right_wall_present;
bool left_wall_present;
bool front_wall_present;

float cross_track_error;
float cross_track_error_prev;


void initialize_adc() {
    /********************** ADC Unit Init **********************/
    adc_oneshot_unit_init_cfg_t init_config1;
    init_config1.unit_id = ADC_UNIT_1;
    init_config1.clk_src = (adc_oneshot_clk_src_t)0;
    init_config1.ulp_mode = ADC_ULP_MODE_DISABLE;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_unit_init_cfg_t init_config2;
    init_config2.unit_id = ADC_UNIT_2;
    init_config2.clk_src = (adc_oneshot_clk_src_t)0;
    init_config2.ulp_mode = ADC_ULP_MODE_DISABLE;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    /********************** ADC Config **********************/
    adc_oneshot_chan_cfg_t config;
    config.bitwidth = ADC_BITWIDTH_DEFAULT;
    config.atten = ADC_ATTEN_DB_11;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, FL_RECV_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, FR_RECV_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, DL_RECV_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, DR_RECV_CHANNEL, &config));

    /********************** ADC Calibration **********************/
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = FL_RECV_CHANNEL,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle_FL_RECV));
    
    cali_config.chan = FR_RECV_CHANNEL;
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle_FR_RECV));

    cali_config.unit_id = ADC_UNIT_2;
    cali_config.chan = DL_RECV_CHANNEL;
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle_DL_RECV));
    
    cali_config.chan = DR_RECV_CHANNEL;
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle_DR_RECV));

    for(uint8_t i = 0; i < 4; i++) {
        recv_adc_raw_idx[i] = 0;
        rec_adc_raw_sum[i] = 0;
        recv_avg_val[i] = 0;
    }
}

typedef enum {
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    DIAGONAL_RIGHT,
    DIAGONAL_LEFT
} ir_dir;

void ir_sensor_poll(ir_dir sensor) {
    uint16_t raw_val = 0;

    switch (sensor) {
        case FRONT_LEFT:
            gpio_set_level(EMIT_FL, 1);
            rec_adc_raw_sum[FRONT_LEFT] -=  FRONT_LEFT_SCALE * recv_adc_raw_val[FRONT_LEFT][recv_adc_raw_idx[FRONT_LEFT]];
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, FL_RECV_CHANNEL, &recv_adc_raw_val[FRONT_LEFT][recv_adc_raw_idx[FRONT_LEFT]]));
            rec_adc_raw_sum[FRONT_LEFT] +=  FRONT_LEFT_SCALE * recv_adc_raw_val[FRONT_LEFT][recv_adc_raw_idx[FRONT_LEFT]];
            recv_adc_raw_idx[FRONT_LEFT] = (recv_adc_raw_idx[FRONT_LEFT] + 1) % NUM_OF_POLLS;
            recv_avg_val[FRONT_LEFT] = rec_adc_raw_sum[FRONT_LEFT] / NUM_OF_POLLS;
            gpio_set_level(EMIT_FL, 0);
            break;
        case FRONT_RIGHT:
            gpio_set_level(EMIT_FR, 1);
            rec_adc_raw_sum[FRONT_RIGHT] -=  FRONT_RIGHT_SCALE * recv_adc_raw_val[FRONT_RIGHT][recv_adc_raw_idx[FRONT_RIGHT]];
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, FR_RECV_CHANNEL, &recv_adc_raw_val[FRONT_RIGHT][recv_adc_raw_idx[FRONT_RIGHT]]));
            rec_adc_raw_sum[FRONT_RIGHT] +=  FRONT_RIGHT_SCALE * recv_adc_raw_val[FRONT_RIGHT][recv_adc_raw_idx[FRONT_RIGHT]];
            recv_adc_raw_idx[FRONT_RIGHT] = (recv_adc_raw_idx[FRONT_RIGHT] + 1) % NUM_OF_POLLS;
            recv_avg_val[FRONT_RIGHT] = rec_adc_raw_sum[FRONT_RIGHT] / NUM_OF_POLLS;
            gpio_set_level(EMIT_FR, 0);
            break;
        case DIAGONAL_LEFT:
            gpio_set_level(EMIT_DL, 1);
            rec_adc_raw_sum[DIAGONAL_LEFT] -=  DIAG_LEFT_SCALE * recv_adc_raw_val[DIAGONAL_LEFT][recv_adc_raw_idx[DIAGONAL_LEFT]];
            ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, DL_RECV_CHANNEL, &recv_adc_raw_val[DIAGONAL_LEFT][recv_adc_raw_idx[DIAGONAL_LEFT]]));
            rec_adc_raw_sum[DIAGONAL_LEFT] +=  DIAG_LEFT_SCALE * recv_adc_raw_val[DIAGONAL_LEFT][recv_adc_raw_idx[DIAGONAL_LEFT]];
            recv_adc_raw_idx[DIAGONAL_LEFT] = (recv_adc_raw_idx[DIAGONAL_LEFT] + 1) % NUM_OF_POLLS;
            recv_avg_val[DIAGONAL_LEFT] = rec_adc_raw_sum[DIAGONAL_LEFT] / NUM_OF_POLLS;
            gpio_set_level(EMIT_DL, 0);
            break;
        case DIAGONAL_RIGHT:
            gpio_set_level(EMIT_DR, 1);
            rec_adc_raw_sum[DIAGONAL_RIGHT] -=  DIAG_RIGHT_SCALE * recv_adc_raw_val[DIAGONAL_RIGHT][recv_adc_raw_idx[DIAGONAL_RIGHT]];
            ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, DR_RECV_CHANNEL, &recv_adc_raw_val[DIAGONAL_RIGHT][recv_adc_raw_idx[DIAGONAL_RIGHT]]));
            rec_adc_raw_sum[DIAGONAL_RIGHT] +=  DIAG_RIGHT_SCALE * recv_adc_raw_val[DIAGONAL_RIGHT][recv_adc_raw_idx[DIAGONAL_RIGHT]];
            recv_adc_raw_idx[DIAGONAL_RIGHT] = (recv_adc_raw_idx[DIAGONAL_RIGHT] + 1) % NUM_OF_POLLS;
            recv_avg_val[DIAGONAL_RIGHT] = rec_adc_raw_sum[DIAGONAL_RIGHT] / NUM_OF_POLLS;
            gpio_set_level(EMIT_DR, 0);
            break;
        default:
            break;
    }    
    return;
}

void calc_cross_track_error()
{
    front_sensor_sum = recv_avg_val[FRONT_LEFT] + recv_avg_val[FRONT_RIGHT];
    front_sensor_diff = recv_avg_val[FRONT_LEFT] - recv_avg_val[FRONT_RIGHT];

    right_wall_present = recv_avg_val[DIAGONAL_RIGHT] > RIGHT_THRESHOLD;
    left_wall_present = recv_avg_val[DIAGONAL_LEFT] > LEFT_THRESHOLD;
    front_wall_present = front_sensor_sum > FRONT_THRESHOLD;

    // calculate cross track error
    static int error = 0;
	int right_error = SIDE_NOMINAL - recv_avg_val[DIAGONAL_RIGHT];
	int left_error = SIDE_NOMINAL - recv_avg_val[DIAGONAL_LEFT];

    // if (steering_mode)

    error = 0;
    if (left_wall_present && right_wall_present)
    {
        error = right_error - left_error;
    }
    else if (left_wall_present)
    {
        error = -2 * left_error;
    }
    else if (right_wall_present)
    {
        error = 2 * right_error;
    }

    cross_track_error = error;  
}
#endif