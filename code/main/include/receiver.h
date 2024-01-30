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

int recv_adc_raw[4];
int recv_voltage[4];

const char *TAG1 = "main2";

adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t adc_cali_handle_FL_RECV = NULL;
adc_cali_handle_t adc_cali_handle_FR_RECV = NULL;
adc_cali_handle_t adc_cali_handle_DL_RECV = NULL;
adc_cali_handle_t adc_cali_handle_DR_RECV = NULL;

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
}

typedef enum {
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    DIAGONAL_RIGHT,
    DIAGONAL_LEFT
} ir_dir;

void ir_sensor_poll(ir_dir sensor) {
    // ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle_FL_RECV, recv_adc_raw[FRONT_LEFT], &recv_voltage[0]));
    uint16_t avg_count = 0;

    switch (sensor) {
        case FRONT_LEFT:
            gpio_set_level(EMIT_FL, 1);

            for (uint8_t i = 0; i < 15; i++) {
                ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, FL_RECV_CHANNEL, &recv_adc_raw[FRONT_LEFT]));
                avg_count = avg_count + recv_adc_raw[FRONT_LEFT];
            }
            gpio_set_level(EMIT_FL, 0);

            recv_adc_raw[FRONT_LEFT] = avg_count / 15;
            break;

        case FRONT_RIGHT:
            gpio_set_level(EMIT_FR, 1);
            for (uint8_t i = 0; i < 15; i++) {
                ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, FR_RECV_CHANNEL, &recv_adc_raw[FRONT_RIGHT]));
                avg_count = avg_count + recv_adc_raw[FRONT_RIGHT];
            }
            gpio_set_level(EMIT_FR, 0);

            recv_adc_raw[FRONT_RIGHT] = avg_count / 15;
            break;

        case DIAGONAL_LEFT:
            gpio_set_level(EMIT_DL, 1);
            for (uint8_t i = 0; i < 15; i++) {
                ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, DL_RECV_CHANNEL, &recv_adc_raw[DIAGONAL_LEFT]));
                avg_count = avg_count + recv_adc_raw[DIAGONAL_LEFT];
            }
            gpio_set_level(EMIT_DL, 0);

            recv_adc_raw[DIAGONAL_LEFT] = avg_count / 15;
            break;

        case DIAGONAL_RIGHT:
            gpio_set_level(EMIT_DR, 1);
            for (uint8_t i = 0; i < 15; i++) {
                ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, DR_RECV_CHANNEL, &recv_adc_raw[DIAGONAL_RIGHT]));
                avg_count = avg_count + recv_adc_raw[DIAGONAL_RIGHT];
            }
            gpio_set_level(EMIT_DR, 0);

            recv_adc_raw[DIAGONAL_RIGHT] = avg_count / 15;
            break;
        
        default:
            break;
    }

    return;
}

#endif