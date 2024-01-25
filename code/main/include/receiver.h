#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

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
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    /********************** ADC Config **********************/
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };

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

uint32_t perform_adc_conversion() {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, FL_RECV_CHANNEL, &recv_adc_raw[0]));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle_FL_RECV, recv_adc_raw[0], &recv_voltage[0]));

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, FR_RECV_CHANNEL, &recv_adc_raw[1]));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle_FR_RECV, recv_adc_raw[1], &recv_voltage[1]));

    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, DL_RECV_CHANNEL, &recv_adc_raw[2]));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle_DL_RECV, recv_adc_raw[2], &recv_voltage[2]));

    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, DR_RECV_CHANNEL, &recv_adc_raw[3]));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle_DR_RECV, recv_adc_raw[3], &recv_voltage[3]));

    return 0;
}

#endif