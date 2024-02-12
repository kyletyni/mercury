#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "config.h"

#define I2C_MASTER_0_SCL_IO         10                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_0_SDA_IO         11                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_1_SCL_IO         18                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_1_SDA_IO         17                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_0                I2C_NUM_0                           
#define I2C_MASTER_1                I2C_NUM_1
#define I2C_MASTER_FREQ_HZ          400000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       5000

#define MAX_ENCODER_VALUE 4095

#define AS5600_ADDRESS              0x36
#define AS5600_RAW_ANGLE_H          0x0C

#define MM_PER_COUNT

const char *ENC_TAG = "encoder";
SemaphoreHandle_t encoderCountMutex;

uint8_t count_buffer[4];

uint16_t raw_count_left = 0;
uint16_t raw_count_right = 0;
uint16_t raw_count_left_prev = 0;
uint16_t raw_count_right_prev = 0;

int32_t total_count_left = 0;
int32_t total_count_left_prev = 0;
int32_t total_count_right = 0;
int32_t total_count_right_prev = 0;

float fwd_change;
float fwd_change_prev;
float enc_rot_change;
float delta_left;
float delta_right;

float mouse_distance;
float mouse_angle;

int raw_left_delta;
int raw_right_delta;
int enc_direction;


static esp_err_t encoder_register_read(i2c_port_t i2c_port, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief i2c master initialization
 */
extern "C" esp_err_t i2c_master_init(void) 
{
    ESP_LOGI(ENC_TAG, "conf def");
    i2c_config_t conf0;
    conf0.mode = I2C_MODE_MASTER;
    conf0.sda_io_num = I2C_MASTER_0_SDA_IO;
    conf0.scl_io_num = I2C_MASTER_0_SCL_IO;
    conf0.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf0.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf0.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf0.clk_flags = 0;

    i2c_config_t conf1;
    conf1.mode = I2C_MODE_MASTER;
    conf1.sda_io_num = I2C_MASTER_1_SDA_IO;
    conf1.scl_io_num = I2C_MASTER_1_SCL_IO;
    conf1.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf1.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf1.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf1.clk_flags = 0;

    ESP_LOGI(ENC_TAG, "conf0 init");
    esp_err_t err = i2c_param_config(I2C_MASTER_0, &conf0);
    
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(ENC_TAG, "conf1 init");
    err = i2c_param_config(I2C_MASTER_1, &conf1);

    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(ENC_TAG, "driver install for conf0");
    err = i2c_driver_install(I2C_MASTER_0, conf0.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(ENC_TAG, "driver install for conf1");
    return i2c_driver_install(I2C_MASTER_1, conf1.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    ESP_ERROR_CHECK(encoder_register_read(I2C_NUM_0, AS5600_RAW_ANGLE_H, &count_buffer[0], 2));
    ESP_ERROR_CHECK(encoder_register_read(I2C_NUM_1, AS5600_RAW_ANGLE_H, &count_buffer[2], 2));

    encoderCountMutex = xSemaphoreCreateMutex();   
}


static esp_err_t encoder_register_read(i2c_port_t i2c_port, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(i2c_port, AS5600_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


void reset_encoder_count(void)
{
    // make sure i2c is initialized before beginning angle calc
    ESP_ERROR_CHECK(encoder_register_read(I2C_NUM_0, AS5600_RAW_ANGLE_H, &count_buffer[0], 2));
    ESP_ERROR_CHECK(encoder_register_read(I2C_NUM_1, AS5600_RAW_ANGLE_H, &count_buffer[2], 2));

    raw_count_left = ((0b00001111 & count_buffer[0]) << 8) | count_buffer[1];
    raw_count_right = ((0b00001111 & count_buffer[2]) << 8) | count_buffer[3];
    raw_count_left_prev = raw_count_left;
    raw_count_right_prev = raw_count_right;

    total_count_left = 0;
    total_count_left_prev = 0;
    
    total_count_right = 0;
    total_count_right_prev = 0;

    mouse_distance = 0.f;
    mouse_angle = 0.f;
}


void update_encoders(void)
{
    // make sure i2c is initialized before beginning angle calc
    ESP_ERROR_CHECK(encoder_register_read(I2C_NUM_0, AS5600_RAW_ANGLE_H, &count_buffer[0], 2));
    ESP_ERROR_CHECK(encoder_register_read(I2C_NUM_1, AS5600_RAW_ANGLE_H, &count_buffer[2], 2));

    // lock encoder count variables so that they are not read from while being updated
    // if (xSemaphoreTake(encoderCountMutex, portMAX_DELAY)) {
    raw_count_left_prev = raw_count_left;
    raw_count_right_prev = raw_count_right;
    raw_count_left = ((0b00001111 & count_buffer[0]) << 8) | count_buffer[1];
    raw_count_right = ((0b00001111 & count_buffer[2]) << 8) | count_buffer[3];

    // calculate total for left encoder
    raw_left_delta = raw_count_left - raw_count_left_prev;
    enc_direction = raw_left_delta >= 0 ? 1 : -1;
    total_count_left += (raw_left_delta + enc_direction * MAX_ENCODER_VALUE / 2) % MAX_ENCODER_VALUE - enc_direction * MAX_ENCODER_VALUE / 2;    
    
    // calculate total for right encoder
    raw_right_delta = raw_count_right - raw_count_right_prev;
    enc_direction = raw_right_delta >= 0 ? 1 : -1;
    total_count_right += (raw_right_delta + enc_direction * MAX_ENCODER_VALUE / 2) % MAX_ENCODER_VALUE - enc_direction * MAX_ENCODER_VALUE / 2;    
}


void update_distances(void) 
{
    delta_left = (total_count_left - total_count_left_prev) * MM_PER_COUNT_RIGHT;
    delta_right = -(total_count_right - total_count_right_prev) * MM_PER_COUNT_LEFT;
    fwd_change = 0.5f * (delta_left + delta_right);
    enc_rot_change = (delta_left - delta_right) * DEG_PER_MM_DIFFERENCE;
    
    total_count_right_prev = total_count_right;
    total_count_left_prev  = total_count_left;

    mouse_distance += fwd_change;
    mouse_angle += enc_rot_change;
}


#endif