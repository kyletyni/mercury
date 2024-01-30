#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2c.h"
#include "driver/gpio.h"

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

#define AS5600_ADDRESS              0x36
#define AS5600_RAW_ANGLE_H          0x0C

const char *ENC_TAG = "encoder";
SemaphoreHandle_t encoderCountMutex;

uint8_t count_buffer[4];
uint16_t raw_count_left = 0;
uint16_t raw_count_right = 0;

float fwd_change;
float mouse_distance;

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

    encoderCountMutex = xSemaphoreCreateMutex();   
}

static esp_err_t encoder_register_read(i2c_port_t i2c_port, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(i2c_port, AS5600_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void get_raw_encoder_count(void)
{
    // make sure i2c is initialized before beginning angle calc
    ESP_ERROR_CHECK(encoder_register_read(I2C_NUM_0, AS5600_RAW_ANGLE_H, &count_buffer[0], 2));
    ESP_ERROR_CHECK(encoder_register_read(I2C_NUM_1, AS5600_RAW_ANGLE_H, &count_buffer[2], 2));

    // lock encoder count variables so that they are not read from while being updated
    // if (xSemaphoreTake(encoderCountMutex, portMAX_DELAY)) {
    raw_count_right = ((0b00001111 & count_buffer[0]) << 8) | count_buffer[1];
    raw_count_left = ((0b00001111 & count_buffer[2]) << 8) | count_buffer[3];
        // xSemaphoreGive(encoderCountMutex);
    // }
    // ESP_LOGI(TAG, "data0 = %4d, data1 = %4d", raw_angle0, raw_angle1);
}


#endif
