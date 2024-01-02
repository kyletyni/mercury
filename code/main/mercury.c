#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

static const char *TAG = "i2c-encoder";

#define I2C_MASTER_0_SCL_IO         15                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_0_SDA_IO         16                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_1_SCL_IO         17                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_1_SDA_IO         18                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_0                0                           /*!< I2C master i2c port number */
#define I2C_MASTER_1                1
#define I2C_MASTER_FREQ_HZ          400000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       5000


#define AS5600_ADDRESS              0x36
#define AS5600_RAW_ANGLE_H          0x0C

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void) 
{
    i2c_config_t conf0 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_0_SDA_IO,
        .scl_io_num = I2C_MASTER_0_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };


    i2c_config_t conf1 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_1_SDA_IO,
        .scl_io_num = I2C_MASTER_1_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_0, &conf0);
    
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_param_config(I2C_MASTER_1, &conf1);

    if (err != ESP_OK) {
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_0, conf0.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_1, conf1.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t encoder_register_read(uint8_t i2c_port, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(i2c_port, AS5600_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void get_encoder_angles(void)
{
    uint8_t data[4];
    uint16_t raw_angle0 = 0;
    uint16_t raw_angle1 = 0;

    // make sure i2c is initialized before beginning angle calc
    ESP_ERROR_CHECK(encoder_register_read(0, AS5600_RAW_ANGLE_H, data, 2));
    ESP_ERROR_CHECK(encoder_register_read(1, AS5600_RAW_ANGLE_H, &data[3], 2));

    
    raw_angle0 = ((0b00001111 & data[0]) << 8) | data[1];
    raw_angle1 = ((0b00001111 & data[3]) << 8) | data[4];
    ESP_LOGI(TAG, "data0 = %4d, data1 = %4d", raw_angle0, raw_angle1);
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    while (true)
    {

        
        vTaskDelay(1);
    }

}
