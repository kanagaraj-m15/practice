/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_IO           8       // GPIO8 for master SDA
#define I2C_MASTER_SCL_IO           9       // GPIO9 for master SCL
#define I2C_MASTER_NUM              0       // I2C master controller number
#define I2C_MASTER_FREQ_HZ          100000  // I2C master clock frequency

#define I2C_SLAVE_SDA_IO            18      // GPIO18 for slave SDA
#define I2C_SLAVE_SCL_IO            17      // GPIO17 for slave SCL
#define I2C_SLAVE_NUM               1       // I2C slave controller number
#define I2C_SLAVE_ADDR              0x28    // I2C slave address

#define BUF_SIZE                    128

static const char *TAG = "i2c_example";

// I2C master initialization
static esp_err_t i2c_master_init(void)
{
    i2c_config_t master_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &master_config);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

// I2C slave initialization
static esp_err_t i2c_slave_init(void)
{
    i2c_config_t slave_config = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDR,
    };
    
    esp_err_t err = i2c_param_config(I2C_SLAVE_NUM, &slave_config);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_SLAVE_NUM, I2C_MODE_SLAVE, BUF_SIZE, BUF_SIZE, 0);
}

// I2C master task
void i2c_master_task(void *pvParameters)
{
    uint8_t data[2] = {0};
    int counter = 0;
    
    while (1) {
        data[0] = 0x10;  // Command
        data[1] = counter++;  // Data to send
        
        ESP_LOGI(TAG, "Master sending: cmd=0x%02x, data=0x%02x", data[0], data[1]);
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, data, sizeof(data), true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Master write successful");
        } else {
            ESP_LOGE(TAG, "Master write failed: %s", esp_err_to_name(ret));
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));  // Send every 2 seconds
    }
}

// I2C slave task
void i2c_slave_task(void *pvParameters)
{
    uint8_t buffer[BUF_SIZE] = {0};
    
    while (1) {
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, buffer, BUF_SIZE, pdMS_TO_TICKS(1000));
        
        if (len > 0) {
            ESP_LOGI(TAG, "Slave received %d bytes:", len);
            for (int i = 0; i < len; i++) {
                ESP_LOGI(TAG, "0x%02x", buffer[i]);
            }
            
            // Process the received data
            if (len >= 2 && buffer[0] == 0x10) {
                ESP_LOGI(TAG, "Received command 0x10 with data: 0x%02x", buffer[1]);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Check for new data every 100ms
    }
}

void app_main(void)
{
    printf("I2C Master-Slave Example\n");
    
    // Initialize I2C master and slave
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C master initialized successfully");
    
    ESP_ERROR_CHECK(i2c_slave_init());
    ESP_LOGI(TAG, "I2C slave initialized successfully");
    
    // Create tasks for master and slave
    xTaskCreate(i2c_master_task, "i2c_master", 2048, NULL, 10, NULL);
    xTaskCreate(i2c_slave_task, "i2c_slave", 2048, NULL, 10, NULL);
}
