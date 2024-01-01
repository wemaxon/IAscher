/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "led.h"
#include "iot_servo.h"
#include "imu/imu.h"


#define SERVO1_PIN           20
#define SERVO2_PIN           21

#define I2C_MASTER_SCL_IO    3    // Define the SCL pin
#define I2C_MASTER_SDA_IO    2    // Define the SDA pin
#define I2C_MASTER_NUM       I2C_NUM_0  // I2C port number
#define I2C_MASTER_FREQ_HZ   100000    // I2C master clock frequency
#define I2C_MASTER_TX_BUF_LEN 0        // I2C master tx buffer length
#define I2C_MASTER_RX_BUF_LEN 0        // I2C master rx buffer length


static const char* TAG = "main";


static void i2c_scanner() {
    int i;
    esp_err_t ret;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
    
    printf("Scanning I2C bus...\n");
    for (i = 0; i < 128; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_stop(cmd);

        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) {
            printf("Found device at address 0x%X\n", i);
        }
        
        i2c_cmd_link_delete(cmd);
    }

    i2c_driver_delete(I2C_MASTER_NUM);
}

static void servoInit(){
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 1000,
        .max_width_us = 2000,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO1_PIN,
                SERVO2_PIN,
            },
            .ch = {
                LEDC_CHANNEL_0,
                LEDC_CHANNEL_1,
            },
        },
        .channel_number = 2,
    };

    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
}


void app_main(void){
    ESP_LOGI(TAG, "Starting Aschenbecher");

    // i2c_scanner();
    imuInit();

    ledInit();
    servoInit();

    while(1){
        ESP_LOGI(TAG, "ON");
        ledStatusSet(1);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, 0.0);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, 0.0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "OFF");
        ledStatusSet(0);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, 90.0);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, 90.0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
