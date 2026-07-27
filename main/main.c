/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "led.h"
#include "iot_servo.h"
#include "imu/imu.h"


#define SERVO1_PIN           21
#define SERVO2_PIN           20


static const char* TAG = "main";


static void servoInit(){
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
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

    servoInit();
    ledInit();
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, 15.0);
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, 15.0);
    if (imuInit() != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed");
    }

    vTaskDelete(NULL);
}
