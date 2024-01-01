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


#define SERVO1_PIN           20
#define SERVO2_PIN           21


static const char* TAG = "main";


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
