/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "led.h"


static const char* TAG = "main";


void app_main(void){
    ESP_LOGI(TAG, "Starting Aschenbecher");

    ledInit();

    while(1){
        ESP_LOGI(TAG, "ON");
        ledStatusSet(1);
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "OFF");
        ledStatusSet(0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
