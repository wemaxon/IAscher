#include "led.h"

#include "driver/gpio.h"

esp_err_t ledInit(void){
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_STATUS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    return gpio_config(&io_conf);
}

esp_err_t ledStatusSet(uint8_t level){
    if (level > 1) {
        return ESP_ERR_INVALID_ARG;
    }

    return gpio_set_level(LED_STATUS_PIN, level);
    }