#ifndef ASHLESS_I2C_INTERFACE_H
#define ASHLESS_I2C_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    uint32_t clock_hz;
    uint8_t device_address;
    bool enable_internal_pullups;
} i2c_interface_config_t;

typedef struct {
    uint32_t completed_transactions;
    uint32_t failed_transactions;
} i2c_interface_statistics_t;

esp_err_t i2c_interface_init(const i2c_interface_config_t *config);
esp_err_t i2c_interface_deinit(void);
bool i2c_interface_is_initialized(void);
i2c_interface_statistics_t i2c_interface_get_statistics(void);

int8_t i2c_interface_write(uint8_t dev_addr,
                           uint8_t reg_addr,
                           uint8_t *write_data,
                           uint16_t len);
int8_t i2c_interface_read(uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t *read_data,
                          uint16_t len);
void i2c_interface_delay(uint32_t period_ms);

#endif /* ASHLESS_I2C_INTERFACE_H */
