#ifndef _IMU_I2C_INTERFACE_H_
#define _IMU_I2C_INTERFACE_H_

#include "esp_err.h"

esp_err_t i2c_interface_init(void);
int8_t i2c_interface_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
int8_t i2c_interface_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
void i2c_interface_delay(uint32_t period);



#endif /* _IMU_I2C_INTERFACE_H_ */