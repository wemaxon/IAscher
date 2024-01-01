#include "imu.h"

#include "esp_log.h"

#include "bmi160.h"
#include "i2c_interface.h"


#define BMI160_SHUTTLE_ID     0x38
#define BMI160_DEV_ADDR       0x69

static const char* TAG = "imu";
struct bmi160_dev bmi160dev;
struct bmi160_sensor_data bmi160_accel;
struct bmi160_sensor_data bmi160_gyro;


esp_err_t imuInit(void){

    /* setup i2c interface functions */
    bmi160dev.write = i2c_interface_write;
    bmi160dev.read = i2c_interface_read;
    bmi160dev.delay_ms = i2c_interface_delay;
    bmi160dev.id = BMI160_DEV_ADDR;
    bmi160dev.intf = BMI160_I2C_INTF;

    i2c_interface_init();

    int8_t rslt = bmi160_init(&bmi160dev);

    if (rslt == BMI160_OK){
        ESP_LOGI(TAG, "BMI160 initialization success !");
        ESP_LOGI(TAG, "Chip ID 0x%X\n", bmi160dev.chip_id);
    }
    else{
        ESP_LOGI(TAG, "BMI160 initialization failure !");
        return ESP_FAIL;
    }

    return ESP_OK;
}