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
        ESP_LOGI(TAG, "Chip ID 0x%X", bmi160dev.chip_id);
    }
    else{
        ESP_LOGI(TAG, "BMI160 initialization failure !");
        return ESP_FAIL;
    }

    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi160dev);
    if (rslt == BMI160_OK){
        ESP_LOGI(TAG, "BMI160 configuration success !");
    }
    else{
        ESP_LOGI(TAG, "BMI160 configuration failure !");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t imuDumpData(void){
    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &bmi160_accel, &bmi160_gyro, &bmi160dev);
    ESP_LOGI(TAG,"ax:%d\tay:%d\taz:%d", bmi160_accel.x, bmi160_accel.y, bmi160_accel.z);
    ESP_LOGI(TAG,"gx:%d\tgy:%d\tgz:%d", bmi160_gyro.x, bmi160_gyro.y, bmi160_gyro.z);
    return ESP_OK;
}