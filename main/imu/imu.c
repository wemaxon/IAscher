#include "imu.h"

#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi160.h"
#include "i2c_interface.h"


#define BMI160_SHUTTLE_ID           0x38
#define BMI160_DEV_ADDR             0x69
#define IMU_SAMPLE_INTERVAL_MS      1000
#define G_MPS2                      9.81f
#define RAD_TO_DEG                  57.2957795131f


static const char* TAG = "imu";
struct bmi160_dev bmi160dev;
struct bmi160_sensor_data bmi160_accel, bmi160_gyro;


static attitude_t estimateAttitude(struct bmi160_sensor_data acc, struct bmi160_sensor_data gyro){
    ESP_LOGI(TAG,"ax:%d\tay:%d\taz:%d", acc.x, acc.y, acc.z);
    ESP_LOGI(TAG,"gx:%d\tgy:%d\tgz:%d", gyro.x, gyro.y, gyro.z);
    
    attitude_t attitude;

    /* get accelerometer measurements */
    float ax_mps2 = acc.x * 1.0;
    float ay_mps2 = acc.y * 1.0;
    float az_mps2 = acc.z * 1.0;
    
    /* estimate angles using accelerometer measurements */
    attitude.pitch = atanf(ay_mps2 / az_mps2);
    attitude.roll = asinf(ax_mps2 / sqrtf((ay_mps2 * ay_mps2) + (az_mps2 * az_mps2)));

    return attitude;
}

static void imuTask(void* pvParameters){
    while(1){
        bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &bmi160_accel, &bmi160_gyro, &bmi160dev);
        attitude_t attitude = estimateAttitude(bmi160_accel, bmi160_gyro);
        ESP_LOGI(TAG,"pitch: %f, roll: %f", attitude.pitch, attitude.roll);
        vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_INTERVAL_MS));
    }
}

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

    xTaskCreate(imuTask, "estimator", 4096, NULL, 5, NULL);

    return ESP_OK;
}