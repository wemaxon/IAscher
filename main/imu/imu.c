#include "imu.h"

#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi160.h"
#include "i2c_interface.h"

#include "../iot_servo.h"


#define BMI160_SHUTTLE_ID           0x38
#define BMI160_DEV_ADDR             0x69
#define IMU_SAMPLE_INTERVAL_MS      100

#define COMP_FILTER_GYRO_WEIGHT     0.95f



static const char* TAG = "imu";
struct bmi160_dev bmi160dev;
struct bmi160_sensor_data bmi160_accel, bmi160_gyro;


static attitude_t estimateAttitude(struct bmi160_sensor_data acc, struct bmi160_sensor_data gyro){
    // ESP_LOGI(TAG,"ax:%d\tay:%d\taz:%d", acc.x, acc.y, acc.z);
    // ESP_LOGI(TAG,"gx:%d\tgy:%d\tgz:%d", gyro.x, gyro.y, gyro.z);
    
    static attitude_t attitude;

    /* Complementary Filter */
    float ax_mps2 = acc.x * 1.0;
    float ay_mps2 = acc.y * 1.0;
    float az_mps2 = acc.z * 1.0;

    float pitch_acc = atan(-acc.x / sqrt(pow(acc.y, 2) + pow(acc.z, 2)));
    float roll_acc = atan(acc.y / sqrt(pow(acc.x, 2) + pow(acc.z, 2)));

    attitude.pitch = (COMP_FILTER_GYRO_WEIGHT) * (attitude.pitch + (gyro.y * (1.00 / (IMU_SAMPLE_INTERVAL_MS)))) + (1.00 - COMP_FILTER_GYRO_WEIGHT) * (pitch_acc);
    attitude.roll = (COMP_FILTER_GYRO_WEIGHT) * (attitude.roll + (gyro.x * (1.00 / (IMU_SAMPLE_INTERVAL_MS)))) + (1.00 - COMP_FILTER_GYRO_WEIGHT) * (roll_acc);

    return attitude;
}

static void imuTask(void* pvParameters){
    while(1){
        bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &bmi160_accel, &bmi160_gyro, &bmi160dev);
        attitude_t attitude = estimateAttitude(bmi160_accel, bmi160_gyro);
        ESP_LOGI(TAG,"pitch: %f, roll: %f", attitude.pitch, attitude.roll);
        if(fabs(attitude.pitch) > 0.25 || fabs(attitude.roll) > 0.25){
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, 79.0);
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, 81.0);
        }
        else{
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, 15.0);
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, 15.0);
        }


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