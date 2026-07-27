#include "imu.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi160.h"
#include "i2c_interface.h"

#include "../iot_servo.h"


#define BMI160_DEV_ADDR                    0x69

#define IMU_SAMPLE_INTERVAL_MS             10
#define IMU_NOMINAL_INTERVAL_S             (IMU_SAMPLE_INTERVAL_MS / 1000.0f)
#define IMU_MAX_VALID_INTERVAL_S           0.05f
#define IMU_LOG_INTERVAL_MS                250

/* These scale factors must match the sensor ranges configured in imuInit(). */
#define ACCEL_LSB_PER_G                    8192.0f
#define GYRO_LSB_PER_DPS                   65.6f
#define DEGREES_TO_RADIANS                 0.01745329252f
#define RADIANS_TO_DEGREES                 57.29577951f

/* Lower values react faster; higher values reject more vibration. */
#define FILTER_CORRECTION_TIME_S           0.15f

#define VISOR_OPEN_THRESHOLD_RAD           0.25f
#define VISOR_CLOSE_THRESHOLD_RAD          0.18f
#define VISOR_CLOSED_ANGLE_DEG             0.0f
#define VISOR_LEFT_OPEN_ANGLE_DEG          79.0f
#define VISOR_RIGHT_OPEN_ANGLE_DEG         81.0f


static const char *TAG = "imu";
static struct bmi160_dev bmi160dev;


static esp_err_t setVisorOpen(bool open)
{
    const float left_angle = open ? VISOR_LEFT_OPEN_ANGLE_DEG : VISOR_CLOSED_ANGLE_DEG;
    const float right_angle = open ? VISOR_RIGHT_OPEN_ANGLE_DEG : VISOR_CLOSED_ANGLE_DEG;
    const esp_err_t left_result = iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, left_angle);
    const esp_err_t right_result = iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, right_angle);

    return (left_result == ESP_OK && right_result == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static float validSampleInterval(int64_t now_us, int64_t previous_us)
{
    const float measured_interval_s = (now_us - previous_us) / 1000000.0f;

    if (measured_interval_s <= 0.0f || measured_interval_s > IMU_MAX_VALID_INTERVAL_S) {
        return IMU_NOMINAL_INTERVAL_S;
    }

    return measured_interval_s;
}

static void imuTask(void *pvParameters)
{
    (void)pvParameters;

    attitude_filter_t filter;
    attitudeFilterReset(&filter);

    bool visor_open = false;
    uint32_t log_sample_count = 0;
    uint32_t read_error_count = 0;
    int64_t previous_update_us = esp_timer_get_time();
    TickType_t previous_wake_tick = xTaskGetTickCount();

    while (true) {
        struct bmi160_sensor_data accel;
        struct bmi160_sensor_data gyro;
        const int8_t result = bmi160_get_sensor_data(
            BMI160_ACCEL_SEL | BMI160_GYRO_SEL, &accel, &gyro, &bmi160dev);

        if (result == BMI160_OK) {
            const int64_t now_us = esp_timer_get_time();
            const float dt_s = validSampleInterval(now_us, previous_update_us);
            previous_update_us = now_us;
            read_error_count = 0;

            const float accel_x_g = accel.x / ACCEL_LSB_PER_G;
            const float accel_y_g = accel.y / ACCEL_LSB_PER_G;
            const float accel_z_g = accel.z / ACCEL_LSB_PER_G;
            const float gyro_x_rad_s = (gyro.x / GYRO_LSB_PER_DPS) * DEGREES_TO_RADIANS;
            const float gyro_y_rad_s = (gyro.y / GYRO_LSB_PER_DPS) * DEGREES_TO_RADIANS;
            const float gyro_z_rad_s = (gyro.z / GYRO_LSB_PER_DPS) * DEGREES_TO_RADIANS;

            const attitude_t attitude = attitudeFilterUpdate(
                &filter,
                accel_x_g,
                accel_y_g,
                accel_z_g,
                gyro_x_rad_s,
                gyro_y_rad_s,
                gyro_z_rad_s,
                dt_s,
                FILTER_CORRECTION_TIME_S);

            const float maximum_tilt = fmaxf(fabsf(attitude.pitch), fabsf(attitude.roll));
            const bool should_open = visor_open
                                         ? maximum_tilt > VISOR_CLOSE_THRESHOLD_RAD
                                         : maximum_tilt > VISOR_OPEN_THRESHOLD_RAD;

            if (should_open != visor_open) {
                if (setVisorOpen(should_open) == ESP_OK) {
                    visor_open = should_open;
                } else {
                    ESP_LOGE(TAG, "Failed to update visor servos");
                }
            }

            log_sample_count++;
            if (log_sample_count >= (IMU_LOG_INTERVAL_MS / IMU_SAMPLE_INTERVAL_MS)) {
                ESP_LOGI(TAG,
                         "pitch: %.1f deg, roll: %.1f deg, visor: %s",
                         attitude.pitch * RADIANS_TO_DEGREES,
                         attitude.roll * RADIANS_TO_DEGREES,
                         visor_open ? "open" : "closed");
                log_sample_count = 0;
            }
        } else {
            read_error_count++;
            if (read_error_count == 1 || (read_error_count % 100) == 0) {
                ESP_LOGW(TAG, "BMI160 read failed (%d), count: %lu",
                         result, (unsigned long)read_error_count);
            }
        }

        vTaskDelayUntil(&previous_wake_tick, pdMS_TO_TICKS(IMU_SAMPLE_INTERVAL_MS));
    }
}

esp_err_t imuInit(void)
{
    bmi160dev.write = i2c_interface_write;
    bmi160dev.read = i2c_interface_read;
    bmi160dev.delay_ms = i2c_interface_delay;
    bmi160dev.id = BMI160_DEV_ADDR;
    bmi160dev.intf = BMI160_I2C_INTF;

    const esp_err_t i2c_result = i2c_interface_init();
    if (i2c_result != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(i2c_result));
        return i2c_result;
    }

    int8_t result = bmi160_init(&bmi160dev);
    if (result != BMI160_OK) {
        ESP_LOGE(TAG, "BMI160 initialization failed (%d)", result);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BMI160 initialized, chip ID 0x%X", bmi160dev.chip_id);

    /* 200 Hz provides fresh, low-latency data for the 100 Hz estimator. */
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_200HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_200HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_500_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    result = bmi160_set_sens_conf(&bmi160dev);
    if (result != BMI160_OK) {
        ESP_LOGE(TAG, "BMI160 configuration failed (%d)", result);
        return ESP_FAIL;
    }

    if (xTaskCreate(imuTask, "estimator", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create estimator task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
