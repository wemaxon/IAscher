#include "imu.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "attitude_filter.h"
#include "bmi160.h"
#include "i2c_interface.h"
#include "led.h"
#include "../iot_servo.h"

#define BMI160_DEV_ADDR              0x69
#define IMU_SAMPLE_RATE_HZ           400U
#define IMU_EXPECTED_PERIOD_US       (1000000U / IMU_SAMPLE_RATE_HZ)
#define IMU_NOMINAL_INTERVAL_S       (1.0f / IMU_SAMPLE_RATE_HZ)
#define IMU_MAX_VALID_INTERVAL_S     0.05f
#define IMU_TELEMETRY_PERIOD_US      1000000LL
#define IMU_SAMPLER_TASK_PRIORITY    8
#define IMU_TELEMETRY_TASK_PRIORITY  2
#define IMU_SAMPLER_STACK_BYTES      4096
#define IMU_TELEMETRY_STACK_BYTES    3072

#define ACCEL_LSB_PER_G              8192.0f
#define GYRO_LSB_PER_DPS             65.6f
#define DEGREES_TO_RADIANS           0.01745329252f
#define RADIANS_TO_DEGREES           57.29577951f

#define FILTER_CORRECTION_TIME_S     0.15f

#define VISOR_OPEN_THRESHOLD_RAD     0.25f
#define VISOR_CLOSE_THRESHOLD_RAD    0.18f
#define VISOR_LEFT_CLOSED_ANGLE_DEG  50.0f
#define VISOR_RIGHT_CLOSED_ANGLE_DEG 55.0f
#define VISOR_LEFT_OPEN_ANGLE_DEG    102.5f
#define VISOR_RIGHT_OPEN_ANGLE_DEG   112.5f

typedef struct {
    attitude_t attitude;
    i2c_interface_statistics_t i2c;
    float measured_rate_hz;
    uint32_t minimum_period_us;
    uint32_t maximum_period_us;
    uint32_t maximum_read_latency_us;
    uint32_t maximum_pipeline_latency_us;
    uint32_t missed_sample_count;
    uint32_t read_error_count;
    uint32_t actuator_error_count;
    bool visor_open;
} imu_telemetry_t;

static const char *TAG = "imu";
static struct bmi160_dev bmi160dev;
static TaskHandle_t sampler_task_handle;
static TaskHandle_t telemetry_task_handle;
static QueueHandle_t telemetry_queue;

static uint32_t saturatingMicroseconds(int64_t elapsed_us)
{
    if (elapsed_us <= 0) {
        return 0;
    }
    if ((uint64_t)elapsed_us > UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t)elapsed_us;
}

static int64_t expandTimestamp(uint32_t timestamp_low, int64_t now_us)
{
    const uint32_t now_low = (uint32_t)now_us;
    const uint32_t elapsed_us = now_low - timestamp_low;
    return now_us - elapsed_us;
}

static esp_err_t setVisorOpen(bool open)
{
    ledStatusSet(open ? 1 : 0);

    const float left_angle = open
                                 ? VISOR_LEFT_OPEN_ANGLE_DEG
                                 : VISOR_LEFT_CLOSED_ANGLE_DEG;
    const float right_angle = open
                                  ? VISOR_RIGHT_OPEN_ANGLE_DEG
                                  : VISOR_RIGHT_CLOSED_ANGLE_DEG;
    const esp_err_t left_result = iot_servo_write_angle(
        LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, left_angle);
    const esp_err_t right_result = iot_servo_write_angle(
        LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, right_angle);

    return left_result == ESP_OK && right_result == ESP_OK
               ? ESP_OK
               : ESP_FAIL;
}

static float validSampleInterval(int64_t trigger_us,
                                 int64_t previous_trigger_us,
                                 uint32_t *missed_sample_count)
{
    const int64_t elapsed_us = trigger_us - previous_trigger_us;
    if (elapsed_us <= 0) {
        return IMU_NOMINAL_INTERVAL_S;
    }

    if (elapsed_us >
        IMU_EXPECTED_PERIOD_US + (IMU_EXPECTED_PERIOD_US / 2U)) {
        const uint32_t elapsed_periods = (uint32_t)(
            (elapsed_us + (IMU_EXPECTED_PERIOD_US / 2U)) /
            IMU_EXPECTED_PERIOD_US);
        *missed_sample_count += elapsed_periods - 1U;
    }

    const float measured_interval_s = elapsed_us / 1000000.0f;
    if (measured_interval_s > IMU_MAX_VALID_INTERVAL_S) {
        return IMU_NOMINAL_INTERVAL_S;
    }
    return measured_interval_s;
}

static void dataReadyIsr(void *argument)
{
    TaskHandle_t task = argument;
    const uint32_t timestamp_us = (uint32_t)esp_timer_get_time();
    BaseType_t higher_priority_task_woken = pdFALSE;

    xTaskNotifyFromISR(task,
                       timestamp_us,
                       eSetValueWithOverwrite,
                       &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void telemetryTask(void *argument)
{
    (void)argument;
    imu_telemetry_t telemetry;

    while (true) {
        if (xQueueReceive(telemetry_queue,
                          &telemetry,
                          pdMS_TO_TICKS(1500)) != pdTRUE) {
            ESP_LOGW(TAG,
                     "No BMI160 data-ready events; check INT1 -> GPIO%d",
                     CONFIG_ASHLESS_BMI160_INT1_GPIO);
            continue;
        }

        ESP_LOGI(TAG,
                 "drdy=%.1fHz period=%lu..%luus max_read=%luus "
                 "max_pipeline=%luus missed=%lu read_errors=%lu "
                 "i2c_errors=%lu actuator_errors=%lu "
                 "pitch=%.1f roll=%.1f visor=%s",
                 telemetry.measured_rate_hz,
                 (unsigned long)telemetry.minimum_period_us,
                 (unsigned long)telemetry.maximum_period_us,
                 (unsigned long)telemetry.maximum_read_latency_us,
                 (unsigned long)telemetry.maximum_pipeline_latency_us,
                 (unsigned long)telemetry.missed_sample_count,
                 (unsigned long)telemetry.read_error_count,
                 (unsigned long)telemetry.i2c.failed_transactions,
                 (unsigned long)telemetry.actuator_error_count,
                 telemetry.attitude.pitch * RADIANS_TO_DEGREES,
                 telemetry.attitude.roll * RADIANS_TO_DEGREES,
                 telemetry.visor_open ? "open" : "closed");
    }
}

static void imuTask(void *argument)
{
    (void)argument;

    attitude_filter_t filter;
    attitudeFilterReset(&filter);

    attitude_t attitude = {0};
    bool visor_open = false;
    bool have_previous_trigger = false;
    int64_t previous_trigger_us = 0;
    int64_t telemetry_window_start_us = 0;
    uint32_t telemetry_window_sample_count = 0;
    uint32_t minimum_period_us = UINT32_MAX;
    uint32_t maximum_period_us = 0;
    uint32_t missed_sample_count = 0;
    uint32_t read_error_count = 0;
    uint32_t actuator_error_count = 0;
    uint32_t maximum_read_latency_us = 0;
    uint32_t maximum_pipeline_latency_us = 0;

    while (true) {
        uint32_t trigger_timestamp_low;
        if (xTaskNotifyWait(0,
                            UINT32_MAX,
                            &trigger_timestamp_low,
                            portMAX_DELAY) != pdTRUE) {
            continue;
        }

        const int64_t read_start_us = esp_timer_get_time();
        const int64_t trigger_us = expandTimestamp(trigger_timestamp_low,
                                                   read_start_us);
        float dt_s = IMU_NOMINAL_INTERVAL_S;
        if (have_previous_trigger) {
            const int64_t period_us = trigger_us - previous_trigger_us;
            dt_s = validSampleInterval(trigger_us,
                                       previous_trigger_us,
                                       &missed_sample_count);
            if (period_us > 0 && period_us <= UINT32_MAX) {
                const uint32_t period = (uint32_t)period_us;
                if (period < minimum_period_us) {
                    minimum_period_us = period;
                }
                if (period > maximum_period_us) {
                    maximum_period_us = period;
                }
            }
        } else {
            have_previous_trigger = true;
            telemetry_window_start_us = trigger_us;
        }
        previous_trigger_us = trigger_us;
        telemetry_window_sample_count++;

        struct bmi160_sensor_data accel;
        struct bmi160_sensor_data gyro;
        const int8_t result = bmi160_get_sensor_data(
            BMI160_ACCEL_SEL | BMI160_GYRO_SEL,
            &accel,
            &gyro,
            &bmi160dev);

        if (result == BMI160_OK) {
            const uint32_t read_latency_us = saturatingMicroseconds(
                esp_timer_get_time() - trigger_us);
            if (read_latency_us > maximum_read_latency_us) {
                maximum_read_latency_us = read_latency_us;
            }

            const float accel_x_g = accel.x / ACCEL_LSB_PER_G;
            const float accel_y_g = accel.y / ACCEL_LSB_PER_G;
            const float accel_z_g = accel.z / ACCEL_LSB_PER_G;
            const float gyro_x_rad_s =
                (gyro.x / GYRO_LSB_PER_DPS) * DEGREES_TO_RADIANS;
            const float gyro_y_rad_s =
                (gyro.y / GYRO_LSB_PER_DPS) * DEGREES_TO_RADIANS;
            const float gyro_z_rad_s =
                (gyro.z / GYRO_LSB_PER_DPS) * DEGREES_TO_RADIANS;

            attitude = attitudeFilterUpdate(
                &filter,
                accel_x_g,
                accel_y_g,
                accel_z_g,
                gyro_x_rad_s,
                gyro_y_rad_s,
                gyro_z_rad_s,
                dt_s,
                FILTER_CORRECTION_TIME_S);

            const float maximum_tilt = fmaxf(fabsf(attitude.pitch),
                                             fabsf(attitude.roll));
            const bool should_open = visor_open
                                         ? maximum_tilt >
                                               VISOR_CLOSE_THRESHOLD_RAD
                                         : maximum_tilt >
                                               VISOR_OPEN_THRESHOLD_RAD;

            if (should_open != visor_open) {
                if (setVisorOpen(should_open) == ESP_OK) {
                    visor_open = should_open;
                } else {
                    actuator_error_count++;
                }
            }
        } else {
            read_error_count++;
        }

        const int64_t pipeline_complete_us = esp_timer_get_time();
        const uint32_t pipeline_latency_us = saturatingMicroseconds(
            pipeline_complete_us - trigger_us);
        if (pipeline_latency_us > maximum_pipeline_latency_us) {
            maximum_pipeline_latency_us = pipeline_latency_us;
        }
        const int64_t telemetry_elapsed_us =
            trigger_us - telemetry_window_start_us;
        if (telemetry_elapsed_us >= IMU_TELEMETRY_PERIOD_US) {
            const uint32_t completed_intervals =
                telemetry_window_sample_count > 0
                    ? telemetry_window_sample_count - 1U
                    : 0;
            const imu_telemetry_t telemetry = {
                .attitude = attitude,
                .i2c = i2c_interface_get_statistics(),
                .measured_rate_hz =
                    (completed_intervals * 1000000.0f) /
                    telemetry_elapsed_us,
                .minimum_period_us =
                    minimum_period_us == UINT32_MAX
                        ? 0
                        : minimum_period_us,
                .maximum_period_us = maximum_period_us,
                .maximum_read_latency_us = maximum_read_latency_us,
                .maximum_pipeline_latency_us =
                    maximum_pipeline_latency_us,
                .missed_sample_count = missed_sample_count,
                .read_error_count = read_error_count,
                .actuator_error_count = actuator_error_count,
                .visor_open = visor_open,
            };
            xQueueOverwrite(telemetry_queue, &telemetry);

            telemetry_window_start_us = trigger_us;
            telemetry_window_sample_count = 1;
            minimum_period_us = UINT32_MAX;
            maximum_period_us = 0;
            maximum_read_latency_us = 0;
            maximum_pipeline_latency_us = 0;
        }
    }
}

static esp_err_t verifySensorConfiguration(void)
{
    uint8_t registers[4];
    if (bmi160_get_regs(BMI160_ACCEL_CONFIG_ADDR,
                        registers,
                        sizeof(registers),
                        &bmi160dev) != BMI160_OK) {
        return ESP_FAIL;
    }

    const uint8_t expected_accel_config =
        (BMI160_ACCEL_BW_NORMAL_AVG4 << BMI160_ACCEL_BW_POS) |
        BMI160_ACCEL_ODR_400HZ;
    const uint8_t expected_gyro_config =
        (BMI160_GYRO_BW_NORMAL_MODE << BMI160_GYRO_BW_POS) |
        BMI160_GYRO_ODR_400HZ;
    if (registers[0] != expected_accel_config ||
        registers[1] != BMI160_ACCEL_RANGE_4G ||
        registers[2] != expected_gyro_config ||
        registers[3] != BMI160_GYRO_RANGE_500_DPS) {
        ESP_LOGE(TAG,
                 "BMI160 config readback mismatch: %02X %02X %02X %02X",
                 registers[0],
                 registers[1],
                 registers[2],
                 registers[3]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

static esp_err_t configureSensor(void)
{
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_400HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_400HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_500_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    if (bmi160_set_sens_conf(&bmi160dev) != BMI160_OK) {
        return ESP_FAIL;
    }

    const esp_err_t verify_result = verifySensorConfiguration();
    if (verify_result != ESP_OK) {
        return verify_result;
    }

    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;
    return bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL,
                                  &accel,
                                  &gyro,
                                  &bmi160dev) == BMI160_OK
               ? ESP_OK
               : ESP_FAIL;
}

static esp_err_t verifyDataReadyConfiguration(void)
{
    uint8_t interrupt_enable;
    uint8_t output_control;
    uint8_t interrupt_latch;
    uint8_t interrupt_map;
    if (bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR,
                        &interrupt_enable,
                        1,
                        &bmi160dev) != BMI160_OK ||
        bmi160_get_regs(BMI160_INT_OUT_CTRL_ADDR,
                        &output_control,
                        1,
                        &bmi160dev) != BMI160_OK ||
        bmi160_get_regs(BMI160_INT_LATCH_ADDR,
                        &interrupt_latch,
                        1,
                        &bmi160dev) != BMI160_OK ||
        bmi160_get_regs(BMI160_INT_MAP_1_ADDR,
                        &interrupt_map,
                        1,
                        &bmi160dev) != BMI160_OK) {
        return ESP_FAIL;
    }

    const uint8_t required_output_bits =
        BMI160_INT1_OUTPUT_EN_MASK | BMI160_INT1_OUTPUT_TYPE_MASK;
    if ((interrupt_enable & BMI160_DATA_RDY_INT_EN_MASK) == 0 ||
        (output_control & required_output_bits) != required_output_bits ||
        (output_control & BMI160_INT1_OUTPUT_MODE_MASK) != 0 ||
        (output_control & BMI160_INT1_EDGE_CTRL_MASK) != 0 ||
        (interrupt_latch &
         (BMI160_INT1_INPUT_EN_MASK | BMI160_INT_LATCH_MASK)) != 0 ||
        (interrupt_map & BMI160_INT1_DATA_READY_MASK) == 0) {
        ESP_LOGE(TAG,
                 "BMI160 DRDY readback mismatch: "
                 "enable=%02X out=%02X latch=%02X map=%02X",
                 interrupt_enable,
                 output_control,
                 interrupt_latch,
                 interrupt_map);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

static esp_err_t configureDataReadyInterrupt(void)
{
    const gpio_num_t gpio = (gpio_num_t)CONFIG_ASHLESS_BMI160_INT1_GPIO;
    const gpio_config_t gpio_config_data = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    esp_err_t result = gpio_config(&gpio_config_data);
    if (result != ESP_OK) {
        return result;
    }

    result = gpio_install_isr_service(0);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        return result;
    }
    result = gpio_isr_handler_add(gpio,
                                  dataReadyIsr,
                                  sampler_task_handle);
    if (result != ESP_OK) {
        return result;
    }

    struct bmi160_int_settg interrupt_config;
    memset(&interrupt_config, 0, sizeof(interrupt_config));
    interrupt_config.int_channel = BMI160_INT_CHANNEL_1;
    interrupt_config.int_type = BMI160_ACC_GYRO_DATA_RDY_INT;
    interrupt_config.int_pin_settg.output_en = BMI160_ENABLE;
    interrupt_config.int_pin_settg.output_mode = BMI160_DISABLE;
    interrupt_config.int_pin_settg.output_type = BMI160_ENABLE;
    interrupt_config.int_pin_settg.edge_ctrl = BMI160_DISABLE;
    interrupt_config.int_pin_settg.input_en = BMI160_DISABLE;
    interrupt_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE;

    if (bmi160_set_int_config(&interrupt_config, &bmi160dev) != BMI160_OK) {
        gpio_isr_handler_remove(gpio);
        return ESP_FAIL;
    }

    result = verifyDataReadyConfiguration();
    if (result != ESP_OK) {
        gpio_isr_handler_remove(gpio);
    }
    return result;
}

static void cleanupAfterStartFailure(void)
{
    if (sampler_task_handle != NULL) {
        vTaskDelete(sampler_task_handle);
        sampler_task_handle = NULL;
    }
    if (telemetry_task_handle != NULL) {
        vTaskDelete(telemetry_task_handle);
        telemetry_task_handle = NULL;
    }
    if (telemetry_queue != NULL) {
        vQueueDelete(telemetry_queue);
        telemetry_queue = NULL;
    }
    i2c_interface_deinit();
    memset(&bmi160dev, 0, sizeof(bmi160dev));
}

esp_err_t imuInit(void)
{
    if (sampler_task_handle != NULL || i2c_interface_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    const gpio_num_t sda_gpio =
        (gpio_num_t)CONFIG_ASHLESS_I2C_SDA_GPIO;
    const gpio_num_t scl_gpio =
        (gpio_num_t)CONFIG_ASHLESS_I2C_SCL_GPIO;
    const gpio_num_t data_ready_gpio =
        (gpio_num_t)CONFIG_ASHLESS_BMI160_INT1_GPIO;
    if (data_ready_gpio == sda_gpio || data_ready_gpio == scl_gpio) {
        return ESP_ERR_INVALID_ARG;
    }

    const i2c_interface_config_t i2c_config = {
        .sda_gpio = sda_gpio,
        .scl_gpio = scl_gpio,
        .clock_hz = CONFIG_ASHLESS_I2C_CLOCK_HZ,
        .device_address = BMI160_DEV_ADDR,
        .enable_internal_pullups = true,
    };
    esp_err_t result = i2c_interface_init(&i2c_config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization/probe failed: %s",
                 esp_err_to_name(result));
        return result;
    }

    memset(&bmi160dev, 0, sizeof(bmi160dev));
    bmi160dev.write = i2c_interface_write;
    bmi160dev.read = i2c_interface_read;
    bmi160dev.delay_ms = i2c_interface_delay;
    bmi160dev.id = BMI160_DEV_ADDR;
    bmi160dev.intf = BMI160_I2C_INTF;

    if (bmi160_init(&bmi160dev) != BMI160_OK) {
        cleanupAfterStartFailure();
        return ESP_FAIL;
    }

    result = configureSensor();
    if (result != ESP_OK) {
        cleanupAfterStartFailure();
        return result;
    }

    telemetry_queue = xQueueCreate(1, sizeof(imu_telemetry_t));
    if (telemetry_queue == NULL) {
        cleanupAfterStartFailure();
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(telemetryTask,
                    "imu_telemetry",
                    IMU_TELEMETRY_STACK_BYTES,
                    NULL,
                    IMU_TELEMETRY_TASK_PRIORITY,
                    &telemetry_task_handle) != pdPASS ||
        xTaskCreate(imuTask,
                    "imu_sampler",
                    IMU_SAMPLER_STACK_BYTES,
                    NULL,
                    IMU_SAMPLER_TASK_PRIORITY,
                    &sampler_task_handle) != pdPASS) {
        cleanupAfterStartFailure();
        return ESP_ERR_NO_MEM;
    }

    result = configureDataReadyInterrupt();
    if (result != ESP_OK) {
        cleanupAfterStartFailure();
        return result;
    }

    ESP_LOGI(TAG,
             "BMI160 0x%02X: accel+gyro 400 Hz, I2C %lu Hz, INT1 -> GPIO%d",
             bmi160dev.chip_id,
             (unsigned long)CONFIG_ASHLESS_I2C_CLOCK_HZ,
             CONFIG_ASHLESS_BMI160_INT1_GPIO);
    return ESP_OK;
}
