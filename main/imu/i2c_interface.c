#include "i2c_interface.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_PORT                   I2C_NUM_0
#define I2C_MAX_CLOCK_HZ           400000
#define I2C_TRANSACTION_TIMEOUT_MS 10

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t device_handle;
static uint8_t configured_address;
static i2c_interface_statistics_t statistics;

static int8_t record_result(esp_err_t result)
{
    if (result == ESP_OK) {
        statistics.completed_transactions++;
        return 0;
    }

    statistics.failed_transactions++;
    return -1;
}

esp_err_t i2c_interface_init(const i2c_interface_config_t *config)
{
    if (config == NULL || config->clock_hz == 0 ||
        config->clock_hz > I2C_MAX_CLOCK_HZ ||
        config->device_address > 0x7F ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->sda_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->scl_gpio) ||
        config->sda_gpio == config->scl_gpio) {
        return ESP_ERR_INVALID_ARG;
    }
    if (bus_handle != NULL || device_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = config->sda_gpio,
        .scl_io_num = config->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = config->enable_internal_pullups,
    };
    esp_err_t result = i2c_new_master_bus(&bus_config, &bus_handle);
    if (result != ESP_OK) {
        return result;
    }

    result = i2c_master_probe(bus_handle,
                              config->device_address,
                              I2C_TRANSACTION_TIMEOUT_MS);
    if (result != ESP_OK) {
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
        return result;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->device_address,
        .scl_speed_hz = config->clock_hz,
    };
    result = i2c_master_bus_add_device(bus_handle,
                                       &device_config,
                                       &device_handle);
    if (result != ESP_OK) {
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
        return result;
    }

    configured_address = config->device_address;
    memset(&statistics, 0, sizeof(statistics));
    return ESP_OK;
}

esp_err_t i2c_interface_deinit(void)
{
    esp_err_t result = ESP_OK;

    if (device_handle != NULL) {
        result = i2c_master_bus_rm_device(device_handle);
        device_handle = NULL;
    }

    if (bus_handle != NULL) {
        const esp_err_t bus_result = i2c_del_master_bus(bus_handle);
        if (result == ESP_OK) {
            result = bus_result;
        }
        bus_handle = NULL;
    }

    configured_address = 0;
    memset(&statistics, 0, sizeof(statistics));
    return result;
}

bool i2c_interface_is_initialized(void)
{
    return device_handle != NULL;
}

i2c_interface_statistics_t i2c_interface_get_statistics(void)
{
    return statistics;
}

int8_t i2c_interface_write(uint8_t dev_addr,
                           uint8_t reg_addr,
                           uint8_t *write_data,
                           uint16_t len)
{
    if (device_handle == NULL || dev_addr != configured_address ||
        (len > 0 && write_data == NULL)) {
        return -1;
    }

    i2c_master_transmit_multi_buffer_info_t buffers[] = {
        {
            .write_buffer = &reg_addr,
            .buffer_size = 1,
        },
        {
            .write_buffer = write_data,
            .buffer_size = len,
        },
    };
    const size_t buffer_count = len > 0 ? 2 : 1;
    return record_result(i2c_master_multi_buffer_transmit(
        device_handle,
        buffers,
        buffer_count,
        I2C_TRANSACTION_TIMEOUT_MS));
}

int8_t i2c_interface_read(uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t *read_data,
                          uint16_t len)
{
    if (device_handle == NULL || dev_addr != configured_address ||
        read_data == NULL || len == 0) {
        return -1;
    }

    return record_result(i2c_master_transmit_receive(
        device_handle,
        &reg_addr,
        1,
        read_data,
        len,
        I2C_TRANSACTION_TIMEOUT_MS));
}

void i2c_interface_delay(uint32_t period_ms)
{
    if (period_ms == 0) {
        return;
    }

    TickType_t delay_ticks = pdMS_TO_TICKS(period_ms);
    if (delay_ticks < portMAX_DELAY) {
        delay_ticks++;
    }
    vTaskDelay(delay_ticks);
}
