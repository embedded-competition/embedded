#include "sgp40_sensor.h"

#include <math.h>
#include <stddef.h>

#include "board_pins.h"
#include "sensor_tracker.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static uint8_t sgp40_crc(const uint8_t *data, size_t length)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

void sgp40_sensor_init(void)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, config.mode, 0, 0, 0));
}

esp_err_t sgp40_sensor_read_raw(uint16_t *raw_signal)
{
    /* Default compensation: 25 degC and 50%RH. Replace with real values later. */
    uint8_t command[] = {0x26, 0x0F, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93};
    uint8_t response[3] = {0};

    esp_err_t err = i2c_master_write_to_device(I2C_PORT, SGP40_I2C_ADDRESS,
                                                command, sizeof(command), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(30));
    err = i2c_master_read_from_device(I2C_PORT, SGP40_I2C_ADDRESS,
                                      response, sizeof(response), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }
    if (sgp40_crc(response, 2) != response[2]) {
        return ESP_ERR_INVALID_CRC;
    }

    *raw_signal = ((uint16_t)response[0] << 8) | response[1];
    return ESP_OK;
}

int sgp40_sensor_normalize(float raw_value)
{
    return (int)fminf(NORMALIZED_MAX, fmaxf(0.0f,
                     (1.0f - raw_value / 65535.0f) * NORMALIZED_MAX));
}
