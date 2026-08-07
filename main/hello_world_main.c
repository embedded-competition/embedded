#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ESP32-C3 SuperMini pin assignments. Change these to match your board. */
#define MQ7_ADC_CHANNEL              ADC_CHANNEL_0 /* GPIO0 */
#define MQ8_ADC_CHANNEL              ADC_CHANNEL_1 /* GPIO1 */
#define FSR402_ADC_CHANNEL           ADC_CHANNEL_2 /* GPIO2 */
#define WATER_LEVEL_ADC_CHANNEL     ADC_CHANNEL_3 /* GPIO3 */
#define I2C_SDA_GPIO                 GPIO_NUM_8
#define I2C_SCL_GPIO                 GPIO_NUM_9
#define I2C_PORT                     I2C_NUM_0
#define SGP40_I2C_ADDRESS            0x59

/* RYLR998 UART: ESP32-C3 TX -> module RX, ESP32-C3 RX -> module TX. */
#define LORA_UART_PORT               UART_NUM_1
#define LORA_UART_TX_GPIO            GPIO_NUM_4
#define LORA_UART_RX_GPIO            GPIO_NUM_5
#define LORA_UART_BAUD_RATE          115200
#define LORA_UART_BUF_SIZE           512
#define LORA_NODE_ADDRESS            1
#define LORA_RECEIVER_ADDRESS        2
#define LORA_NETWORK_ID              18
#define LORA_BAND_HZ                 915000000
#define LORA_PARAMETER                "9,7,1,12"

#define SAMPLE_PERIOD_MS             1000
#define SAMPLES_PER_MINUTE           60
#define BASELINE_BUCKET_COUNT        3
#define TRIM_COUNT                   6 /* 10% from both ends of 60 samples */
#define ADC_RAW_MAX                  4095
#define ADC_SATURATION_MARGIN        5

#define MQ7_RAW_DANGER_RATIO         1.50f
#define MQ8_RAW_DANGER_RATIO         1.50f
#define FSR402_RAW_DANGER_RATIO      1.50f
#define WATER_LEVEL_RAW_DANGER_RATIO 1.50f
#define SGP40_RAW_DROP_DANGER_RATIO  0.80f
#define DANGER_HOLD_SAMPLES          3

/*
 * MQ voltage-divider assumptions:
 *   VCC --- Rs(sensor) --- ADC node --- RL(load resistor) --- GND
 *   Vout = VCC * RL / (Rs + RL)
 *   Rs = RL * (VCC - Vout) / Vout
 *
 * Many MQ modules use 5V heater/sensor power and a board-level divider before
 * the ESP32 ADC. Set MQ_ADC_FULL_SCALE_MV to the maximum ESP32 ADC voltage that
 * corresponds to MQ_SENSOR_VCC_MV at the sensor node.
 */
#define MQ_SENSOR_VCC_MV             5000.0f
#define MQ_ADC_FULL_SCALE_MV         3300.0f
#define MQ7_RL_KOHM                  10.0f
#define MQ8_RL_KOHM                  10.0f

/*
 * R0 calibration targets and clean-air factors.
 * R0 is estimated as Rs / CLEAN_AIR_FACTOR during warmup, then frozen when the
 * 3-minute baseline window becomes ready. Tune these with your sensor module
 * and environment.
 */
#define MQ7_CLEAN_AIR_FACTOR         27.5f
#define MQ8_CLEAN_AIR_FACTOR         70.0f
#define MQ_R0_EMA_ALPHA              0.05f

/*
 * ppm curve: ppm = A * powf(Rs / R0, B)
 * These are starting points from commonly used log-log MQ curves. For accurate
 * readings, replace them with coefficients fitted from your sensor datasheet
 * target-gas curve and calibration gas.
 */
#define MQ7_PPM_CURVE_A              99.042f
#define MQ7_PPM_CURVE_B              -1.518f
#define MQ8_PPM_CURVE_A              976.97f
#define MQ8_PPM_CURVE_B              -0.688f

#define TAG "GAS_MONITOR"

typedef struct {
    float samples[SAMPLES_PER_MINUTE];
    size_t sample_count;
    float minute_values[BASELINE_BUCKET_COUNT];
    size_t minute_count;
    size_t next_minute_index;
    float baseline;
    float ema;
    bool has_ema;
    bool ready;
    float spike_abs_threshold;
    float spike_ratio_threshold;
} baseline_tracker_t;

typedef struct {
    float raw;
    float filtered;
    bool accepted;
} tracker_result_t;

typedef struct {
    float rl_kohm;
    float clean_air_factor;
    float curve_a;
    float curve_b;
    float r0_kohm;
    bool has_r0;
    bool r0_locked;
} mq_calibration_t;

typedef struct {
    float sensor_mv;
    float rs_kohm;
    float r0_kohm;
    float ratio;
    float ppm;
    bool valid;
} mq_gas_result_t;

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali_handle;
static bool s_adc_cali_enabled;

static void lora_uart_init(void)
{
    const uart_config_t config = {
        .baud_rate = LORA_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(LORA_UART_PORT, LORA_UART_BUF_SIZE,
                                        LORA_UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LORA_UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART_PORT, LORA_UART_TX_GPIO,
                                 LORA_UART_RX_GPIO, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

static int lora_read_line(char *line, size_t line_size, TickType_t timeout)
{
    size_t length = 0;
    TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < timeout && length < line_size - 1) {
        uint8_t ch = 0;
        if (uart_read_bytes(LORA_UART_PORT, &ch, 1, pdMS_TO_TICKS(50)) <= 0) {
            continue;
        }
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            if (length == 0) {
                continue;
            }
            break;
        }
        line[length++] = (char)ch;
    }

    line[length] = '\0';
    return (int)length;
}

static bool lora_send_command(const char *command, const char *expected_reply)
{
    char response[128];
    uart_write_bytes(LORA_UART_PORT, command, strlen(command));
    uart_write_bytes(LORA_UART_PORT, "\r\n", 2);

    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(1500)) {
        int length = lora_read_line(response, sizeof(response), pdMS_TO_TICKS(200));
        if (length <= 0) {
            continue;
        }
        if (strstr(response, expected_reply) != NULL) {
            return true;
        }
        if (strstr(response, "+ERR") != NULL) {
            return false;
        }
    }

    ESP_LOGW(TAG, "LoRa command timeout: %s", command);
    return false;
}

static void lora_init_module(void)
{
    char command[64];

    uart_flush_input(LORA_UART_PORT);
    lora_send_command("AT", "+OK");
    snprintf(command, sizeof(command), "AT+ADDRESS=%d", LORA_NODE_ADDRESS);
    lora_send_command(command, "+OK");
    snprintf(command, sizeof(command), "AT+NETWORKID=%d", LORA_NETWORK_ID);
    lora_send_command(command, "+OK");
    snprintf(command, sizeof(command), "AT+BAND=%d", LORA_BAND_HZ);
    lora_send_command(command, "+OK");
    snprintf(command, sizeof(command), "AT+PARAMETER=%s", LORA_PARAMETER);
    lora_send_command(command, "+OK");
}

static bool lora_send_sensor_data(const char *payload)
{
    char command[180];
    int length = (int)strlen(payload);
    snprintf(command, sizeof(command), "AT+SEND=%d,%d,%s",
             LORA_RECEIVER_ADDRESS, length, payload);
    return lora_send_command(command, "+OK");
}

static float sort_and_trimmed_mean(const float *input, size_t count)
{
    float sorted[SAMPLES_PER_MINUTE];
    memcpy(sorted, input, count * sizeof(float));

    for (size_t i = 1; i < count; ++i) {
        float value = sorted[i];
        size_t j = i;
        while (j > 0 && sorted[j - 1] > value) {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = value;
    }

    size_t trim = count >= (TRIM_COUNT * 2 + 1) ? TRIM_COUNT : 0;
    float sum = 0.0f;
    size_t used = 0;
    for (size_t i = trim; i < count - trim; ++i) {
        sum += sorted[i];
        ++used;
    }
    return used > 0 ? sum / (float)used : 0.0f;
}

static void tracker_recalculate_baseline(baseline_tracker_t *tracker)
{
    float sum = 0.0f;
    for (size_t i = 0; i < tracker->minute_count; ++i) {
        sum += tracker->minute_values[i];
    }

    if (tracker->minute_count > 0) {
        tracker->baseline = sum / (float)tracker->minute_count;
    }
    tracker->ready = tracker->minute_count >= BASELINE_BUCKET_COUNT;
}

static tracker_result_t tracker_add_sample(baseline_tracker_t *tracker, float raw)
{
    tracker_result_t result = {
        .raw = raw,
        .filtered = raw,
        .accepted = true,
    };

    bool is_spike = false;
    if (tracker->ready) {
        float delta = fabsf(raw - tracker->baseline);
        float dynamic_limit = fabsf(tracker->baseline) * tracker->spike_ratio_threshold;
        float limit = fmaxf(tracker->spike_abs_threshold, dynamic_limit);
        is_spike = delta > limit;
    }

    if (!is_spike) {
        if (!tracker->has_ema) {
            tracker->ema = raw;
            tracker->has_ema = true;
        } else {
            tracker->ema = tracker->ema * 0.8f + raw * 0.2f;
        }

        if (tracker->sample_count < SAMPLES_PER_MINUTE) {
            tracker->samples[tracker->sample_count++] = tracker->ema;
        }
        result.filtered = tracker->ema;
    } else {
        result.accepted = false;
        result.filtered = tracker->has_ema ? tracker->ema : raw;
    }

    return result;
}

static void tracker_finish_minute(baseline_tracker_t *tracker)
{
    if (tracker->sample_count > 0) {
        float minute_value = sort_and_trimmed_mean(tracker->samples, tracker->sample_count);

        tracker->minute_values[tracker->next_minute_index] = minute_value;
        tracker->next_minute_index = (tracker->next_minute_index + 1) % BASELINE_BUCKET_COUNT;
        if (tracker->minute_count < BASELINE_BUCKET_COUNT) {
            ++tracker->minute_count;
        }
        tracker_recalculate_baseline(tracker);
    }

    tracker->sample_count = 0;
}

static bool adc_calibration_init(adc_unit_t unit, adc_atten_t attenuation,
                                 adc_cali_handle_t *out_handle)
{
    bool calibrated = false;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (unit == ADC_UNIT_1) {
        adc_cali_curve_fitting_config_t config = {
            .unit_id = unit,
            .chan = ADC_CHANNEL_0,
            .atten = attenuation,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_curve_fitting(&config, out_handle) == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated && unit == ADC_UNIT_1) {
        adc_cali_line_fitting_config_t config = {
            .unit_id = unit,
            .atten = attenuation,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_line_fitting(&config, out_handle) == ESP_OK) {
            calibrated = true;
        }
    }
#endif
    return calibrated;
}

static void adc_init_all(void)
{
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &s_adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, MQ7_ADC_CHANNEL, &channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, MQ8_ADC_CHANNEL, &channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, FSR402_ADC_CHANNEL, &channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, WATER_LEVEL_ADC_CHANNEL, &channel_config));

    s_adc_cali_enabled = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &s_adc_cali_handle);
    ESP_LOGI(TAG, "ADC calibration: %s", s_adc_cali_enabled ? "enabled" : "not available");
}

static esp_err_t adc_read_raw(adc_channel_t channel, int *raw_value)
{
    ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc_handle, channel, raw_value),
                        TAG, "ADC read failed");
    return ESP_OK;
}

static bool adc_raw_is_saturated(int raw_value)
{
    return raw_value >= (ADC_RAW_MAX - ADC_SATURATION_MARGIN);
}

static esp_err_t adc_raw_to_mv(int raw, int *millivolts)
{
    if (s_adc_cali_enabled) {
        return adc_cali_raw_to_voltage(s_adc_cali_handle, raw, millivolts);
    }

    /* Internal approximation only; voltage is not exposed as the sensor value. */
    *millivolts = (raw * 3300) / 4095;
    return ESP_OK;
}

static float mq_adc_mv_to_sensor_mv(float adc_mv)
{
    return adc_mv * (MQ_SENSOR_VCC_MV / MQ_ADC_FULL_SCALE_MV);
}

static mq_gas_result_t mq_calculate(float adc_raw, mq_calibration_t *calibration,
                                    bool accepted, bool baseline_ready)
{
    mq_gas_result_t result = {
        .valid = false,
    };

    int adc_mv = 0;
    if (adc_raw_to_mv((int)adc_raw, &adc_mv) != ESP_OK) {
        return result;
    }

    result.sensor_mv = mq_adc_mv_to_sensor_mv((float)adc_mv);
    if (result.sensor_mv <= 0.0f || result.sensor_mv >= MQ_SENSOR_VCC_MV) {
        return result;
    }

    result.rs_kohm = calibration->rl_kohm * (MQ_SENSOR_VCC_MV - result.sensor_mv) / result.sensor_mv;

    if (accepted && !calibration->r0_locked) {
        float r0_estimate = result.rs_kohm / calibration->clean_air_factor;
        if (!calibration->has_r0) {
            calibration->r0_kohm = r0_estimate;
            calibration->has_r0 = true;
        } else {
            calibration->r0_kohm = calibration->r0_kohm * (1.0f - MQ_R0_EMA_ALPHA)
                                + r0_estimate * MQ_R0_EMA_ALPHA;
        }

        if (baseline_ready) {
            calibration->r0_locked = true;
        }
    }

    result.r0_kohm = calibration->r0_kohm;
    if (!calibration->has_r0 || calibration->r0_kohm <= 0.0f) {
        return result;
    }

    result.ratio = result.rs_kohm / calibration->r0_kohm;
    if (result.ratio <= 0.0f) {
        return result;
    }

    result.ppm = calibration->curve_a * powf(result.ratio, calibration->curve_b);
    result.valid = isfinite(result.ppm);
    return result;
}

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

static esp_err_t sgp40_read_raw(uint16_t *raw_signal)
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

static void i2c_init_all(void)
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

static void print_mq_line(const char *name, const tracker_result_t *value,
                          const baseline_tracker_t *tracker,
                          const mq_gas_result_t *gas,
                          bool adc_saturated)
{
    const char *state = adc_saturated ? "ADC_SATURATED" :
                        (tracker->ready ? (value->accepted ? "OK" : "SPIKE_IGNORED")
                                        : "R0_CALIBRATING");

    if (gas->valid) {
        ESP_LOGI(TAG,
                 "%s raw=%.0f filtered=%.1f baseline=%.1f Rs=%.2fk R0=%.2fk ratio=%.3f ppm=%.1f state=%s",
                 name, value->raw, value->filtered, tracker->baseline,
                 gas->rs_kohm, gas->r0_kohm, gas->ratio, gas->ppm, state);
    } else {
        ESP_LOGW(TAG,
                 "%s raw=%.0f filtered=%.1f baseline=%.1f ppm=invalid state=%s",
                 name, value->raw, value->filtered, tracker->baseline, state);
    }
}

static void print_sensor_line(const char *name, const tracker_result_t *value,
                              const baseline_tracker_t *tracker, const char *unit)
{
    ESP_LOGI(TAG, "%s raw=%.1f%s filtered=%.1f%s baseline=%.1f%s state=%s",
             name, value->raw, unit, value->filtered, unit,
             tracker->baseline, unit,
             tracker->ready ? (value->accepted ? "OK" : "SPIKE_IGNORED") : "WARMUP");
}

static void print_analog_line(const char *name, const tracker_result_t *value,
                              const baseline_tracker_t *tracker)
{
    int millivolts = 0;
    if (adc_raw_to_mv((int)value->filtered, &millivolts) != ESP_OK) {
        ESP_LOGW(TAG, "%s raw=%.1f mV=invalid", name, value->raw);
        return;
    }

    ESP_LOGI(TAG, "%s raw=%.1f filtered=%.1f baseline=%.1f mV=%d state=%s",
             name, value->raw, value->filtered, tracker->baseline, millivolts,
             tracker->ready ? (value->accepted ? "OK" : "SPIKE_IGNORED") : "WARMUP");
}

void app_main(void)
{
    adc_init_all();
    i2c_init_all();
    lora_uart_init();
    lora_init_module();
    TickType_t next_sample = xTaskGetTickCount();

    baseline_tracker_t mq7 = {
        .spike_abs_threshold = 250.0f,
        .spike_ratio_threshold = 0.35f,
    };
    baseline_tracker_t mq8 = {
        .spike_abs_threshold = 250.0f,
        .spike_ratio_threshold = 0.35f,
    };
    baseline_tracker_t sgp40 = {
        .spike_abs_threshold = 1500.0f,
        .spike_ratio_threshold = 0.20f,
    };
    baseline_tracker_t fsr402 = {
        .spike_abs_threshold = 150.0f,
        .spike_ratio_threshold = 0.50f,
    };
    baseline_tracker_t water_level = {
        .spike_abs_threshold = 150.0f,
        .spike_ratio_threshold = 0.50f,
    };

    mq_calibration_t mq7_calibration = {
        .rl_kohm = MQ7_RL_KOHM,
        .clean_air_factor = MQ7_CLEAN_AIR_FACTOR,
        .curve_a = MQ7_PPM_CURVE_A,
        .curve_b = MQ7_PPM_CURVE_B,
    };
    mq_calibration_t mq8_calibration = {
        .rl_kohm = MQ8_RL_KOHM,
        .clean_air_factor = MQ8_CLEAN_AIR_FACTOR,
        .curve_a = MQ8_PPM_CURVE_A,
        .curve_b = MQ8_PPM_CURVE_B,
    };

    ESP_LOGI(TAG, "Start: baseline window=%d minutes, update interval=1 minute", BASELINE_BUCKET_COUNT);
    ESP_LOGI(TAG, "Pins: MQ7=GPIO0 MQ8=GPIO1 FSR402=GPIO2 WATER_LEVEL=GPIO3 SGP40 SDA=GPIO8 SCL=GPIO9");
    ESP_LOGI(TAG, "MQ setup: MQ7_RL=%.1fk MQ8_RL=%.1fk; baseline uses ADC raw counts",
             MQ7_RL_KOHM, MQ8_RL_KOHM);

    uint8_t mq7_danger_count = 0;
    uint8_t mq8_danger_count = 0;
    uint8_t fsr402_danger_count = 0;
    uint8_t water_level_danger_count = 0;
    uint8_t sgp40_danger_count = 0;

    while (true) {
        int mq7_raw = 0;
        int mq8_raw = 0;
        int fsr402_raw = 0;
        int water_level_raw = 0;
        uint16_t sgp40_raw = 0;

        if (adc_read_raw(MQ7_ADC_CHANNEL, &mq7_raw) != ESP_OK) {
            ESP_LOGW(TAG, "MQ7 read failed");
        }
        if (adc_read_raw(MQ8_ADC_CHANNEL, &mq8_raw) != ESP_OK) {
            ESP_LOGW(TAG, "MQ8 read failed");
        }
        if (adc_read_raw(FSR402_ADC_CHANNEL, &fsr402_raw) != ESP_OK) {
            ESP_LOGW(TAG, "FSR402 read failed");
        }
        if (adc_read_raw(WATER_LEVEL_ADC_CHANNEL, &water_level_raw) != ESP_OK) {
            ESP_LOGW(TAG, "water level read failed");
        }
        if (sgp40_read_raw(&sgp40_raw) != ESP_OK) {
            ESP_LOGW(TAG, "SGP40 read failed");
        }

        tracker_result_t mq7_result = tracker_add_sample(&mq7, (float)mq7_raw);
        tracker_result_t mq8_result = tracker_add_sample(&mq8, (float)mq8_raw);
        tracker_result_t sgp40_result = tracker_add_sample(&sgp40, (float)sgp40_raw);
        tracker_result_t fsr402_result = tracker_add_sample(&fsr402, (float)fsr402_raw);
        tracker_result_t water_level_result = tracker_add_sample(&water_level, (float)water_level_raw);

        mq_gas_result_t mq7_gas = mq_calculate(mq7_result.filtered, &mq7_calibration,
                                               mq7_result.accepted, mq7.ready);
        mq_gas_result_t mq8_gas = mq_calculate(mq8_result.filtered, &mq8_calibration,
                                               mq8_result.accepted, mq8.ready);

        bool mq7_adc_saturated = adc_raw_is_saturated(mq7_raw);
        bool mq8_adc_saturated = adc_raw_is_saturated(mq8_raw);

        print_mq_line("MQ7", &mq7_result, &mq7, &mq7_gas, mq7_adc_saturated);
        print_mq_line("MQ8", &mq8_result, &mq8, &mq8_gas, mq8_adc_saturated);
        print_sensor_line("SGP40", &sgp40_result, &sgp40, "raw");
        print_analog_line("FSR402", &fsr402_result, &fsr402);
        print_analog_line("WATER_LEVEL", &water_level_result, &water_level);

        bool mq7_signal_high = mq7.ready && !mq7_adc_saturated
                             && (float)mq7_raw > mq7.baseline * MQ7_RAW_DANGER_RATIO;
        bool mq8_signal_high = mq8.ready && !mq8_adc_saturated
                             && (float)mq8_raw > mq8.baseline * MQ8_RAW_DANGER_RATIO;
        bool fsr402_signal_high = fsr402.ready
                                && (float)fsr402_raw > fsr402.baseline * FSR402_RAW_DANGER_RATIO;
        bool water_level_signal_high = water_level.ready
                                     && (float)water_level_raw
                                            > water_level.baseline * WATER_LEVEL_RAW_DANGER_RATIO;
        bool sgp40_signal_low = sgp40.ready
                              && (float)sgp40_raw < sgp40.baseline * SGP40_RAW_DROP_DANGER_RATIO;

        mq7_danger_count = mq7_signal_high ? mq7_danger_count + 1 : 0;
        mq8_danger_count = mq8_signal_high ? mq8_danger_count + 1 : 0;
        fsr402_danger_count = fsr402_signal_high ? fsr402_danger_count + 1 : 0;
        water_level_danger_count = water_level_signal_high ? water_level_danger_count + 1 : 0;
        sgp40_danger_count = sgp40_signal_low ? sgp40_danger_count + 1 : 0;

        bool mq7_danger = mq7_danger_count >= DANGER_HOLD_SAMPLES;
        bool mq8_danger = mq8_danger_count >= DANGER_HOLD_SAMPLES;
        bool fsr402_danger = fsr402_danger_count >= DANGER_HOLD_SAMPLES;
        bool water_level_danger = water_level_danger_count >= DANGER_HOLD_SAMPLES;
        bool sgp40_danger = sgp40_danger_count >= DANGER_HOLD_SAMPLES;

        char alert[64] = "";
        size_t alert_length = 0;
        if (mq7_danger) {
            alert_length += (size_t)snprintf(alert + alert_length, sizeof(alert) - alert_length,
                                             "MQ7|");
        }
        if (mq8_danger) {
            alert_length += (size_t)snprintf(alert + alert_length, sizeof(alert) - alert_length,
                                             "MQ8|");
        }
        if (mq7_adc_saturated) {
            alert_length += (size_t)snprintf(alert + alert_length, sizeof(alert) - alert_length,
                                             "MQ7_SATURATED|");
        }
        if (mq8_adc_saturated) {
            alert_length += (size_t)snprintf(alert + alert_length, sizeof(alert) - alert_length,
                                             "MQ8_SATURATED|");
        }
        if (fsr402_danger) {
            alert_length += (size_t)snprintf(alert + alert_length, sizeof(alert) - alert_length,
                                             "FSR402|");
        }
        if (water_level_danger) {
            alert_length += (size_t)snprintf(alert + alert_length, sizeof(alert) - alert_length,
                                             "WATER_LEVEL|");
        }
        if (sgp40_danger) {
            alert_length += (size_t)snprintf(alert + alert_length, sizeof(alert) - alert_length,
                                             "SGP40|");
        }
        if (alert_length == 0) {
            snprintf(alert, sizeof(alert), "NONE");
        } else {
            alert[alert_length - 1] = '\0';
        }

        char payload[128];
        snprintf(payload, sizeof(payload),
                 "MQ7=%d,MQ8=%d,SGP=%u,FSR=%d,WATER=%d,ALERT=%s",
                 mq7_raw, mq8_raw, sgp40_raw, fsr402_raw, water_level_raw, alert);
        if (lora_send_sensor_data(payload)) {
            ESP_LOGI(TAG, "LoRa sent: %s", payload);
        }

        if (mq7_danger || mq8_danger || fsr402_danger || water_level_danger || sgp40_danger) {
            ESP_LOGW(TAG, "DANGER source=%s%s%s%s%s",
                     mq7_danger ? "MQ7 " : "",
                     mq8_danger ? "MQ8 " : "",
                     fsr402_danger ? "FSR402 " : "",
                     water_level_danger ? "WATER_LEVEL " : "",
                     sgp40_danger ? "SGP40" : "");
        } else if (mq7_adc_saturated || mq8_adc_saturated) {
            ESP_LOGW(TAG, "RISK source=%s%s",
                     mq7_adc_saturated ? "MQ7_ADC_SATURATED " : "",
                     mq8_adc_saturated ? "MQ8_ADC_SATURATED" : "");
        } else if (mq7.ready && mq8.ready && fsr402.ready && water_level.ready && sgp40.ready) {
            ESP_LOGI(TAG, "RISK source=none");
        } else {
            ESP_LOGI(TAG, "RISK source=baseline_warmup");
        }

        static uint32_t sample_number = 0;
        ++sample_number;
        if (sample_number % SAMPLES_PER_MINUTE == 0) {
            tracker_finish_minute(&mq7);
            tracker_finish_minute(&mq8);
            tracker_finish_minute(&sgp40);
            tracker_finish_minute(&fsr402);
            tracker_finish_minute(&water_level);
            ESP_LOGI(TAG,
                     "BASELINE UPDATED MQ7=%.1f raw R0=%.2fk MQ8=%.1f raw R0=%.2fk SGP40=%.1f raw ready=%s",
                     mq7.baseline, mq7_calibration.r0_kohm,
                     mq8.baseline, mq8_calibration.r0_kohm,
                     sgp40.baseline,
                     (mq7.ready && mq8.ready && sgp40.ready) ? "yes" : "no");
        }

        vTaskDelayUntil(&next_sample, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
