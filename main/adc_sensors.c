#include "adc_sensors.h"

#include "board_pins.h"
#include "sensor_tracker.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "ADC_SENSORS";

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali_handle;
static bool s_adc_cali_enabled;

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

void adc_sensors_init(void)
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

static int adc_read_raw_or_zero(adc_channel_t channel)
{
    int raw_value = 0;
    if (adc_oneshot_read(s_adc_handle, channel, &raw_value) != ESP_OK) {
        return 0;
    }
    return raw_value;
}

int adc_sensors_read_mq7(void) { return adc_read_raw_or_zero(MQ7_ADC_CHANNEL); }
int adc_sensors_read_mq8(void) { return adc_read_raw_or_zero(MQ8_ADC_CHANNEL); }
int adc_sensors_read_fsr402(void) { return adc_read_raw_or_zero(FSR402_ADC_CHANNEL); }
int adc_sensors_read_water_level(void) { return adc_read_raw_or_zero(WATER_LEVEL_ADC_CHANNEL); }

bool adc_sensors_is_saturated(int raw_value)
{
    return raw_value >= (ADC_RAW_MAX - ADC_SATURATION_MARGIN);
}

esp_err_t adc_sensors_raw_to_mv(int raw, int *millivolts)
{
    if (s_adc_cali_enabled) {
        return adc_cali_raw_to_voltage(s_adc_cali_handle, raw, millivolts);
    }

    /* Internal approximation only; voltage is not exposed as the sensor value. */
    *millivolts = (raw * 3300) / 4095;
    return ESP_OK;
}

int adc_sensors_normalize(float raw_value)
{
    return normalize_absolute(raw_value, ADC_RAW_MAX);
}
