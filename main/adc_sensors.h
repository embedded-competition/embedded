#pragma once

#include <stdbool.h>

#include "esp_err.h"

void adc_sensors_init(void);

/* Each returns the ADC raw count, or 0 if the read failed. */
int adc_sensors_read_mq7(void);
int adc_sensors_read_mq8(void);
int adc_sensors_read_fsr402(void);
int adc_sensors_read_water_level(void);

bool adc_sensors_is_saturated(int raw_value);
esp_err_t adc_sensors_raw_to_mv(int raw, int *millivolts);

/* Maps an ADC raw count onto [0, 1000], clamped. */
int adc_sensors_normalize(float raw_value);
