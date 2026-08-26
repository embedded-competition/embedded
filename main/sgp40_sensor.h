#pragma once

#include <stdint.h>

#include "esp_err.h"

void sgp40_sensor_init(void);
esp_err_t sgp40_sensor_read_raw(uint16_t *raw_signal);

/* SGP40 raw signal falls as VOC increases; maps it onto [0, 1000] inverted. */
int sgp40_sensor_normalize(float raw_value);
