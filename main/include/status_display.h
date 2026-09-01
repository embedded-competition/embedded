#pragma once

#include <stdbool.h>

#include "mq_gas.h"
#include "sensor_tracker.h"

void status_display_show(const tracker_result_t *mq7_result, const baseline_tracker_t *mq7,
                         const mq_gas_result_t *mq7_gas, bool mq7_adc_saturated,
                         const tracker_result_t *mq8_result, const baseline_tracker_t *mq8,
                         const mq_gas_result_t *mq8_gas, bool mq8_adc_saturated,
                         const tracker_result_t *sgp40_result, const baseline_tracker_t *sgp40,
                         const tracker_result_t *fsr402_result, const baseline_tracker_t *fsr402,
                         const tracker_result_t *water_level_result,
                         const baseline_tracker_t *water_level,
                         bool gps_has_fix, float gps_lat, float gps_lon,
                         const char *overall_state, const char *alert, bool lora_ok);
