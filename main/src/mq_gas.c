#include "mq_gas.h"

#include <math.h>

#include "adc_sensors.h"
#include "esp_err.h"

static float mq_adc_mv_to_sensor_mv(float adc_mv)
{
    return adc_mv * (MQ_SENSOR_VCC_MV / MQ_ADC_FULL_SCALE_MV);
}

mq_gas_result_t mq_calculate(float adc_raw, mq_calibration_t *calibration,
                             bool accepted, bool baseline_ready)
{
    mq_gas_result_t result = {
        .valid = false,
    };

    int adc_mv = 0;
    if (adc_sensors_raw_to_mv((int)adc_raw, &adc_mv) != ESP_OK) {
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
