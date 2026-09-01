#pragma once

#include <stdbool.h>

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

mq_gas_result_t mq_calculate(float adc_raw, mq_calibration_t *calibration,
                             bool accepted, bool baseline_ready);
