#include "status_display.h"

#include <stdio.h>

#include "adc_sensors.h"
#include "sgp40_sensor.h"

#define STATUS_BLOCK_LINE_COUNT 7

static void print_status_row(const char *name, const tracker_result_t *value,
                             const baseline_tracker_t *tracker, const char *state,
                             int normalized, int normalized_baseline,
                             const char *extra)
{
    printf("%-12s value=%4d/1000 base=%4d/1000 state=%-15s%s\n",
           name, normalized, normalized_baseline, state, extra);
}

void status_display_show(const tracker_result_t *mq7_result, const baseline_tracker_t *mq7,
                         const mq_gas_result_t *mq7_gas, bool mq7_adc_saturated,
                         const tracker_result_t *mq8_result, const baseline_tracker_t *mq8,
                         const mq_gas_result_t *mq8_gas, bool mq8_adc_saturated,
                         const tracker_result_t *sgp40_result, const baseline_tracker_t *sgp40,
                         const tracker_result_t *fsr402_result, const baseline_tracker_t *fsr402,
                         const tracker_result_t *water_level_result,
                         const baseline_tracker_t *water_level,
                         bool gps_has_fix, float gps_lat, float gps_lon,
                         const char *overall_state, const char *alert, bool lora_ok)
{
    static bool first_draw = true;
    /* Reuse the same terminal lines instead of appending a new block. */
    if (!first_draw) {
        printf("\033[%uA", (unsigned)STATUS_BLOCK_LINE_COUNT);
    }
    first_draw = false;

    char extra[24];

    const char *mq7_state = mq7_adc_saturated ? "ADC_SATURATED" :
                            (mq7->ready ? (mq7_result->accepted ? "OK" : "SPIKE_IGNORED")
                                        : "R0_CALIBRATING");
    if (mq7_gas->valid) {
        snprintf(extra, sizeof(extra), " ppm=%10.1f", mq7_gas->ppm);
    } else {
        snprintf(extra, sizeof(extra), " ppm=%10s", "n/a");
    }
    print_status_row("MQ7", mq7_result, mq7, mq7_state,
                     adc_sensors_normalize(mq7_result->filtered),
                     mq7->ready ? adc_sensors_normalize(mq7->baseline) : 0, extra);

    const char *mq8_state = mq8_adc_saturated ? "ADC_SATURATED" :
                            (mq8->ready ? (mq8_result->accepted ? "OK" : "SPIKE_IGNORED")
                                        : "R0_CALIBRATING");
    if (mq8_gas->valid) {
        snprintf(extra, sizeof(extra), " ppm=%10.1f", mq8_gas->ppm);
    } else {
        snprintf(extra, sizeof(extra), " ppm=%10s", "n/a");
    }
    print_status_row("MQ8", mq8_result, mq8, mq8_state,
                     adc_sensors_normalize(mq8_result->filtered),
                     mq8->ready ? adc_sensors_normalize(mq8->baseline) : 0, extra);

    const char *sgp40_state = sgp40->ready ? (sgp40_result->accepted ? "OK" : "SPIKE_IGNORED") : "WARMUP";
    print_status_row("SGP40", sgp40_result, sgp40, sgp40_state,
                     sgp40_sensor_normalize(sgp40_result->filtered),
                     sgp40->ready ? sgp40_sensor_normalize(sgp40->baseline) : 0, "");

    const char *fsr402_state = fsr402->ready ? (fsr402_result->accepted ? "OK" : "SPIKE_IGNORED") : "WARMUP";
    print_status_row("FSR402", fsr402_result, fsr402, fsr402_state,
                     adc_sensors_normalize(fsr402_result->filtered),
                     fsr402->ready ? adc_sensors_normalize(fsr402->baseline) : 0, "");

    const char *water_level_state = water_level->ready
                                   ? (water_level_result->accepted ? "OK" : "SPIKE_IGNORED")
                                   : "WARMUP";
    print_status_row("WATER_LEVEL", water_level_result, water_level, water_level_state,
                     adc_sensors_normalize(water_level_result->filtered),
                     water_level->ready ? adc_sensors_normalize(water_level->baseline) : 0, "");

    if (gps_has_fix) {
        printf("\r\033[K%-12s lat=%11.6f lon=%11.6f fix=YES\n", "GPS", gps_lat, gps_lon);
    } else {
        printf("\r\033[K%-12s lat=%11s lon=%11s fix=NO\n", "GPS", "n/a", "n/a");
    }

    printf("\r\033[K""STATE=%-8s ALERT=%-28s LORA=%s\n", overall_state, alert, lora_ok ? "OK" : "FAIL");

    fflush(stdout);
}
