#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "adc_sensors.h"
#include "gps_link.h"
#include "lora_link.h"
#include "mq_gas.h"
#include "sensor_tracker.h"
#include "sgp40_sensor.h"
#include "status_display.h"

#define SAMPLE_PERIOD_MS             1000

#define MQ7_RAW_DANGER_RATIO         1.50f
#define MQ8_RAW_DANGER_RATIO         1.50f
#define FSR402_RAW_DANGER_RATIO      1.50f
#define WATER_LEVEL_RAW_DANGER_RATIO 1.50f
#define SGP40_RAW_DROP_DANGER_RATIO  0.80f
#define DANGER_HOLD_SAMPLES          3

static const char *TAG = "GAS_MONITOR";

void app_main(void)
{
    uint8_t device_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(device_mac, ESP_MAC_WIFI_STA));

    adc_sensors_init();
    sgp40_sensor_init();
    lora_link_init();
    gps_link_init();
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
        /* A pressure event is the intended signal for the FSR. Do not
         * discard a fast change as a spike; tracker EMA still smooths it. */
        .spike_abs_threshold = 4095.0f,
        .spike_ratio_threshold = 10.0f,
    };
    baseline_tracker_t water_level = {
        /* Water level is used as a binary wet/dry check. A dry->wet
         * transition is a real, sustained step change, not noise to
         * reject -- do not spike-filter it out. */
        .spike_abs_threshold = 4095.0f,
        .spike_ratio_threshold = 10.0f,
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
        gps_link_update();

        int mq7_raw = adc_sensors_read_mq7();
        int mq8_raw = adc_sensors_read_mq8();
        int fsr402_raw = adc_sensors_read_fsr402();
        int water_level_raw = adc_sensors_read_water_level();
        uint16_t sgp40_raw = 0;
        sgp40_sensor_read_raw(&sgp40_raw); /* leaves sgp40_raw at 0 on failure */

        tracker_result_t mq7_result = tracker_add_sample(&mq7, (float)mq7_raw);
        tracker_result_t mq8_result = tracker_add_sample(&mq8, (float)mq8_raw);
        tracker_result_t sgp40_result = tracker_add_sample(&sgp40, (float)sgp40_raw);
        tracker_result_t fsr402_result = tracker_add_sample(&fsr402, (float)fsr402_raw);
        tracker_result_t water_level_result = tracker_add_sample(&water_level, (float)water_level_raw);

        mq_gas_result_t mq7_gas = mq_calculate(mq7_result.filtered, &mq7_calibration,
                                               mq7_result.accepted, mq7.ready);
        mq_gas_result_t mq8_gas = mq_calculate(mq8_result.filtered, &mq8_calibration,
                                               mq8_result.accepted, mq8.ready);

        bool mq7_adc_saturated = adc_sensors_is_saturated(mq7_raw);
        bool mq8_adc_saturated = adc_sensors_is_saturated(mq8_raw);

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

        uint16_t mq7_value = (uint16_t)adc_sensors_normalize(mq7_result.filtered);
        uint16_t mq8_value = (uint16_t)adc_sensors_normalize(mq8_result.filtered);
        uint16_t pressure_value = (uint16_t)adc_sensors_normalize(fsr402_result.filtered);
        uint16_t water_value = (uint16_t)adc_sensors_normalize(water_level_result.filtered);
        uint16_t voc_value = (uint16_t)sgp40_sensor_normalize(sgp40_result.filtered);

        float gps_lat = 0.0f;
        float gps_lon = 0.0f;
        bool gps_has_fix = gps_link_get_fix(&gps_lat, &gps_lon);

        bool lora_ok = lora_link_send_packet(device_mac, mq7_value, mq8_value,
                                             pressure_value, water_value, voc_value,
                                             gps_lat, gps_lon);

        const char *state;
        if (mq7_danger || mq8_danger || fsr402_danger || water_level_danger || sgp40_danger) {
            state = "DANGER";
        } else if (mq7_adc_saturated || mq8_adc_saturated) {
            state = "RISK";
        } else if (mq7.ready && mq8.ready && fsr402.ready && water_level.ready && sgp40.ready) {
            state = "OK";
        } else {
            state = "WARMUP";
        }

        status_display_show(&mq7_result, &mq7, &mq7_gas, mq7_adc_saturated,
                            &mq8_result, &mq8, &mq8_gas, mq8_adc_saturated,
                            &sgp40_result, &sgp40,
                            &fsr402_result, &fsr402,
                            &water_level_result, &water_level,
                            gps_has_fix, gps_lat, gps_lon,
                            state, alert, lora_ok);

        static uint32_t sample_number = 0;
        ++sample_number;
        if (sample_number % SAMPLES_PER_MINUTE == 0) {
            tracker_finish_minute(&mq7);
            tracker_finish_minute(&mq8);
            tracker_finish_minute(&sgp40);
            tracker_finish_minute(&fsr402);
            /* Freeze the water baseline once the initial (assumed-dry)
             * calibration window locks in, the same way mq_gas locks R0.
             * Otherwise a prolonged flood would slowly become the new
             * "normal" baseline and the wet/dry check would stop firing. */
            if (!water_level.ready) {
                tracker_finish_minute(&water_level);
            }
        }

        vTaskDelayUntil(&next_sample, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
