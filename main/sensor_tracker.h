#pragma once

#include <stdbool.h>
#include <stddef.h>

#define SAMPLES_PER_MINUTE           60
#define BASELINE_BUCKET_COUNT        3
#define NORMALIZED_MAX               1000.0f

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

tracker_result_t tracker_add_sample(baseline_tracker_t *tracker, float raw);
void tracker_finish_minute(baseline_tracker_t *tracker);

/* Maps value in [0, full_scale] onto [0, 1000], clamped. */
int normalize_absolute(float value, float full_scale);
