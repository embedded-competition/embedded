#include "sensor_tracker.h"

#include <math.h>
#include <string.h>

#define TRIM_COUNT 6 /* 10% from both ends of 60 samples */

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

tracker_result_t tracker_add_sample(baseline_tracker_t *tracker, float raw)
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

void tracker_finish_minute(baseline_tracker_t *tracker)
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

int normalize_absolute(float value, float full_scale)
{
    if (full_scale <= 0.0f) {
        return 0;
    }
    return (int)fminf(NORMALIZED_MAX, fmaxf(0.0f,
                     value * NORMALIZED_MAX / full_scale));
}
