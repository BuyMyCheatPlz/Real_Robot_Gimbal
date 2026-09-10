#include "notch_filter.h"
#include <math.h>
#include <string.h>

#define NOTCH_TWO_PI_F 6.2831853071795864769f

void NotchFilter_Init(NotchFilter_t *filter, float sample_rate_hz,
                      float center_frequency_hz, float quality_factor)
{
    float angular_frequency;
    float alpha;
    float inverse_a0;

    if (filter == 0) return;
    memset(filter, 0, sizeof(*filter));
    filter->b0 = 1.0f;
    if ((sample_rate_hz <= 0.0f) ||
        (center_frequency_hz <= 0.0f) ||
        (center_frequency_hz >= 0.5f * sample_rate_hz) ||
        (quality_factor <= 0.0f))
        return;

    angular_frequency = NOTCH_TWO_PI_F * center_frequency_hz /
                        sample_rate_hz;
    alpha = sinf(angular_frequency) / (2.0f * quality_factor);
    inverse_a0 = 1.0f / (1.0f + alpha);

    filter->b0 = inverse_a0;
    filter->b1 = -2.0f * cosf(angular_frequency) * inverse_a0;
    filter->b2 = inverse_a0;
    filter->a1 = filter->b1;
    filter->a2 = (1.0f - alpha) * inverse_a0;
}

void NotchFilter_Reset(NotchFilter_t *filter)
{
    if (filter == 0) return;
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

float NotchFilter_Update(NotchFilter_t *filter, float input)
{
    float output;
    if (filter == 0) return input;

    output = filter->b0 * input + filter->b1 * filter->x1 +
             filter->b2 * filter->x2 - filter->a1 * filter->y1 -
             filter->a2 * filter->y2;
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    return output;
}
