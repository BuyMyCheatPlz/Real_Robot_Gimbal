#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float x1;
    float x2;
    float y1;
    float y2;
} NotchFilter_t;

void NotchFilter_Init(NotchFilter_t *filter, float sample_rate_hz,
                      float center_frequency_hz, float quality_factor);
void NotchFilter_Reset(NotchFilter_t *filter);
float NotchFilter_Update(NotchFilter_t *filter, float input);

#endif
