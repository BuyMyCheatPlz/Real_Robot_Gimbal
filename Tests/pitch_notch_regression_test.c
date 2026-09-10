#include "notch_filter.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>

#define SAMPLE_RATE_HZ 1000.0f
#define TWO_PI_F       6.2831853071795864769f

static float update_pitch_d_notches(NotchFilter_t *first, NotchFilter_t *second,
                                    float input)
{
    input = NotchFilter_Update(first, input);
    input = NotchFilter_Update(second, input);
    return input;
}

static float measure_gain(float frequency_hz)
{
    NotchFilter_t notch32;
    NotchFilter_t notch23;
    float input_energy = 0.0f;
    float output_energy = 0.0f;
    uint32_t sample;

    NotchFilter_Init(&notch32, SAMPLE_RATE_HZ, 32.0f, 1.5f);
    NotchFilter_Init(&notch23, SAMPLE_RATE_HZ, 23.7f, 1.5f);
    for (sample = 0U; sample < 4000U; ++sample)
    {
        float input = sinf(TWO_PI_F * frequency_hz *
                           (float)sample / SAMPLE_RATE_HZ);
        float output = update_pitch_d_notches(&notch32, &notch23, input);
        if (sample >= 2000U)
        {
            input_energy += input * input;
            output_energy += output * output;
        }
    }
    return sqrtf(output_energy / input_energy);
}

int main(void)
{
    NotchFilter_t filter;
    float output = 0.0f;
    uint32_t sample;

    /* MISSION 要求 300 ms 内到位，约 3.33 Hz 的动态不能被明显削弱。 */
    assert(measure_gain(3.33f) > 0.99f);

    /* 实测 -60° 处的 23.7 Hz 与 31.6~34.3 Hz 模态需要显著衰减。 */
    assert(measure_gain(23.7f) < 0.10f);
    assert(measure_gain(31.7f) < 0.10f);
    assert(measure_gain(34.3f) < 0.25f);

    /* 陷波器必须保持直流增益，不能引入稳态位置偏差。 */
    NotchFilter_Init(&filter, SAMPLE_RATE_HZ, 32.0f, 1.5f);
    for (sample = 0U; sample < 2000U; ++sample)
        output = NotchFilter_Update(&filter, 1.0f);
    assert(fabsf(output - 1.0f) < 0.001f);

    NotchFilter_Reset(&filter);
    assert(fabsf(NotchFilter_Update(&filter, 0.0f)) < 0.000001f);
    return 0;
}
