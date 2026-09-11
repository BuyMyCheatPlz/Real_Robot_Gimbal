#include "config.h"
#include "pitch_approach.h"
#include <assert.h>
#include <math.h>

static int nearly_equal(float lhs, float rhs, float tolerance)
{
    return fabsf(lhs - rhs) <= tolerance;
}

static float expected_stop_speed(float acceleration, float delay_s,
                                 float error_deg)
{
    float acceleration_delay = acceleration * delay_s;
    return sqrtf(acceleration_delay * acceleration_delay +
                 2.0f * acceleration * error_deg) - acceleration_delay;
}

static float brake_once(float error_deg, float measurement_deg,
                        float speed_deg_s)
{
    PitchApproachState_t state;
    PitchApproach_Reset(&state);
    return PitchApproach_UpdateBrakeFeedforward(
        &state, error_deg, measurement_deg, speed_deg_s, -1.0f, 1.0f);
}

int main(void)
{
    PitchApproachState_t state;
    float limited;
    float brake;

    PitchApproach_Reset(&state);
    assert(state.brake_voltage == 0.0f);
    brake = PitchApproach_UpdateBrakeFeedforward(
        &state, -4.0f, -26.0f, -200.0f, -1.0f, 0.001f);
    assert(nearly_equal(brake,
                        -PITCH_APPROACH_BRAKE_FF_SLEW_VOLT_PER_S * 0.001f,
                        0.01f));
    brake = PitchApproach_UpdateBrakeFeedforward(
        &state, -4.0f, -26.0f, -200.0f, -1.0f, 0.001f);
    assert(nearly_equal(brake,
                        -2.0f * PITCH_APPROACH_BRAKE_FF_SLEW_VOLT_PER_S *
                        0.001f, 0.01f));
    PitchApproach_Reset(&state);
    assert(state.brake_voltage == 0.0f);

    /* 接近限制只能缩小朝向目标的速度，不能主动生成反向速度目标。 */
    limited = PitchApproach_LimitSpeed(300.0f, 1.0f, -1.0f);
    assert((limited > 0.0f) && (limited < 300.0f));
    assert(PitchApproach_LimitSpeed(-100.0f, 1.0f, -1.0f) == 0.0f);
    assert(PitchApproach_LimitSpeed(100.0f, -1.0f, -29.0f) == 0.0f);

    /* 停车速度计入响应延迟，低目标向下比普通向下更早收速。 */
    limited = PitchApproach_LimitSpeed(200.0f, 4.0f, -34.0f);
    assert(nearly_equal(limited,
                        expected_stop_speed(
                            PITCH_APPROACH_BRAKE_ACCEL_DEG_S2 *
                            PITCH_APPROACH_LOW_ANGLE_SCALE,
                            PITCH_APPROACH_LOW_ANGLE_UP_BRAKE_DELAY_S,
                            4.0f),
                        0.01f));
    limited = PitchApproach_LimitSpeed(-200.0f, -4.0f, -56.0f);
    assert(nearly_equal(limited,
                        -expected_stop_speed(
                            PITCH_APPROACH_BRAKE_ACCEL_DEG_S2,
                            PITCH_APPROACH_LOW_TARGET_DOWN_BRAKE_DELAY_S,
                            4.0f),
                        0.01f));
    assert(fabsf(limited) < fabsf(PitchApproach_LimitSpeed(
        -200.0f, -4.0f, -26.0f)));

    /* 低角度向上仍按 6→4° 渐入，高角度向上使用较小制动限幅。 */
    assert(brake_once(6.0f, -36.0f, 200.0f) == 0.0f);
    assert(brake_once(5.0f, -35.0f, 200.0f) > 0.0f);
    brake = brake_once(0.1f, -0.1f, 200.0f);
    assert((brake > 0.0f) &&
           (brake <= PITCH_APPROACH_HIGH_ANGLE_UP_BRAKE_FF_LIMIT));

    /* 穿越目标后误差虽已换向，只要仍有远离目标的速度就继续制动。 */
    brake = brake_once(-0.1f, -29.9f, 30.0f);
    assert(brake > 0.0f);
    assert(brake > brake_once(-0.1f, 0.1f, 30.0f));
    brake = brake_once(0.1f, -30.1f, -30.0f);
    assert(brake < 0.0f);
    assert(brake_once(-0.1f, -29.9f, 2.9f) == 0.0f);

    /* 静差补偿越过 MAX 后保持端点值，不允许在边界突然归零。 */
    assert(nearly_equal(PitchApproach_StaticErrorComp(-0.2f, 0.0f, -1.0f),
                        2000.0f, 0.01f));
    assert(nearly_equal(PitchApproach_StaticErrorComp(0.2f, 0.0f, -1.0f),
                        -2000.0f, 0.01f));
    assert(PitchApproach_StaticErrorComp(0.09f, 0.0f, -1.0f) == 0.0f);
    assert(nearly_equal(PitchApproach_StaticErrorComp(0.99f, 0.0f, -1.0f),
                        PitchApproach_StaticErrorComp(1.01f, 0.0f, -1.0f),
                        0.01f));
    assert(PitchApproach_StaticErrorComp(0.2f, 8.0f, -1.0f) == 0.0f);

    return 0;
}
