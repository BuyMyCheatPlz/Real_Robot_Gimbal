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
    float static_comp;

    PitchApproach_Reset(&state);
    assert(state.brake_voltage == 0.0f);
    assert(state.static_voltage == 0.0f);
    brake = PitchApproach_UpdateBrakeFeedforward(
        &state, -4.0f, -26.0f, -200.0f, -1.0f, 0.001f);
    assert(brake == 0.0f);
    PitchApproach_Reset(&state);
    assert(state.brake_voltage == 0.0f);

    /* 接近限制只能缩小朝向目标的速度，不能主动生成反向速度目标。 */
    limited = PitchApproach_LimitSpeed(300.0f, 1.0f, -1.0f);
    assert((limited > 0.0f) && (limited < 300.0f));
    assert(PitchApproach_LimitSpeed(-100.0f, 1.0f, -1.0f) == 0.0f);
    assert(PitchApproach_LimitSpeed(100.0f, -1.0f, -29.0f) == 0.0f);

    /* 停车速度计入响应延迟，低目标向下不能比普通向下更晚收速。 */
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
    assert(fabsf(limited) <= fabsf(PitchApproach_LimitSpeed(
        -200.0f, -4.0f, -26.0f)));

    /* 0→-30 在剩余 2° 时收至 50°/s，降低下降沿越线后的回摆。 */
    limited = PitchApproach_LimitSpeed(-200.0f, -2.0f, -28.0f);
    assert(nearly_equal(fabsf(limited), 50.0f, 0.1f));

    /* 实测上升段的代表误差点必须提前收速：降低 -60→-30 穿越速度，
     * 并把 -30→0 的制动分摊到末段，而不是到目标前才打满。 */
    limited = PitchApproach_LimitSpeed(300.0f, 2.168f, -32.168f);
    assert(limited < 50.0f);
    limited = PitchApproach_LimitSpeed(300.0f, 5.7f, -5.7f);
    assert(limited < 115.0f);

    /* -30→0 在剩余 2° 时收至约 34°/s，继续压低接近 0° 时的残余速度。 */
    limited = PitchApproach_LimitSpeed(200.0f, 2.0f, -2.0f);
    assert(nearly_equal(limited, 34.2f, 0.1f));

    /* 动态制动由速度 PI 承担，前馈通道保持为零。 */
    assert(brake_once(5.0f, -35.0f, 200.0f) == 0.0f);
    assert(brake_once(-0.1f, -29.9f, 30.0f) == 0.0f);

    /* 静差补偿每毫秒最多变化 40，不能随误差和速度瞬间跳变。 */
    PitchApproach_Reset(&state);
    static_comp = PitchApproach_UpdateStaticErrorComp(
        &state, -0.2f, -29.8f, 0.0f, -1.0f, 0.001f);
    assert(nearly_equal(static_comp, 40.0f, 0.01f));
    static_comp = PitchApproach_UpdateStaticErrorComp(
        &state, -0.2f, -29.8f, 0.0f, -1.0f, 0.001f);
    assert(nearly_equal(static_comp, 80.0f, 0.01f));

    /* -45° 以下使用较低静差增益和限幅，其他角度保留原补偿能力。 */
    PitchApproach_Reset(&state);
    static_comp = PitchApproach_UpdateStaticErrorComp(
        &state, -0.3f, -59.7f, 0.0f, -1.0f, 1.0f);
    assert(nearly_equal(static_comp, 1000.0f, 0.01f));
    PitchApproach_Reset(&state);
    static_comp = PitchApproach_UpdateStaticErrorComp(
        &state, -0.3f, -29.7f, 0.0f, -1.0f, 1.0f);
    assert(nearly_equal(static_comp, 3200.0f, 0.01f));

    /* 进入死区后也按斜率释放，不允许把已有补偿瞬间撤掉。 */
    state.static_voltage = 100.0f;
    static_comp = PitchApproach_UpdateStaticErrorComp(
        &state, 0.07f, -30.07f, 0.0f, -1.0f, 0.001f);
    assert(nearly_equal(static_comp, 60.0f, 0.01f));

    return 0;
}
