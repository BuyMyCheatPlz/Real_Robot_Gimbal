#include "pitch_approach.h"
#include "config.h"
#include <math.h>
#include <stdint.h>

static float pitch_sign(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

static float pitch_clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float pitch_smoothstep(float value)
{
    if (value <= 0.0f) return 0.0f;
    if (value >= 1.0f) return 1.0f;
    return value * value * (3.0f - 2.0f * value);
}

typedef struct
{
    float acceleration_deg_s2;
    float delay_s;
    float gain;
    float voltage_limit;
    float fade;
} PitchBrakeProfile_t;

static PitchBrakeProfile_t pitch_brake_profile(float error_deg,
                                               float measurement_deg,
                                               float motion_direction)
{
    PitchBrakeProfile_t profile;
    float target_deg = measurement_deg + error_deg;

    profile.acceleration_deg_s2 = PITCH_APPROACH_BRAKE_ACCEL_DEG_S2;
    profile.delay_s = 0.0f;
    profile.gain = PITCH_APPROACH_DOWN_BRAKE_FF_GAIN;
    profile.voltage_limit = PITCH_APPROACH_DOWN_BRAKE_FF_LIMIT;
    profile.fade = 1.0f;

    if (motion_direction < 0.0f)
    {
        profile.delay_s = PITCH_APPROACH_DOWN_BRAKE_DELAY_S;
        if (target_deg <= PITCH_APPROACH_LOW_TARGET_DOWN_MAX_DEG)
            profile.delay_s =
                PITCH_APPROACH_LOW_TARGET_DOWN_BRAKE_DELAY_S;
    }
    else if ((motion_direction > 0.0f) &&
             (target_deg <= PITCH_APPROACH_LOW_ANGLE_TARGET_MAX_DEG))
    {
#if (PITCH_APPROACH_LOW_ANGLE_SCALE_ENABLE != 0U)
        float scale = PITCH_APPROACH_LOW_ANGLE_SCALE;
        if (scale < 0.0f) scale = 0.0f;
        if (scale > 1.0f) scale = 1.0f;
        profile.acceleration_deg_s2 *= scale;
#endif
        profile.delay_s = PITCH_APPROACH_LOW_ANGLE_UP_BRAKE_DELAY_S;
        profile.gain = PITCH_APPROACH_LOW_ANGLE_UP_BRAKE_FF_GAIN;
        profile.voltage_limit =
            PITCH_APPROACH_LOW_ANGLE_UP_BRAKE_FF_LIMIT;
        if (PITCH_APPROACH_LOW_ANGLE_BRAKE_START_ERROR_DEG >
            PITCH_APPROACH_LOW_ANGLE_BRAKE_FULL_ERROR_DEG)
        {
            profile.fade = pitch_smoothstep(
                (PITCH_APPROACH_LOW_ANGLE_BRAKE_START_ERROR_DEG -
                 fabsf(error_deg)) /
                (PITCH_APPROACH_LOW_ANGLE_BRAKE_START_ERROR_DEG -
                 PITCH_APPROACH_LOW_ANGLE_BRAKE_FULL_ERROR_DEG));
        }
    }
    else if (motion_direction > 0.0f)
    {
        profile.acceleration_deg_s2 *=
            PITCH_APPROACH_HIGH_ANGLE_UP_ACCEL_SCALE;
        profile.delay_s = PITCH_APPROACH_HIGH_ANGLE_UP_BRAKE_DELAY_S;
        profile.gain = PITCH_APPROACH_HIGH_ANGLE_UP_BRAKE_FF_GAIN;
        profile.voltage_limit =
            PITCH_APPROACH_HIGH_ANGLE_UP_BRAKE_FF_LIMIT;
    }
    return profile;
}

static float pitch_stop_speed(float abs_error,
                              const PitchBrakeProfile_t *profile)
{
    float acceleration_delay;
    if ((profile == 0) || (profile->acceleration_deg_s2 <= 0.0f))
        return 0.0f;
    acceleration_delay = profile->acceleration_deg_s2 * profile->delay_s;
    return sqrtf(acceleration_delay * acceleration_delay +
                 2.0f * profile->acceleration_deg_s2 * abs_error) -
           acceleration_delay;
}

float PitchApproach_LimitSpeed(float speed_target_deg_s,
                               float error_deg,
                               float measurement_deg)
{
#if (PITCH_APPROACH_SPEED_LIMIT_ENABLE != 0U)
    float direction = pitch_sign(error_deg);
    float toward_target_speed = speed_target_deg_s * direction;
    PitchBrakeProfile_t profile = pitch_brake_profile(
        error_deg, measurement_deg, direction);
    float speed_limit;
    float min_speed = PITCH_APPROACH_MIN_SPEED_DEG_S;

    if ((direction == 0.0f) || (toward_target_speed <= 0.0f))
        return 0.0f;
    if (profile.acceleration_deg_s2 <= 0.0f) return speed_target_deg_s;

    speed_limit = pitch_stop_speed(fabsf(error_deg), &profile);
#if (PITCH_APPROACH_MIN_SPEED_FADE_ENABLE != 0U)
    if (PITCH_APPROACH_MIN_SPEED_FADE_ERROR_DEG > 0.0f)
    {
        float blend = fabsf(error_deg) /
                      PITCH_APPROACH_MIN_SPEED_FADE_ERROR_DEG;
        if (blend < 0.0f) blend = 0.0f;
        if (blend > 1.0f) blend = 1.0f;
        min_speed = PITCH_APPROACH_MIN_SPEED_NEAR_DEG_S +
            (PITCH_APPROACH_MIN_SPEED_DEG_S -
             PITCH_APPROACH_MIN_SPEED_NEAR_DEG_S) * blend;
    }
#endif
    if (speed_limit < min_speed) speed_limit = min_speed;
    if (toward_target_speed > speed_limit)
        toward_target_speed = speed_limit;
    return direction * toward_target_speed;
#else
    (void)error_deg;
    (void)measurement_deg;
    return speed_target_deg_s;
#endif
}

void PitchApproach_Reset(PitchApproachState_t *state)
{
    if (state != 0) state->brake_voltage = 0.0f;
}

float PitchApproach_UpdateBrakeFeedforward(PitchApproachState_t *state,
                                           float error_deg,
                                           float measurement_deg,
                                           float speed_actual_deg_s,
                                           float control_to_motor_sign,
                                           float dt_s)
{
#if (PITCH_APPROACH_BRAKE_FF_ENABLE != 0U)
    float abs_error = fabsf(error_deg);
    float abs_speed = fabsf(speed_actual_deg_s);
    float motion_direction = pitch_sign(speed_actual_deg_s);
    PitchBrakeProfile_t profile = pitch_brake_profile(
        error_deg, measurement_deg, motion_direction);
    float stop_speed;
    float overspeed;
    float target_voltage = 0.0f;
    float max_delta;
    float delta;

    if (state == 0) return 0.0f;
    if ((dt_s <= 0.0f) ||
        (PITCH_APPROACH_BRAKE_FF_SLEW_VOLT_PER_S <= 0.0f))
    {
        PitchApproach_Reset(state);
        return 0.0f;
    }

    stop_speed = pitch_stop_speed(abs_error, &profile);
    if (stop_speed < PITCH_APPROACH_BRAKE_FF_STOP_SPEED_DEG_S)
        stop_speed = PITCH_APPROACH_BRAKE_FF_STOP_SPEED_DEG_S;
    overspeed = abs_speed - stop_speed;
    if ((overspeed > 0.0f) && (profile.fade > 0.0f))
    {
        target_voltage = overspeed * profile.gain * profile.fade;
        if (target_voltage > profile.voltage_limit)
            target_voltage = profile.voltage_limit;
        /* 电机坐标与物理轴坐标可能反向；制动力始终与物理速度相反。 */
        target_voltage *= -control_to_motor_sign * motion_direction;
    }

    if ((target_voltage != 0.0f) &&
        ((target_voltage * state->brake_voltage) < 0.0f))
        state->brake_voltage = 0.0f;
    max_delta = PITCH_APPROACH_BRAKE_FF_SLEW_VOLT_PER_S * dt_s;
    delta = target_voltage - state->brake_voltage;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    state->brake_voltage += delta;
    return state->brake_voltage;
#else
    (void)state;
    (void)error_deg;
    (void)measurement_deg;
    (void)speed_actual_deg_s;
    (void)control_to_motor_sign;
    (void)dt_s;
    return 0.0f;
#endif
}

float PitchApproach_StaticErrorComp(float error_deg,
                                    float speed_deg_s,
                                    float control_to_motor_sign)
{
#if (PITCH_STATIC_ERROR_COMP_ENABLE != 0U)
    float abs_error = fabsf(error_deg);
    float abs_speed = fabsf(speed_deg_s);
    float effective_error;
    float voltage;
    float speed_scale = 1.0f;

    if ((abs_error <= PITCH_STATIC_ERROR_COMP_DEADBAND_DEG) ||
        (abs_speed >= PITCH_STATIC_ERROR_COMP_FADE_SPEED_DEG_S))
        return 0.0f;

    effective_error = abs_error;
    if (effective_error > PITCH_STATIC_ERROR_COMP_MAX_ERROR_DEG)
        effective_error = PITCH_STATIC_ERROR_COMP_MAX_ERROR_DEG;
    if (PITCH_STATIC_ERROR_COMP_FADE_SPEED_DEG_S >
        PITCH_STATIC_ERROR_COMP_FULL_SPEED_DEG_S)
    {
        if (abs_speed > PITCH_STATIC_ERROR_COMP_FULL_SPEED_DEG_S)
        {
            speed_scale =
                (PITCH_STATIC_ERROR_COMP_FADE_SPEED_DEG_S - abs_speed) /
                (PITCH_STATIC_ERROR_COMP_FADE_SPEED_DEG_S -
                 PITCH_STATIC_ERROR_COMP_FULL_SPEED_DEG_S);
        }
    }

    voltage = (effective_error - PITCH_STATIC_ERROR_COMP_DEADBAND_DEG) *
        PITCH_STATIC_ERROR_COMP_VOLT_PER_DEG * speed_scale;
    voltage = pitch_clamp(voltage, PITCH_STATIC_ERROR_COMP_LIMIT);
    return control_to_motor_sign * pitch_sign(error_deg) * voltage;
#else
    (void)error_deg;
    (void)speed_deg_s;
    (void)control_to_motor_sign;
    return 0.0f;
#endif
}
