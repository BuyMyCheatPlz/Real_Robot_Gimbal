#include "motor_common.h"

static float clampf(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void MotorSpeedPid_Init(MotorSpeedPid_t *pid, float kp, float ki,
                        float integral_limit, float output_limit)
{
    if (pid == 0) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = 0.0f;
    pid->integral = 0.0f;
    pid->integral_limit = integral_limit;
    pid->integral_separation_error = 0.0f;
    pid->output_limit = output_limit;
    pid->previous_measurement = 0.0f;
    pid->initialized = 0U;
}

void MotorSpeedPid_Reset(MotorSpeedPid_t *pid)
{
    if (pid != 0)
    {
        pid->integral = 0.0f;
        pid->initialized = 0U;
    }
}

void MotorSpeedPid_SetGains(MotorSpeedPid_t *pid, float kp, float ki,
                            float kd)
{
    if (pid == 0) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void MotorSpeedPid_SetIntegralSeparation(MotorSpeedPid_t *pid,
                                         float error_limit)
{
    if (pid == 0) return;
    pid->integral_separation_error = (error_limit > 0.0f) ?
        error_limit : 0.0f;
}

float MotorSpeedPid_CalculateFloat(MotorSpeedPid_t *pid, float target_rpm,
                                   float measured_rpm, float dt_s)
{
    float error;
    float output;
    float derivative = 0.0f;
    float proportional_derivative;
    float candidate_integral;
    uint8_t integrate;
    if ((pid == 0) || (dt_s <= 0.0f)) return 0.0f;

    error = target_rpm - measured_rpm;
    if (pid->initialized != 0U)
        derivative = -(measured_rpm - pid->previous_measurement) / dt_s;
    else
        pid->initialized = 1U;
    pid->previous_measurement = measured_rpm;
    proportional_derivative = error * pid->kp + derivative * pid->kd;
    integrate = (uint8_t)((pid->integral_separation_error <= 0.0f) ||
                          (error <= pid->integral_separation_error &&
                           error >= -pid->integral_separation_error));
    if (integrate != 0U)
    {
        candidate_integral = clampf(pid->integral + error * pid->ki * dt_s,
                                    pid->integral_limit);
        output = proportional_derivative + candidate_integral;
        /* 执行器已经饱和时，不要继续向饱和方向积分；允许反向误差释放累加量。 */
        if (((output < pid->output_limit) && (output > -pid->output_limit)) ||
            ((output >= pid->output_limit) && (error < 0.0f)) ||
            ((output <= -pid->output_limit) && (error > 0.0f)))
            pid->integral = candidate_integral;
    }
    output = proportional_derivative + pid->integral;
    output = clampf(output, pid->output_limit);
    return output;
}

int16_t MotorSpeedPid_Calculate(MotorSpeedPid_t *pid, float target_rpm,
                                float measured_rpm, float dt_s)
{
    return (int16_t)MotorSpeedPid_CalculateFloat(pid, target_rpm,
                                                  measured_rpm, dt_s);
}

void DjiMotor_DecodeFeedback(DjiMotorFeedback_t *feedback,
                             const uint8_t data[8], uint32_t now_ms)
{
    if ((feedback == 0) || (data == 0)) return;
    feedback->encoder = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    feedback->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    feedback->current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    feedback->temperature = data[6];
    feedback->last_update_ms = now_ms;
    feedback->online = 1U;
}
