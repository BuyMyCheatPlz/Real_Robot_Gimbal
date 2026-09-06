#include "gm6020.h"
#include "config.h"
#include <math.h>
#include <string.h>

void GM6020_Init(GM6020_t *motor, uint8_t id, float kp, float ki)
{
    if (motor == 0) return;
    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    motor->speed_filter_alpha = 1.0f;
    MotorSpeedPid_Init(&motor->speed_pid, kp, ki, GM6020_VOLTAGE_LIMIT,
                       GM6020_VOLTAGE_LIMIT);
}

void GM6020_SetSpeed(GM6020_t *motor, float speed_rpm)
{
    if (motor != 0) motor->target_speed_rpm = speed_rpm;
}

void GM6020_SetVoltageFeedforward(GM6020_t *motor, float voltage)
{
    if (motor == 0) return;
    if (voltage > GM6020_VOLTAGE_LIMIT) voltage = GM6020_VOLTAGE_LIMIT;
    if (voltage < -GM6020_VOLTAGE_LIMIT) voltage = -GM6020_VOLTAGE_LIMIT;
    motor->voltage_feedforward = voltage;
}

void GM6020_SetSpeedFilterAlpha(GM6020_t *motor, float alpha)
{
    if (motor == 0) return;
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    motor->speed_filter_alpha = alpha;
}

void GM6020_Decode(GM6020_t *motor, const uint8_t data[8], uint32_t now_ms)
{
    if (motor != 0) DjiMotor_DecodeFeedback(&motor->feedback, data, now_ms);
}

int16_t GM6020_Update(GM6020_t *motor, float dt_s)
{
    float output;
    if (motor == 0) return 0;
    if (motor->feedback.online == 0U)
    {
        /* 与 DM4310(yaw) 一致：离线立即清积分与滤波状态，防止反馈时断时续时
         * 速度环积分累积，恢复瞬间灌出猛电压导致 pitch 疯转。 */
        MotorSpeedPid_Reset(&motor->speed_pid);
        motor->speed_filter_initialized = 0U;
        return 0;
    }
    if (motor->speed_filter_initialized == 0U)
    {
        motor->filtered_speed_rpm = (float)motor->feedback.speed_rpm;
        motor->speed_filter_initialized = 1U;
    }
    else
    {
        motor->filtered_speed_rpm += motor->speed_filter_alpha *
            ((float)motor->feedback.speed_rpm - motor->filtered_speed_rpm);
    }
    output = (float)MotorSpeedPid_Calculate(&motor->speed_pid,
                                            motor->target_speed_rpm,
                                            motor->filtered_speed_rpm, dt_s) +
             motor->voltage_feedforward;
    if (output > GM6020_VOLTAGE_LIMIT) output = GM6020_VOLTAGE_LIMIT;
    if (output < -GM6020_VOLTAGE_LIMIT) output = -GM6020_VOLTAGE_LIMIT;
    if (fabsf(motor->target_speed_rpm) >= PITCH_STARTUP_SPEED_THRESHOLD_RPM &&
        fabsf(output) < PITCH_STARTUP_MIN_VOLTAGE)
    {
        output = (motor->target_speed_rpm > 0.0f) ?
            PITCH_STARTUP_MIN_VOLTAGE : -PITCH_STARTUP_MIN_VOLTAGE;
    }
    return (int16_t)output;
}
