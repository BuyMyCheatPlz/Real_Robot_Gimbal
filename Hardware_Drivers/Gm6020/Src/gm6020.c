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
    float feedback_output;
    float speed_feedback_rpm;
    if (motor == 0) return 0;
    if (motor->feedback.online == 0U)
    {
        /* 与 DM4310(yaw) 一致：离线立即清积分与滤波状态，防止反馈时断时续时
         * 速度环积分累积，恢复瞬间灌出猛电压导致 pitch 疯转。 */
        MotorSpeedPid_Reset(&motor->speed_pid);
        motor->speed_filter_initialized = 0U;
        motor->output_hold = 0U;   /* 离线解除保持，防止旧输出复活 */
        motor->last_feedback_output = 0.0f;
        motor->last_output = 0.0f;
        return 0;
    }
    if (motor->output_hold != 0U)
    {
        /* yaw 运动期间锁存 pitch：输出沿用进入保持时的值，PID/滤波状态冻结，
         * 避免 yaw 转动漏进 Roll 通道让 pitch 速度环误动。 */
        return motor->held_output;
    }
    /* 速度环反馈源：默认用编码器转速；若启用外部反馈(如 BMI 陀螺 Roll
     * 角速度)，则目标与反馈必须使用同一单位。陀螺直接测云台真实角速度，
     * 不受减速/背隙/柔性影响。 */
    speed_feedback_rpm = (motor->use_external_speed_feedback != 0U) ?
                         motor->external_speed_rpm :
                         (float)motor->feedback.speed_rpm;
    if (motor->speed_filter_initialized == 0U)
    {
        motor->filtered_speed_rpm = speed_feedback_rpm;
        motor->speed_filter_initialized = 1U;
    }
    else
    {
        motor->filtered_speed_rpm += motor->speed_filter_alpha *
            (speed_feedback_rpm - motor->filtered_speed_rpm);
    }
    /* 前馈会占用一部分执行器余量。把剩余的非对称上下限传给 PID，
     * 使积分器看到真正的最终饱和边界，而不是在 PID+前馈被二次裁剪时饱和累积。 */
    feedback_output = MotorSpeedPid_CalculateFloatBounded(
        &motor->speed_pid, motor->target_speed_rpm,
        motor->filtered_speed_rpm, dt_s,
        -GM6020_VOLTAGE_LIMIT - motor->voltage_feedforward,
         GM6020_VOLTAGE_LIMIT - motor->voltage_feedforward);
    output = feedback_output + motor->voltage_feedforward;
    if (output > GM6020_VOLTAGE_LIMIT) output = GM6020_VOLTAGE_LIMIT;
    if (output < -GM6020_VOLTAGE_LIMIT) output = -GM6020_VOLTAGE_LIMIT;
    motor->last_feedback_output = feedback_output;
    motor->last_output = output;
    return (int16_t)output;
}
