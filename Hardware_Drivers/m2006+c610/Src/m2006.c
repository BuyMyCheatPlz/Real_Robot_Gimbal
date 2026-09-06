#include "m2006.h"
#include "stm32f4xx_hal.h"
#include <string.h>

void M2006_Init(M2006_t *motor, uint8_t id, float kp, float ki)
{
    if (motor == 0) return;
    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    motor->speed_filter_alpha = 1.0f;
    MotorSpeedPid_Init(&motor->speed_pid, kp, ki, M2006_CURRENT_LIMIT,
                       M2006_CURRENT_LIMIT);
}

void M2006_SetSpeed(M2006_t *motor, float speed_rpm)
{
    if (motor == 0) return;
    motor->target_speed_rpm = speed_rpm;
    motor->last_cmd_ms = HAL_GetTick();   /* 刷新命令保活时间戳 */
}

void M2006_SetSpeedFilterAlpha(M2006_t *motor, float alpha)
{
    if (motor == 0) return;
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    motor->speed_filter_alpha = alpha;
}

void M2006_Decode(M2006_t *motor, const uint8_t data[8], uint32_t now_ms)
{
    if (motor != 0) DjiMotor_DecodeFeedback(&motor->feedback, data, now_ms);
}

int16_t M2006_Update(M2006_t *motor, float dt_s)
{
    uint32_t now;
    if (motor == 0) return 0;
    if (motor->feedback.online == 0U)
    {
        /* 与 DM4310(yaw) 一致：离线立即清积分与滤波状态，防止反馈时断时续时
         * 速度环积分累积，恢复瞬间灌出猛电流导致拨盘疯转。 */
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
    /* 命令保活安全：launch 任务每 ~2ms 调 SetSpeed。若超过 M2006_CMD_STALE_
     * TIMEOUT_MS 未刷新(任务卡死/崩溃)，立即断电，防止电机按最后目标疯转。 */
    now = HAL_GetTick();
    if ((now - motor->last_cmd_ms) > M2006_CMD_STALE_TIMEOUT_MS)
    {
        MotorSpeedPid_Reset(&motor->speed_pid);
        return 0;
    }
    /* 断电保持条件：目标转速≈0 且电机实际转速≈0。若目标已归零但电机仍在转
     * (到位死区直接断电会让电机自由滑行，实测单发过冲 0.5 发+)，必须让速度环
     * 先主动刹车(带电流)把电机真正停住，再断电保持，避免滑行过冲或目标点极限环。 */
    if ((motor->target_speed_rpm < M2006_STOP_DEADBAND_RPM) &&
        (motor->target_speed_rpm > -M2006_STOP_DEADBAND_RPM) &&
        (motor->filtered_speed_rpm < M2006_STOP_DEADBAND_RPM) &&
        (motor->filtered_speed_rpm > -M2006_STOP_DEADBAND_RPM))
    {
        MotorSpeedPid_Reset(&motor->speed_pid);
        return 0;
    }
    return MotorSpeedPid_Calculate(&motor->speed_pid, motor->target_speed_rpm,
                                   motor->filtered_speed_rpm, dt_s);
}
