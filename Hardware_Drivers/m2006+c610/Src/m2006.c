#include "m2006.h"
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
    if (motor != 0) motor->target_speed_rpm = speed_rpm;
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
    if ((motor == 0) || (motor->feedback.online == 0U)) return 0;
    motor->filtered_speed_rpm += motor->speed_filter_alpha *
        ((float)motor->feedback.speed_rpm - motor->filtered_speed_rpm);
    /* 目标转速接近零(由角度环到位死区或保持指令触发)时直接断电，不再经过
     * 速度环。否则重载 + 减速器背隙下 P 项对振荡的速度反馈持续输出大电流，
     * 形成目标点附近的极限环(实测 ~5Hz ±4° 抖动)。断电后靠负载阻力自然停车。 */
    if ((motor->target_speed_rpm < M2006_STOP_DEADBAND_RPM) &&
        (motor->target_speed_rpm > -M2006_STOP_DEADBAND_RPM))
    {
        MotorSpeedPid_Reset(&motor->speed_pid);
        return 0;
    }
    return MotorSpeedPid_Calculate(&motor->speed_pid, motor->target_speed_rpm,
                                   motor->filtered_speed_rpm, dt_s);
}
