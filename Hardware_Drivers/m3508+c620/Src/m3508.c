#include "m3508.h"
#include <string.h>

void M3508_Init(M3508_t *motor, uint8_t id, float kp, float ki)
{
    if (motor == 0) return;
    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    motor->speed_filter_alpha = 1.0f;
    MotorSpeedPid_Init(&motor->speed_pid, kp, ki, M3508_CURRENT_LIMIT,
                       M3508_CURRENT_LIMIT);
}

void M3508_SetSpeed(M3508_t *motor, float speed_rpm)
{
    if (motor != 0) motor->target_speed_rpm = speed_rpm;
}

void M3508_SetSpeedFilterAlpha(M3508_t *motor, float alpha)
{
    if (motor == 0) return;
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    motor->speed_filter_alpha = alpha;
}

void M3508_Decode(M3508_t *motor, const uint8_t data[8], uint32_t now_ms)
{
    if (motor != 0) DjiMotor_DecodeFeedback(&motor->feedback, data, now_ms);
}

int16_t M3508_Update(M3508_t *motor, float dt_s)
{
    if ((motor == 0) || (motor->feedback.online == 0U)) return 0;
    motor->filtered_speed_rpm += motor->speed_filter_alpha *
        ((float)motor->feedback.speed_rpm - motor->filtered_speed_rpm);
    return MotorSpeedPid_Calculate(&motor->speed_pid, motor->target_speed_rpm,
                                   motor->filtered_speed_rpm, dt_s);
}
