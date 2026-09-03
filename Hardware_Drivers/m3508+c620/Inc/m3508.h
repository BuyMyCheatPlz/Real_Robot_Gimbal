#ifndef M3508_H
#define M3508_H

#include "motor_common.h"
#include "config.h"

typedef struct
{
    uint8_t id;
    float target_speed_rpm;
    float filtered_speed_rpm;
    float speed_filter_alpha;
    DjiMotorFeedback_t feedback;
    MotorSpeedPid_t speed_pid;
} M3508_t;

void M3508_Init(M3508_t *motor, uint8_t id, float kp, float ki);
void M3508_SetSpeed(M3508_t *motor, float speed_rpm);
void M3508_SetSpeedFilterAlpha(M3508_t *motor, float alpha);
void M3508_Decode(M3508_t *motor, const uint8_t data[8], uint32_t now_ms);
int16_t M3508_Update(M3508_t *motor, float dt_s);

#endif
