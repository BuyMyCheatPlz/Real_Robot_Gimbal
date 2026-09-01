#ifndef M2006_H
#define M2006_H

#include "motor_common.h"

#define M2006_CURRENT_LIMIT 10000.0f

typedef struct
{
    uint8_t id;
    float target_speed_rpm;
    float filtered_speed_rpm;
    float speed_filter_alpha;
    DjiMotorFeedback_t feedback;
    MotorSpeedPid_t speed_pid;
} M2006_t;

void M2006_Init(M2006_t *motor, uint8_t id, float kp, float ki);
void M2006_SetSpeed(M2006_t *motor, float speed_rpm);
void M2006_SetSpeedFilterAlpha(M2006_t *motor, float alpha);
void M2006_Decode(M2006_t *motor, const uint8_t data[8], uint32_t now_ms);
int16_t M2006_Update(M2006_t *motor, float dt_s);

#endif
