#ifndef GM6020_H
#define GM6020_H

#include "motor_common.h"

#define GM6020_VOLTAGE_LIMIT 30000.0f

typedef struct
{
    uint8_t id;
    float target_speed_rpm;
    float voltage_feedforward;
    float filtered_speed_rpm;
    float speed_filter_alpha;
    DjiMotorFeedback_t feedback;
    MotorSpeedPid_t speed_pid;
} GM6020_t;

void GM6020_Init(GM6020_t *motor, uint8_t id, float kp, float ki);
void GM6020_SetSpeed(GM6020_t *motor, float speed_rpm);
void GM6020_SetVoltageFeedforward(GM6020_t *motor, float voltage);
void GM6020_SetSpeedFilterAlpha(GM6020_t *motor, float alpha);
void GM6020_Decode(GM6020_t *motor, const uint8_t data[8], uint32_t now_ms);
int16_t GM6020_Update(GM6020_t *motor, float dt_s);

#endif
