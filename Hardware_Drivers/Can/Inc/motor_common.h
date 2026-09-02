#ifndef MOTOR_COMMON_H
#define MOTOR_COMMON_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float integral_separation_error;
    float output_limit;
    float previous_measurement;
    uint8_t initialized;
} MotorSpeedPid_t;

typedef struct
{
    uint16_t encoder;
    int16_t speed_rpm;
    int16_t current;
    uint8_t temperature;
    uint32_t last_update_ms;
    uint8_t online;
} DjiMotorFeedback_t;

void MotorSpeedPid_Init(MotorSpeedPid_t *pid, float kp, float ki,
                        float integral_limit, float output_limit);
void MotorSpeedPid_Reset(MotorSpeedPid_t *pid);
void MotorSpeedPid_SetGains(MotorSpeedPid_t *pid, float kp, float ki,
                            float kd);
/* Error magnitude above this threshold disables integration.  A value of zero
 * keeps the legacy behaviour (no error-based separation). */
void MotorSpeedPid_SetIntegralSeparation(MotorSpeedPid_t *pid,
                                         float error_limit);
int16_t MotorSpeedPid_Calculate(MotorSpeedPid_t *pid, float target_rpm,
                                float measured_rpm, float dt_s);
void DjiMotor_DecodeFeedback(DjiMotorFeedback_t *feedback,
                             const uint8_t data[8], uint32_t now_ms);

#endif
