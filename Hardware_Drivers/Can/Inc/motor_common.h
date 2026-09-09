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
    /* D 项上一拍误差 e[k-1]。D = kd*(e[k]-e[k-1])（每 1 ms 一拍，不除以 dt），
     * 与模板 pid.c 语义一致，可直接使用模板整定数值。 */
    float previous_error;
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
/* 误差幅值超过此阈值时停止积分。设为 0 可保持旧行为（不按误差分离）。 */
void MotorSpeedPid_SetIntegralSeparation(MotorSpeedPid_t *pid,
                                         float error_limit);
float MotorSpeedPid_CalculateFloat(MotorSpeedPid_t *pid, float target_rpm,
                                   float measured_rpm, float dt_s);
int16_t MotorSpeedPid_Calculate(MotorSpeedPid_t *pid, float target_rpm,
                                float measured_rpm, float dt_s);
void DjiMotor_DecodeFeedback(DjiMotorFeedback_t *feedback,
                             const uint8_t data[8], uint32_t now_ms);

#endif
