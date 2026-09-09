#ifndef GM6020_H
#define GM6020_H

#include "motor_common.h"

/* 力矩/电压输出限幅：已按需求放开（原 25000 → 30000，驱动与 GM6020 指令上限）。 */
#define GM6020_VOLTAGE_LIMIT 30000.0f

typedef struct
{
    uint8_t id;
    float target_speed_rpm;
    float voltage_feedforward;
    float filtered_speed_rpm;
    float speed_filter_alpha;
    uint8_t speed_filter_initialized;
    uint8_t use_external_speed_feedback;  /* 1=速度环反馈用外部(如 BMI 陀螺)，而非编码器 */
    /* 名称为兼容旧接口保留。外部反馈可使用任意与 target_speed_rpm 一致的
     * 速度单位；本项目 Pitch 使用 BMI088 Roll 的 °/s。 */
    float external_speed_rpm;
    /* 输出保持（yaw 运动期间锁存 pitch 用）：
     * output_hold=1 时 GM6020_Update 不再更新 PID/滤波，直接返回 held_output，
     * 避免 yaw 转动漏进 Roll 通道导致 pitch 速度环误动。 */
    uint8_t output_hold;
    int16_t held_output;
    float last_feedback_output; /* 速度 PID 输出，不含前馈，供整定观测 */
    float last_output;    /* 最近一次正常(非保持)输出，用于进入保持时锁存 */
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
