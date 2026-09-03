#ifndef DM4310_H
#define DM4310_H

#include "motor_common.h"
#include <stdint.h>

#define DM4310_MOTOR_ID_MIN                  1U
#define DM4310_MOTOR_ID_MAX                  8U
#define DM4310_CURRENT_CONTROL_ID_1_TO_4     0x3FEU
#define DM4310_CURRENT_CONTROL_ID_5_TO_8     0x4FEU
#define DM4310_FEEDBACK_BASE_ID              0x300U
#define DM4310_ENCODER_COUNTS                8192U
/* The wire field is signed 16-bit, but the current-control firmware fitted to
 * this gimbal develops substantial torque from a command near 3.  Enforce the
 * same safe ceiling below the task layer as a final guard. */
#define DM4310_CURRENT_COMMAND_LIMIT             3.0f
#define DM4310_SPEED_FEEDBACK_SCALE          100.0f

typedef struct
{
    uint8_t id;
    uint16_t encoder;
    float single_turn_position_rad;
    float speed_rpm;
    float target_speed_rpm;
    float current_feedforward;
    float current_quantization_error;
    float filtered_speed_rpm;
    float speed_filter_alpha;
    int16_t torque_current_ma;
    uint8_t winding_temperature;
    uint8_t pcb_temperature;
    uint8_t speed_filter_initialized;
    uint8_t online;
    uint32_t last_update_ms;
    MotorSpeedPid_t speed_pid;
} DM4310_t;

void DM4310_Init(DM4310_t *motor, uint8_t id, float kp, float ki);
void DM4310_SetSpeed(DM4310_t *motor, float speed_rpm);
/* Added to the speed-PID result and then constrained by the PID output limit. */
void DM4310_SetCurrentFeedforward(DM4310_t *motor, float current);
void DM4310_SetSpeedFilterAlpha(DM4310_t *motor, float alpha);
void DM4310_Decode(DM4310_t *motor, const uint8_t data[8], uint32_t now_ms);
int16_t DM4310_Update(DM4310_t *motor, float dt_s);
uint8_t DM4310_PackCurrentCommand(const DM4310_t *motor, int16_t current,
                                  uint16_t *std_id, uint8_t data[8]);

#endif
