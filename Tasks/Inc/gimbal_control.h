#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <stdint.h>

enum
{
    GIMBAL_MSG_ATTITUDE    = (1U << 0),
    GIMBAL_MSG_PITCH_DELTA = (1U << 1),
    GIMBAL_MSG_YAW_DELTA   = (1U << 2),
    GIMBAL_MSG_REMOTE_OK   = (1U << 3),
    GIMBAL_MSG_REMOTE_BAD  = (1U << 4)
};

/* Target_Angle 队列消息；角度增量只允许 PID_calc 消费一次。 */
typedef struct
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float pitch_delta_rad;
    float yaw_delta_rad;
    uint32_t timestamp_ms;
    uint32_t flags;
} TargetAngleMessage_t;

typedef enum
{
    PID_PARAM_KP_POS = 0,
    PID_PARAM_KI_POS,
    PID_PARAM_KD_POS,
    PID_PARAM_KP_SPD,
    PID_PARAM_KI_SPD,
    PID_PARAM_KD_SPD
} PidParameterId_t;

typedef struct
{
    PidParameterId_t id;
    float value;
} PidParameterUpdate_t;

enum
{
    LAUNCH_FLYWHEEL_READY = (1U << 0),
    LAUNCH_FEEDER_READY   = (1U << 1)
};

typedef struct
{
    float flywheel_speed_rpm;
    float feeder_speed_rpm;
    uint32_t timestamp_ms;
    uint32_t flags;
} LaunchParameterUpdate_t;

typedef struct
{
    float pitch_target_rad;
    float yaw_target_rad;
    float pitch_encoder_rad;
    float yaw_encoder_rad;
    float pitch_speed_rpm;
    float yaw_speed_rad_s;
    float gravity_feedforward;
    float yaw_trajectory_speed_rad_s;
    float yaw_velocity_feedforward_rad_s;
    float yaw_acceleration_feedforward_current;
    uint32_t can_tx_failure_count;
    uint32_t control_overrun_count;
    uint8_t active;
    uint8_t feedback_healthy;
    uint8_t imu_fresh;
    uint8_t can_tx_fault;
} GimbalControlState_t;

extern volatile GimbalControlState_t gimbal_control_state;

void Data_Process(void *argument);
void PID_calc(void *argument);
void VOFA_print(void *argument);
void Launch_Task(void *argument);

#endif
