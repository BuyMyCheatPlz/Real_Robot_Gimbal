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

/* Bit mask exported through VOFA.  A non-zero value explains why the gimbal
 * control task is deliberately sending zero output. */
enum
{
    GIMBAL_INHIBIT_PITCH_OFFLINE = (1U << 0),
    GIMBAL_INHIBIT_YAW_OFFLINE   = (1U << 1),
    GIMBAL_INHIBIT_IMU_STALE     = (1U << 2),
    GIMBAL_INHIBIT_REMOTE_STALE  = (1U << 3),
    GIMBAL_INHIBIT_CAN_TX_FAULT  = (1U << 4),
    GIMBAL_INHIBIT_OVERRUN       = (1U << 5),
    GIMBAL_INHIBIT_UNINITIALIZED = (1U << 6)
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
    PID_PARAM_PITCH_KP_POS = 0,
    PID_PARAM_PITCH_KI_POS,
    PID_PARAM_PITCH_KD_POS,
    PID_PARAM_PITCH_KP_SPD,
    PID_PARAM_PITCH_KI_SPD,
    PID_PARAM_PITCH_KD_SPD,
    PID_PARAM_YAW_KP_POS,
    PID_PARAM_YAW_KI_POS,
    PID_PARAM_YAW_KD_POS,
    PID_PARAM_YAW_KP_SPD,
    PID_PARAM_YAW_KI_SPD,
    PID_PARAM_YAW_KD_SPD
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
    float yaw_imu_actual_rad;
    float pitch_speed_rpm;
    float yaw_speed_rad_s;
    float gravity_feedforward;
    float yaw_trajectory_speed_rad_s;
    float yaw_velocity_feedforward_rad_s;
    float yaw_acceleration_feedforward_current;
    float yaw_motor_speed_rpm;
    float imu_roll_rad;
    float imu_pitch_rad;
    float imu_yaw_rad;
    uint32_t can_tx_failure_count;
    uint32_t dm4310_feedback_count;
    uint32_t can2_tx_complete_count;
    uint32_t can2_tx_busy_count;
    uint16_t can2_last_rx_std_id;
    uint16_t yaw_encoder_count;
    uint8_t can_last_send_failure_mask;
    uint8_t can1_tx_free_level;
    uint8_t can2_tx_free_level;
    uint8_t can1_m3508_id2_online;
    uint8_t can1_m3508_id3_online;
    uint8_t can1_gm6020_id2_online;
    uint8_t can2_m2006_id5_online;
    uint8_t can2_dm4310_id1_online;
    uint32_t control_overrun_count;
    uint32_t control_inhibit_flags;
    uint32_t pid_heartbeat;
    int16_t pitch_can_command;
    int16_t yaw_can_command;
    int16_t yaw_torque_current_ma;
    int16_t yaw_test_request_current;
    uint8_t active;
    uint8_t feedback_healthy;
    uint8_t imu_fresh;
    uint8_t can_tx_fault;
    uint8_t remote_fresh;
    uint8_t targets_initialized;
    uint8_t yaw_test_active;
    uint8_t yaw_hold_active;
} GimbalControlState_t;

extern volatile GimbalControlState_t gimbal_control_state;

void Data_Process(void *argument);
void PID_calc(void *argument);
void Gimbal_YawTest_Request(int16_t current);
void VOFA_print(void *argument);
void Launch_Task(void *argument);

#endif
