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

/* 通过 VOFA 输出的诊断位掩码。远控超时只禁止新的遥控增量；已有云台目标仍可在
 * IMU、CAN 与对应电机反馈健康时继续保持。 */
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
    float roll_rate_rad_s;    /* BMI 陀螺 Roll 角速度(rad/s)，本机 Pitch 速度环反馈 */
    float pitch_rate_rad_s;   /* BMI 陀螺 Pitch 角速度(rad/s)，供姿态诊断 */
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
    PID_PARAM_YAW_KD_SPD,
    PID_PARAM_PITCH_GRAVITY_FF
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
    uint16_t feeder_switch;   /* 原始 S1 开关值：1=保持 2=单步 3=连续 */
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
    float pitch_speed_target_rpm;
    float pitch_speed_rpm;
    float pitch_motor_speed_rpm;
    float pitch_imu_speed_rpm;
    float yaw_speed_rad_s;
    float gravity_feedforward;
    float pitch_gravity_ff_setting;
    float yaw_profile_target_rad;
    float yaw_speed_target_rad_s;
    float yaw_trajectory_speed_rad_s;
    float yaw_velocity_feedforward_rad_s;
    float yaw_acceleration_feedforward_current;
    float yaw_motor_speed_rpm;
    float m3508_id2_speed_rpm;
    float m3508_id3_speed_rpm;
    float imu_roll_rad;
    float imu_pitch_rad;
    float imu_yaw_rad;
    uint32_t can_tx_failure_count;
    uint32_t can_bus_error_count;
    uint32_t can1_busoff_count;
    uint32_t can2_busoff_count;
    uint32_t can1_recovery_count;
    uint32_t can2_recovery_count;
    uint32_t can1_last_error;
    uint32_t can2_last_error;
    uint32_t dm4310_feedback_count;
    uint32_t can2_tx_complete_count;
    uint32_t can2_tx_busy_count;
    uint32_t can1_rx_count;
    uint32_t can2_rx_count;
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
    float m2006_target_deg;
    float m2006_actual_deg;
    float m2006_target_rounds;   /* 累计指令发弹数 */
    float m2006_actual_rounds;   /* 累计实际发弹数(输出旋转/40°) */
    float sysid_time_s;          /* yaw 辨识运行时间(s) */
    float sysid_command;         /* 给 DM4310 的直通电流指令 */
    float sysid_speed_rpm;       /* DM4310 原始速度反馈 rpm(不滤波) */
    uint8_t sysid_running;       /* yaw 辨识进行中 */
    uint32_t param_parse_count;  /* 在线调参成功解析计数(+1=成功) */
} GimbalControlState_t;

extern volatile GimbalControlState_t gimbal_control_state;

void Data_Process(void *argument);
void PID_calc(void *argument);
void Gimbal_YawTest_Request(int16_t current);
/* Yaw 系统辨识接口：仅在 YAW_SYSID_MODE!=0 的构建中有定义并被调用。 */
void Gimbal_YawSysid_Start(void);     /* identify_on：开始一次辨识 */
void Gimbal_YawSysid_Abort(void);     /* 强制停止(离线/发送失败等) */
uint8_t Gimbal_YawSysid_IsRunning(void);
float Gimbal_YawSysid_ElapsedSeconds(void);
float Gimbal_YawSysid_Update(uint32_t now_ms, float dt_s); /* 返回正弦电流指令 */
void VOFA_print(void *argument);
void Launch_Task(void *argument);

#endif
