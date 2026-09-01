#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

/* ---------------- 任务周期与安全保护 ---------------- */
#define CONTROL_PERIOD_S                  0.001f
#define CONTROL_PERIOD_TICKS              1U
#define DATA_PROCESS_PERIOD_MS            1U
#define SBUS_TIMEOUT_MS                   100U
#define REMOTE_COMMAND_TIMEOUT_MS         150U
#define LAUNCH_REMOTE_TIMEOUT_MS          100U
#define LAUNCH_TASK_WAIT_MS               2U
#define VOFA_PERIOD_MS                    20U
#define ONLINE_PID_VALUE_MAX              100000.0f

/* ---------------- 遥控器通道与线性映射 ---------------- */
#define SBUS_CONTROL_MIN                  240U
#define SBUS_CONTROL_MAX                  1807U
#define REMOTE_EDGE_THRESHOLD             0.55f
#define GIMBAL_COMMAND_STEP_DEG            30.0f
#define LAUNCH_M3508_TARGET_MAX_SPEED_RPM 6000.0f
#define LAUNCH_M2006_ID5_MAX_SPEED_RPM    100.0f
#define REMOTE_CH_FLYWHEEL_INDEX          3U  /* 遥控器 CH4 */
#define REMOTE_CH_FEEDER_INDEX            4U  /* 遥控器 CH5 */
#define REMOTE_CH_PITCH_POS_INDEX         5U  /* 遥控器 CH6 */
#define REMOTE_CH_PITCH_NEG_INDEX         6U  /* 遥控器 CH7 */
#define REMOTE_CH_YAW_POS_INDEX           7U  /* 遥控器 CH8 */
#define REMOTE_CH_YAW_NEG_INDEX           8U  /* 遥控器 CH9 */

/* ---------------- BMI088 安装方向与姿态滤波 ----------------
 * 轴编号对应数组下标：X=0、Y=1、Z=2。调试重力前馈前，必须根据 BMI088
 * 在实车上的安装方向修改轴映射和符号。
 */
#define IMU_ACCEL_X_AXIS                  0U
#define IMU_ACCEL_Y_AXIS                  1U
#define IMU_ACCEL_Z_AXIS                  2U
#define IMU_ACCEL_X_SIGN                  1.0f
#define IMU_ACCEL_Y_SIGN                  1.0f
#define IMU_ACCEL_Z_SIGN                  1.0f
#define IMU_GYRO_ROLL_AXIS                0U
#define IMU_GYRO_PITCH_AXIS               1U
#define IMU_GYRO_YAW_AXIS                 2U
#define IMU_GYRO_ROLL_SIGN                1.0f
#define IMU_GYRO_PITCH_SIGN               1.0f
#define IMU_GYRO_YAW_SIGN                 1.0f
#define ATTITUDE_ACCEL_WEIGHT             0.010f
#define ATTITUDE_MAX_DT_S                 0.010f

/* ---------------- 云台反馈低通滤波 ---------------- */
#define PITCH_ENCODER_LPF_ALPHA           0.15f
#define PITCH_SPEED_LPF_ALPHA             0.20f
#define YAW_ENCODER_LPF_ALPHA             0.15f
#define YAW_SPEED_LPF_ALPHA               0.20f

/* ---------------- Pitch：GM6020 外位置环 + 内速度环 ---------------- */
#define PITCH_ANGLE_KP_RPM_PER_RAD        90.0f
#define PITCH_ANGLE_KI_RPM_PER_RAD_S      0.0f
#define PITCH_ANGLE_KD_RPM_S_PER_RAD      2.0f
#define PITCH_ANGLE_INTEGRAL_LIMIT_RPM    30.0f
#define PITCH_MAX_SPEED_RPM               120.0f
#define PITCH_SPEED_KP                    80.0f
#define PITCH_SPEED_KI                    8.0f
#define PITCH_SPEED_KD                    0.0f
#define PITCH_SPEED_INTEGRAL_LIMIT        30000.0f
#define PITCH_SPEED_OUTPUT_LIMIT          30000.0f
#define PITCH_GRAVITY_FF_MAX_VOLTAGE      1200.0f
#define PITCH_GRAVITY_ZERO_RAD            0.0f
#define PITCH_GRAVITY_SIGN                1.0f
#define PITCH_MOTOR_SIGN                  1.0f
#define PITCH_SOFT_LIMIT_DEG              90.0f

/* ---------------- Yaw：DM4310 外位置环 + 电调内部速度环 ---------------- */
#define YAW_ANGLE_KP_RAD_S_PER_RAD        5.0f
#define YAW_ANGLE_KI_RAD_S_PER_RAD_S      0.0f
#define YAW_ANGLE_KD_RAD_S2_PER_RAD       0.10f
#define YAW_ANGLE_INTEGRAL_LIMIT_RAD_S    1.0f
#define YAW_MAX_SPEED_RAD_S               4.0f
#define YAW_TARGET_SLEW_RAD_S             2.0f
#define YAW_MOTOR_SIGN                    1.0f
#define YAW_SOFT_LIMIT_DEG                180.0f

/* ---------------- 发射 M3508 ID2 速度环 PID ---------------- */
#define LAUNCH_M3508_ID2_SPEED_KP         2.0f
#define LAUNCH_M3508_ID2_SPEED_KI         0.5f
#define LAUNCH_M3508_ID2_SPEED_KD         0.0f
#define LAUNCH_M3508_ID2_INTEGRAL_LIMIT   16384.0f
#define LAUNCH_M3508_ID2_OUTPUT_LIMIT     16384.0f
#define LAUNCH_M3508_ID2_SPEED_LPF_ALPHA  0.20f
#define LAUNCH_M3508_ID2_DIRECTION        1.0f

/* ---------------- 发射 M3508 ID3 速度环 PID ---------------- */
#define LAUNCH_M3508_ID3_SPEED_KP         2.0f
#define LAUNCH_M3508_ID3_SPEED_KI         0.5f
#define LAUNCH_M3508_ID3_SPEED_KD         0.0f
#define LAUNCH_M3508_ID3_INTEGRAL_LIMIT   16384.0f
#define LAUNCH_M3508_ID3_OUTPUT_LIMIT     16384.0f
#define LAUNCH_M3508_ID3_SPEED_LPF_ALPHA  0.20f
#define LAUNCH_M3508_ID3_DIRECTION       (-1.0f)

/* ---------------- 拨弹 M2006 ID5 速度环 PID ---------------- */
#define LAUNCH_M2006_ID5_SPEED_KP         8.0f
#define LAUNCH_M2006_ID5_SPEED_KI         1.0f
#define LAUNCH_M2006_ID5_SPEED_KD         0.0f
#define LAUNCH_M2006_ID5_INTEGRAL_LIMIT   10000.0f
#define LAUNCH_M2006_ID5_OUTPUT_LIMIT     10000.0f
#define LAUNCH_M2006_ID5_SPEED_LPF_ALPHA  0.20f
#define LAUNCH_M2006_ID5_DIRECTION        1.0f

#define TASK_DEG_TO_RAD                   0.017453292519943295f

#endif
