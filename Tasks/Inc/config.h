#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

/* ---------------- 任务周期与安全保护 ---------------- */
#define CONTROL_PERIOD_S                  0.001f
#define CONTROL_PERIOD_TICKS              1U
#define DATA_PROCESS_PERIOD_MS            1U
#define DBUS_TIMEOUT_MS                   100U
#define REMOTE_COMMAND_TIMEOUT_MS         150U
#define IMU_DATA_TIMEOUT_MS                20U
#define CONTROL_MAX_DT_S                   0.010f
#define LAUNCH_REMOTE_TIMEOUT_MS          100U
#define LAUNCH_TASK_WAIT_MS               2U
#define VOFA_PERIOD_MS                    10U
#define ONLINE_PID_VALUE_MAX              100000.0f

/* ---------------- 遥控器通道与线性映射 ---------------- */
#define DBUS_CENTER_CHANNEL               1024U
#define DBUS_DEADZONE                     60U
#define REMOTE_EDGE_THRESHOLD             0.55f
#define GIMBAL_COMMAND_STEP_DEG            30.0f
#define LAUNCH_M3508_TARGET_MAX_SPEED_RPM 6000.0f
#define LAUNCH_M2006_ID5_MAX_SPEED_RPM    100.0f
#define REMOTE_CH_FLYWHEEL_INDEX          4U  /* D-BUS S1 */
#define REMOTE_CH_FEEDER_INDEX            5U  /* D-BUS S2 */
#define REMOTE_CH_PITCH_INDEX             3U  /* 遥控器 CH4 */
#define REMOTE_CH_YAW_INDEX               0U  /* 遥控器 CH1 */
#define REMOTE_CH_S1_INDEX                4U  /* D-BUS S1 */
#define REMOTE_CH_S2_INDEX                5U  /* D-BUS S2 */

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
/* IMU Yaw is the current outer-loop position measurement.  Filter it before
 * it reaches the position PID; the motor encoder remains the inner speed
 * feedback source. */
#define YAW_IMU_POSITION_LPF_ALPHA         0.02f
/* Final-target hold uses hysteresis and is disabled while the trajectory is
 * moving.  The exit threshold matches the required <=0.2 degree accuracy. */
#define YAW_HOLD_ENTER_ERROR_RAD           (0.10f * TASK_DEG_TO_RAD)
#define YAW_HOLD_EXIT_ERROR_RAD            (0.20f * TASK_DEG_TO_RAD)
#define YAW_HOLD_ENTER_SPEED_RPM            0.20f
#define YAW_PROFILE_SETTLED_POSITION_RAD   (0.01f * TASK_DEG_TO_RAD)
#define YAW_PROFILE_SETTLED_SPEED_RAD_S     0.005f
/* Keep yaw de-energized until fresh motor feedback proves that the output
 * axis is stationary.  Capture the hold target only after this window. */
#define YAW_STARTUP_SETTLE_TIME_MS          200U
#define YAW_STARTUP_MAX_FEEDBACK_AGE_MS      10U
#define YAW_STARTUP_MAX_SPEED_RPM             0.50f
#define YAW_STARTUP_MAX_POSITION_DRIFT_RAD   (0.20f * TASK_DEG_TO_RAD)
/* 0: capture and hold the current yaw when control is authorized (safe
 * commissioning default).  1: automatically move to IMU yaw zero. */
/* Keep the present encoder position during first closed-loop commissioning.
 * IMU yaw is integrated and can contain several historical turns; using it
 * blindly as an automatic home target causes a saturated startup command. */
#define YAW_HOME_TO_IMU_ZERO_ON_AUTHORIZE   0U
/* 0 enables the two gimbal axes.  Launch outputs are controlled separately
 * so enabling Pitch cannot unexpectedly start the flywheel or feeder. */
#define YAW_COMMISSIONING_MODE                0U
#define LAUNCH_MOTOR_OUTPUT_ENABLE            0U
#define YAW_CLOSED_LOOP_ENABLE                1U
/* Motor encoder radians divided by this ratio equals output-axis radians.
 * Keep 1.0 only for direct drive; set the actual reduction ratio otherwise. */
#define YAW_ENCODER_TO_OUTPUT_RATIO           1.0f
#define YAW_DIRECTION_TEST_MAX_CURRENT          3
#define YAW_DIRECTION_TEST_DURATION_MS         200U

/* ---------------- Pitch：GM6020 外位置环 + 内速度环 ---------------- */
#define PITCH_ANGLE_KP_RPM_PER_RAD        90.0f
#define PITCH_ANGLE_KI_RPM_PER_RAD_S      0.0f
#define PITCH_ANGLE_KD_RPM_S_PER_RAD      2.0f
#define PITCH_ANGLE_INTEGRAL_LIMIT_RPM    30.0f
#define PITCH_MAX_SPEED_RPM               120.0f
#define PITCH_SPEED_KP                    80.0f
#define PITCH_SPEED_KI                     8.0f
#define PITCH_SPEED_KD                    0.0f
#define PITCH_SPEED_INTEGRAL_LIMIT        30000.0f
#define PITCH_SPEED_OUTPUT_LIMIT          10000.0f
#define PITCH_SPEED_INTEGRAL_SEPARATION_RPM 80.0f
#define PITCH_ANGLE_INTEGRAL_SEPARATION_RAD (10.0f * TASK_DEG_TO_RAD)
#define PITCH_GRAVITY_FF_MAX_VOLTAGE         0.0f
#define PITCH_GRAVITY_ZERO_RAD            0.0f
#define PITCH_GRAVITY_SIGN                1.0f
#define PITCH_MOTOR_SIGN                  1.0f
#define PITCH_SOFT_LIMIT_DEG              90.0f

/* ---------------- Yaw：DM4310 外位置环 + 软件速度环 ---------------- */
#define YAW_ANGLE_KP_RAD_S_PER_RAD        0.35f
#define YAW_ANGLE_KI_RAD_S_PER_RAD_S      0.02f
#define YAW_ANGLE_KD_RAD_S2_PER_RAD       0.0f
#define YAW_ANGLE_INTEGRAL_LIMIT_RAD_S    0.04f
#define YAW_MAX_SPEED_RAD_S               0.10f
/* Yaw reference trajectory.  The profile is output-axis radians. */
#define YAW_TRAJECTORY_MAX_SPEED_RAD_S    0.08f
#define YAW_TRAJECTORY_MAX_ACCEL_RAD_S2   0.20f
/* 0 disables velocity feedforward; 1 applies the planned speed directly. */
#define YAW_VELOCITY_FF_GAIN              0.60f
/* Directly tunable torque feedforward: CAN current-command units per
 * output-axis rad/s^2.  Keep it small until the PID loops are stable.  Positive
 * means positive output-axis acceleration; YAW_MOTOR_COMMAND_SIGN is applied by the
 * controller. */
#define YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2 0.50f
#define YAW_ACCELERATION_FF_CURRENT_LIMIT    0.20f
#define YAW_SPEED_KP_CURRENT_PER_RPM        1.0f
#define YAW_SPEED_KI_CURRENT_PER_RPM_S      0.5f
#define YAW_SPEED_KD_CURRENT_S_PER_RPM      0.0f
#define YAW_SPEED_INTEGRAL_LIMIT_CURRENT    1.0f
/* Captured hardware data showed that a command magnitude of 20 drove the
 * output to about 71 rpm and excited a large reversing oscillation.  Keep
 * closed-loop and direction-test commands at the proven low-torque limit. */
#define YAW_CURRENT_OUTPUT_LIMIT             3.0f
#define YAW_SPEED_INTEGRAL_SEPARATION_RPM  5.0f
#define YAW_ANGLE_INTEGRAL_SEPARATION_RAD (20.0f * TASK_DEG_TO_RAD)
/* Bench tests: +current increases encoder count, raw speed, encoder angle,
 * and IMU yaw; -current decreases all four.  Command and encoder signs are
 * therefore both positive on this installation. */
#define YAW_MOTOR_COMMAND_SIGN             1.0f
#define YAW_ENCODER_SIGN                   1.0f
#define YAW_SOFT_LIMIT_DEG                180.0f

/* ---------------- 发射 M3508 ID2 速度环 PID ---------------- */
#define LAUNCH_M3508_ID2_SPEED_KP         2.0f
#define LAUNCH_M3508_ID2_SPEED_KI         0.5f
#define LAUNCH_M3508_ID2_SPEED_KD         0.0f
#define LAUNCH_M3508_ID2_INTEGRAL_LIMIT   16384.0f
#define LAUNCH_M3508_ID2_OUTPUT_LIMIT     16384.0f
#define LAUNCH_M3508_ID2_INTEGRAL_SEPARATION_RPM 1000.0f
#define LAUNCH_M3508_ID2_SPEED_LPF_ALPHA  0.20f
#define LAUNCH_M3508_ID2_DIRECTION        1.0f

/* ---------------- 发射 M3508 ID3 速度环 PID ---------------- */
#define LAUNCH_M3508_ID3_SPEED_KP         2.0f
#define LAUNCH_M3508_ID3_SPEED_KI         0.5f
#define LAUNCH_M3508_ID3_SPEED_KD         0.0f
#define LAUNCH_M3508_ID3_INTEGRAL_LIMIT   16384.0f
#define LAUNCH_M3508_ID3_OUTPUT_LIMIT     16384.0f
#define LAUNCH_M3508_ID3_INTEGRAL_SEPARATION_RPM 1000.0f
#define LAUNCH_M3508_ID3_SPEED_LPF_ALPHA  0.20f
#define LAUNCH_M3508_ID3_DIRECTION       (-1.0f)

/* ---------------- 拨弹 M2006 ID5 速度环 PID ---------------- */
#define LAUNCH_M2006_ID5_SPEED_KP         8.0f
#define LAUNCH_M2006_ID5_SPEED_KI         1.0f
#define LAUNCH_M2006_ID5_SPEED_KD         0.0f
#define LAUNCH_M2006_ID5_INTEGRAL_LIMIT   10000.0f
#define LAUNCH_M2006_ID5_OUTPUT_LIMIT     10000.0f
#define LAUNCH_M2006_ID5_INTEGRAL_SEPARATION_RPM 30.0f
#define LAUNCH_M2006_ID5_SPEED_LPF_ALPHA  0.20f
#define LAUNCH_M2006_ID5_DIRECTION        1.0f

#define TASK_DEG_TO_RAD                   0.017453292519943295f

#endif
