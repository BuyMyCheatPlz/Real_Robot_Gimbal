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

/* ---------------- 电机电流限幅 ---------------- */
#define M3508_CURRENT_LIMIT               16384.0f
#define M2006_CURRENT_LIMIT               10000.0f
#define DM4310_CURRENT_COMMAND_LIMIT       1000.0f

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
/* IMU Yaw 是当前的位置环测量值。进入位置 PID 前先滤波；电机编码器仍作为
 * 内层速度反馈来源。 */
#define YAW_IMU_POSITION_LPF_ALPHA         0.02f
/* 最终目标保持使用滞回，轨迹运动期间不启用。退出阈值满足不超过 0.2° 的精度要求。 */
#define YAW_HOLD_ENTER_ERROR_RAD           (0.10f * TASK_DEG_TO_RAD)
#define YAW_HOLD_EXIT_ERROR_RAD            (0.20f * TASK_DEG_TO_RAD)
#define YAW_HOLD_ENTER_SPEED_RPM            0.20f
#define YAW_PROFILE_SETTLED_POSITION_RAD   (0.01f * TASK_DEG_TO_RAD)
#define YAW_PROFILE_SETTLED_SPEED_RAD_S     0.005f
/* 在新鲜电机反馈确认输出轴静止前，保持 Yaw 断电；只有经过此窗口后才捕获保持目标。 */
#define YAW_STARTUP_SETTLE_TIME_MS          200U
#define YAW_STARTUP_MAX_FEEDBACK_AGE_MS      10U
#define YAW_STARTUP_MAX_SPEED_RPM             0.50f
#define YAW_STARTUP_MAX_POSITION_DRIFT_RAD   (0.20f * TASK_DEG_TO_RAD)
/* 0：控制授权时捕获并保持当前 Yaw（安全调试默认值）；1：自动移动到 IMU Yaw 零点。 */
/* 首次闭环调试时保持当前编码器位置。IMU Yaw 是积分量，可能包含多圈历史角度；
 * 盲目将其作为自动回零目标会导致启动命令饱和。 */
#define YAW_HOME_TO_IMU_ZERO_ON_AUTHORIZE   0U
/* 0：启用两个云台轴。发射输出单独控制，因此启用 Pitch 不会意外启动摩擦轮或拨弹机构。 */
#define YAW_COMMISSIONING_MODE                0U
#define LAUNCH_MOTOR_OUTPUT_ENABLE            0U
#define YAW_CLOSED_LOOP_ENABLE                1U
/* 电机编码器弧度除以该比例等于输出轴弧度。直驱时保持为 1.0；否则填写实际减速比。 */
#define YAW_ENCODER_TO_OUTPUT_RATIO           1.0f
#define YAW_DIRECTION_TEST_MAX_CURRENT       1000
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
/* Yaw 目标轨迹，轨迹单位为输出轴弧度。 */
#define YAW_TRAJECTORY_MAX_SPEED_RAD_S    0.08f
#define YAW_TRAJECTORY_MAX_ACCEL_RAD_S2   0.20f
/* 0：关闭速度前馈；1：直接使用规划速度。 */
#define YAW_VELOCITY_FF_GAIN              0.60f
/* 可直接调节的力矩前馈：单位为 CAN 电流命令单位/输出轴 rad/s²。
 * PID 环稳定前应保持较小。正值表示输出轴正加速度，控制器会应用
 * YAW_MOTOR_COMMAND_SIGN。 */
#define YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2 0.50f
#define YAW_ACCELERATION_FF_CURRENT_LIMIT  1000.0f
#define YAW_SPEED_KP_CURRENT_PER_RPM        1.0f
#define YAW_SPEED_KI_CURRENT_PER_RPM_S      0.5f
#define YAW_SPEED_KD_CURRENT_S_PER_RPM      0.0f
#define YAW_SPEED_INTEGRAL_LIMIT_CURRENT  1000.0f
/* DM4310 使用小端命令格式；1000 发送为 E8 03，并按 1000 接收，而不是
 * 字节交换后得到的错误值。 */
#define YAW_CURRENT_OUTPUT_LIMIT          1000.0f
#define YAW_SPEED_INTEGRAL_SEPARATION_RPM  5.0f
#define YAW_ANGLE_INTEGRAL_SEPARATION_RAD (20.0f * TASK_DEG_TO_RAD)
/* 台架测试表明：正电流会增加编码器计数、原始速度、编码器角度和 IMU Yaw，
 * 负电流会降低这四项。因此本安装方式下命令符号和编码器符号均为正。 */
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
