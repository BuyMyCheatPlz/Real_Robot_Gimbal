#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

/* ---------------- 任务周期与安全保护 ---------------- */
#define CONTROL_PERIOD_S                  0.004f
#define CONTROL_PERIOD_TICKS              4U
#define CAN_COMMAND_PERIOD_MS             10U
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
#define DM4310_CURRENT_COMMAND_LIMIT       16384.0f

/* ---------------- 遥控器通道与线性映射 ---------------- */
#define DBUS_CENTER_CHANNEL               1024U
#define DBUS_DEADZONE                     60U
#define REMOTE_EDGE_THRESHOLD             0.55f
#define GIMBAL_COMMAND_STEP_DEG            30.0f
#define LAUNCH_M3508_TARGET_MAX_SPEED_RPM 6000.0f
#define LAUNCH_M2006_ID5_MAX_SPEED_RPM    100.0f
#define REMOTE_CH_FLYWHEEL_INDEX          5U  /* D-BUS S2 */
#define REMOTE_CH_FEEDER_INDEX            4U  /* D-BUS S1 */
#define REMOTE_CH_PITCH_INDEX             3U  /* 遥控器 CH4 */
#define REMOTE_CH_YAW_INDEX               0U  /* 遥控器 CH1 */
#define REMOTE_CH_S1_INDEX                4U  /* D-BUS S1：M2006 */
#define REMOTE_CH_S2_INDEX                5U  /* D-BUS S2：M3508 */

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
/* 在新鲜电机反馈确认输出轴静止前，保持 Yaw 断电；只有经过此窗口后才捕获保持目标。
 * 反馈新鲜度允许至 50 ms，兼容低于控制任务频率的反馈流；100 ms 离线保护仍在
 * CAN 驱动层执行。 */
#define YAW_STARTUP_SETTLE_TIME_MS          200U
#define YAW_STARTUP_MAX_FEEDBACK_AGE_MS      50U
#define YAW_STARTUP_MAX_SPEED_RPM             0.50f
#define YAW_STARTUP_MAX_POSITION_DRIFT_RAD   (0.20f * TASK_DEG_TO_RAD)
/* 0：控制授权时捕获并保持当前 Yaw；1：自动移动到 BMI088 Yaw 零点。 */
#define YAW_HOME_TO_IMU_ZERO_ON_AUTHORIZE   1U
/* 0：启用两个云台轴。发射输出单独控制，因此启用 Pitch 不会意外启动摩擦轮或拨弹机构。 */
#define YAW_COMMISSIONING_MODE                0U
#define LAUNCH_MOTOR_OUTPUT_ENABLE            1U
#define YAW_CLOSED_LOOP_ENABLE                1U
/* 电机编码器弧度除以该比例等于输出轴弧度。直驱时保持为 1.0；否则填写实际减速比。 */
#define YAW_ENCODER_TO_OUTPUT_RATIO           1.0f
#define YAW_DIRECTION_TEST_MAX_CURRENT       1000
#define YAW_DIRECTION_TEST_DURATION_MS         200U

/* ---------------- Pitch：GM6020 外位置环 + 内速度环 ---------------- */
#define PITCH_GM6020_CAN_ID                2U
#define PITCH_GRAVITY_ONLY_ENABLE          0U
#define PITCH_ANGLE_KP_RPM_PER_RAD        75.0f
#define PITCH_ANGLE_KI_RPM_PER_RAD_S      0.0f
#define PITCH_ANGLE_KD_RPM_S_PER_RAD      1.0f
#define PITCH_ANGLE_INTEGRAL_LIMIT_RPM    30.0f
#define PITCH_MAX_SPEED_RPM                90.0f
#define PITCH_SPEED_KP                    70.0f
#define PITCH_SPEED_KI                     4.0f
#define PITCH_SPEED_KD                    0.0f
#define PITCH_SPEED_INTEGRAL_LIMIT         12000.0f
#define PITCH_SPEED_OUTPUT_LIMIT           10000.0f
#define PITCH_SPEED_INTEGRAL_SEPARATION_RPM 70.0f
#define PITCH_STARTUP_SPEED_THRESHOLD_RPM   1.0f
#define PITCH_STARTUP_MIN_VOLTAGE           8000.0f
#define PITCH_ANGLE_INTEGRAL_SEPARATION_RAD (10.0f * TASK_DEG_TO_RAD)
/* Positive feedforward counteracts gravity in the measured installation.
 * If bench testing shows it increases the downward pull, flip only this sign. */
#define PITCH_GRAVITY_FF_MAX_VOLTAGE         7000.0f
#define PITCH_GRAVITY_ZERO_RAD            0.0f
#define PITCH_GRAVITY_ANGLE_MIN_DEG      (-24.34f)
#define PITCH_GRAVITY_ANGLE_MAX_DEG       (49.58f)
#define PITCH_GRAVITY_SIGN               -1.0f
#define PITCH_MOTOR_SIGN                  1.0f
#define PITCH_SOFT_LIMIT_DEG              90.0f

/* ---------------- Yaw：DM4310 外位置环 + 软件速度环 ---------------- */
#define YAW_ANGLE_KP_RAD_S_PER_RAD        1.20f
#define YAW_ANGLE_KI_RAD_S_PER_RAD_S      0.02f
#define YAW_ANGLE_KD_RAD_S2_PER_RAD       0.0f
#define YAW_ANGLE_INTEGRAL_LIMIT_RAD_S    0.04f
#define YAW_MAX_SPEED_RAD_S               1.50f
/* Yaw 目标轨迹，轨迹单位为输出轴弧度。 */
#define YAW_TRAJECTORY_MAX_SPEED_RAD_S    1.00f
#define YAW_TRAJECTORY_MAX_ACCEL_RAD_S2   3.00f
/* 0：关闭速度前馈；1：直接使用规划速度。 */
#define YAW_VELOCITY_FF_GAIN              0.90f
/* 可直接调节的力矩前馈：单位为 CAN 电流命令单位/输出轴 rad/s²。
 * PID 环稳定前应保持较小。正值表示输出轴正加速度，控制器会应用
 * YAW_MOTOR_COMMAND_SIGN。 */
#define YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2 20.0f
#define YAW_ACCELERATION_FF_CURRENT_LIMIT  16384.0f
#define YAW_SPEED_KP_CURRENT_PER_RPM       80.0f
#define YAW_SPEED_KI_CURRENT_PER_RPM_S      5.0f
#define YAW_SPEED_KD_CURRENT_S_PER_RPM      0.0f
#define YAW_SPEED_INTEGRAL_LIMIT_CURRENT  16384.0f
/* 克服 DM4310 与机构静摩擦的最小启动电流；目标速度为零时不生效。 */
#define YAW_STARTUP_SPEED_THRESHOLD_RPM    0.50f
#define YAW_STARTUP_MIN_CURRENT             500.0f
/* DM4310 使用小端命令格式；1000 发送为 E8 03，并按 1000 接收，而不是
 * 字节交换后得到的错误值。 */
#define YAW_CURRENT_OUTPUT_LIMIT          16384.0f
#define YAW_SPEED_INTEGRAL_SEPARATION_RPM  5.0f
#define YAW_ANGLE_INTEGRAL_SEPARATION_RAD (20.0f * TASK_DEG_TO_RAD)
/* 台架测试表明：正电流会增加编码器计数、原始速度、编码器角度和 IMU Yaw，
 * 负电流会降低这四项。因此本安装方式下命令符号和编码器符号均为正。 */
#define YAW_MOTOR_COMMAND_SIGN             1.0f
#define YAW_ENCODER_SIGN                   1.0f
/* 0：Yaw 使用多圈编码器连续角度，不限制累计目标角。 */
#define YAW_SOFT_LIMIT_DEG                  0.0f

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

/* ---------------- 拨弹 M2006 ID5 速度环 PID ----------------
 * 电机轴转速环。KP=10 保守抑制极限环。输出限幅放满 10A：推弹瞬间负载大，
 * 8A 不够顶会掉速、弹丸顶开后积分+饱和 P 又把电机猛冲到 5570rpm(超指令)，
 * 猛撞下一颗弹导致卡死(I5 冻结)。满电流让推弹更稳、掉速更小。
 * 积分 KI=2、限幅 2000 防 windup 超速；分离=0 始终积分扛负载。 */
#define LAUNCH_M2006_ID5_SPEED_KP         10.0f
#define LAUNCH_M2006_ID5_SPEED_KI         2.0f
#define LAUNCH_M2006_ID5_SPEED_KD         0.0f
#define LAUNCH_M2006_ID5_INTEGRAL_LIMIT   2000.0f
#define LAUNCH_M2006_ID5_OUTPUT_LIMIT     10000.0f
#define LAUNCH_M2006_ID5_INTEGRAL_SEPARATION_RPM 0.0f
/* 速度滤波 alpha：0.8 滞后小(~2.5ms)，避免快电机刹车前冲过指令速度。 */
#define LAUNCH_M2006_ID5_SPEED_LPF_ALPHA  0.80f
#define LAUNCH_M2006_ID5_DIRECTION        1.0f

/* ---------------- 拨弹 M2006 ID5 角度-速度双环 ----------------
 * 编码器在电机轴(8192 计数/圈)，拨盘在 P36 减速箱输出端(36:1)，
 * 输出角 = 电机角/36。S1 语义：1=保持(角度环)  2=连发(纯速度环 4800rpm=20Hz)
 * 3=单动(角度环)：每次从 1 拨到 3 触发一步 +40°输出。
 * 36:1 使电机端反射惯量放大 36²，动态变慢，比直驱更易控稳。 */
#define M2006_ENCODER_COUNTS_PER_REV      8192.0f
#define M2006_OUTPUT_GEAR_RATIO           36.0f
#define LAUNCH_M2006_ID5_STEP_DEG         40.0f  /* 每发 = 输出轴 40° = 电机 1440° */
/* 连发档名义转速 4800rpm(20Hz 步进)；实际由角度环追目标决定，上限
 * ANGLE_MAX_SPEED_RPM_CONT。 */
#define LAUNCH_M2006_ID5_CONTINUOUS_SPEED_RPM 4800.0f
#define LAUNCH_M2006_ID5_ANGLE_KP_RPM_PER_DEG 40.0f  /* 输出°→电机rpm：10°误差→400rpm */
#define LAUNCH_M2006_ID5_ANGLE_MAX_SPEED_RPM   250.0f /* 单动接近速度降到 250，减速更从容 */
#define LAUNCH_M2006_ID5_ANGLE_MAX_SPEED_RPM_CONT 5000.0f /* 连发档限速：需追 4800rpm 目标 */
#define LAUNCH_M2006_ID5_AUTO_STEP_PERIOD_MS   50U        /* 连发档步进周期 = 20Hz */
#define LAUNCH_M2006_ID5_ANGLE_DEADBAND_DEG    4.0f  /* 到位死区：误差<4°断电靠摩擦停 */
#define M2006_STOP_DEADBAND_RPM                5.0f  /* M2006 速度环断电阈值(rpm) */
#define M2006_CMD_STALE_TIMEOUT_MS             100U  /* 命令保活：launch 卡死则断电 */

#define TASK_DEG_TO_RAD                   0.017453292519943295f

/* ---------------- Yaw 系统辨识模式 ----------------
 * YAW_SYSID_MODE=0：完全的原程序——pitch/yaw 正常 PID，VOFA 按原 6 通道帧
 * 连续打印，行为与原固件一致。
 * YAW_SYSID_MODE=1：辨识固件——pitch 电机不输出，yaw 不使用 PID(空闲 0 电流
 * 自由)；串口静默，收到 identify_on 后 DM4310 直通正弦线性扫频并开始打印
 * (I6=给 DM4310 的电流指令，I7=yaw 原始速度 rpm 不滤波)，运行
 * YAW_SYSID_DURATION_MS 后自动停止打印与激励，可重复触发。
 * 扫频频率随时间线性：f(t)=FREQ_START+(FREQ_END-FREQ_START)*t/时长。
 * 幅值上限 = YAW_CURRENT_OUTPUT_LIMIT(16384, DM4310 协议上限)。 */
#define YAW_SYSID_MODE                    1U
#define YAW_SYSID_AMPLITUDE_CURRENT       8000.0f   /* 扫频幅值(≤16384) */
#define YAW_SYSID_FREQ_START_HZ           1.0f      /* 起始频率 */
#define YAW_SYSID_FREQ_END_HZ             20.0f     /* 结束频率(线性扫频) */
#define YAW_SYSID_DURATION_MS             20000U    /* 单次辨识时长 */

#endif
