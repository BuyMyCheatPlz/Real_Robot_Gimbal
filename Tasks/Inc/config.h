#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

/* ---------------- 任务周期与安全保护 ---------------- */
#define CONTROL_PERIOD_S                  0.001f
#define CONTROL_PERIOD_TICKS              1U
#define CAN_COMMAND_PERIOD_MS             1U
#define CAN_TX_STUCK_ABORT_MS             20U   /* 邮箱被未ACK帧卡住超过此时长则中止释放 */
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
#define IMU_GYRO_ROLL_AXIS                2U
/* 实测：X 轴(0)显示为 pitch 且向下为正，Y 轴(1)为 yaw，因此：
 * pitch=X(0)、yaw=Y(1)、roll=Z(2)。
 * 符号：pitch 用 X 轴(向下为正)；roll 用 Z 轴，取反使其与加速度计(右倾为正)一致。 */
#define IMU_GYRO_PITCH_AXIS               0U
#define IMU_GYRO_YAW_AXIS                 1U
#define IMU_GYRO_ROLL_SIGN               -1.0f
#define IMU_GYRO_PITCH_SIGN               1.0f
#define IMU_GYRO_YAW_SIGN                 1.0f
#define ATTITUDE_ACCEL_WEIGHT             0.010f
#define ATTITUDE_MAX_DT_S                 0.010f

/* ---------------- 云台反馈低通滤波 ---------------- */
#define PITCH_ENCODER_LPF_ALPHA           0.15f
#define PITCH_SPEED_LPF_ALPHA             0.20f
/* 重力前馈单独滤波 Roll：只抑制前馈高频扰动，不给 Pitch 位置/速度环增加滞后。 */
#define PITCH_GRAVITY_ROLL_LPF_ALPHA      0.05f
#define YAW_ENCODER_LPF_ALPHA             0.40f   /* 位置反馈滞后↓(30°快移用) */
#define YAW_SPEED_LPF_ALPHA               0.50f   /* 速度反馈滞后↓(30°快移用) */
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
/* 前馈整定完成后必须启用位置/速度闭环；仅前馈无法在水平附近保持角度。 */
#define PITCH_GRAVITY_ONLY_ENABLE         0U
/* 新云台首次回零采用保守的外环：上电实测约 37° 偏差时，旧 800/90 组合
 * 会立即以最大速度贯穿整个行程。确认方向和阻尼后再逐步增加。 */
#define PITCH_ANGLE_KP_RPM_PER_RAD         75.0f
#define PITCH_ANGLE_KI_RPM_PER_RAD_S      0.0f
/* 回零阶段的位置反馈来自 BMI088。1 ms 周期直接对该角度求导会把传感器噪声
 * 放大到速度限幅，导致目标速度在 ±90 rpm 间翻转；Pitch 外环不使用 D 项。 */
#define PITCH_ANGLE_KD_RPM_S_PER_RAD       2.5f
#define PITCH_ANGLE_INTEGRAL_LIMIT_RPM    30.0f
#define PITCH_MAX_SPEED_RPM                90.0f
#define PITCH_SPEED_KP                     70.0f
#define PITCH_SPEED_KI                      4.0f
#define PITCH_SPEED_KD                    0.0f
#define PITCH_SPEED_INTEGRAL_LIMIT         12000.0f
#define PITCH_SPEED_OUTPUT_LIMIT           25000.0f
#define PITCH_SPEED_INTEGRAL_SEPARATION_RPM 70.0f
/* 起步助推仅用于克服静摩擦。旧值 1 rpm/8000 会把 IMU 噪声造成的微小反向
 * 速度请求放大为满幅翻转；只在明显运动请求时以受限幅值介入。 */
#define PITCH_STARTUP_SPEED_THRESHOLD_RPM   1.0f
#define PITCH_STARTUP_MIN_VOLTAGE         8000.0f
#define PITCH_ANGLE_INTEGRAL_SEPARATION_RAD (10.0f * TASK_DEG_TO_RAD)
/* 目标死区：位置误差小于该角(°)时位置环输出 0，靠重力前馈+速度环把轴稳住，
 * 防止齿距(背隙)在目标附近引起高频抖动。设 0 关闭死区；抖得凶就调大，但过大
 * 会降低到位精度(一般略大于背隙即可)。 */
#define PITCH_POSITION_DEADZONE_RAD         (0.5f * TASK_DEG_TO_RAD)
/* 实测机械限位(IMU 角度，rad)：-2.4260 ≈ -139°(最高)，-1.1170 ≈ -64°(最低)。
 * 卡限幅检测(仅正常模式，gravity_only 不启用)：位置环给了大速度指令但 IMU 角速度
 * 很小 → 判定顶死在机械限位，把目标回锚到当前编码器位置。 */
#define PITCH_LIMIT_MIN_RAD                (-2.4260f)
#define PITCH_LIMIT_MAX_RAD                (-1.1170f)
#define PITCH_LIMIT_STALL_CMD_RPM          30.0f   /* 速度指令大于此值(rpm)判定在推 */
#define PITCH_LIMIT_STALL_SPEED_RAD_S      0.3f    /* IMU 角速度小于此值(rad/s)判定没动 */
#define PITCH_LIMIT_STALL_TIME_MS          150U    /* 卡限幅持续此时长才回锚(防阶跃起步误判) */
#define PITCH_HOME_STABLE_TIME_MS          200U    /* 回零到位：水平死区内稳定此时长判定到达 */
/* 按模板工程的有符号正弦前馈，直接使用 Roll。驱动层将前馈加到
 * 速度环输出，因此调用处会取负，实现模板中的“速度环输出 - 前馈”。 */
#define PITCH_GRAVITY_FF_MAX_VOLTAGE         13000.0f
#define PITCH_GRAVITY_ZERO_RAD             0.0f       /* Roll=0°：前馈过零 */
#define PITCH_GRAVITY_SIGN                -1.0f
#define PITCH_MOTOR_SIGN                  1.0f
/* 编码器展开角度与 IMU Pitch 的增量方向。CSV 表明 I1 与 I11 反向：
 * I11=-64° 为最低、-139° 为最高，因此该符号必须为 -1。 */
#define PITCH_ENCODER_TO_IMU_SIGN        (-1.0f)
#define PITCH_SOFT_LIMIT_DEG              90.0f

/* ---------------- Yaw：DM4310 外位置环 + 软件速度环 ----------------
 * 30° 阶跃(≤200ms、超调≤0.2°)整定组。依据辨识：电流→速度≈积分器(自由轴)，
 * 速度环 P 即稳定；轨迹(高速/大加速度)+ 速度/加速度前馈负责 200ms 快速到位，
 * 位置环做末端修正与防超调。
 * 现场微调方向：
 *  超调>0.2°      → 减 YAW_VELOCITY_FF_GAIN / 加 YAW_ANGLE_KD /
 *                   减 YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2
 *  到位偏慢/滞后大 → 加 YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2(先看电流是否顶到
 *                   16384，若顶到则只能降 YAW_TRAJECTORY_MAX_ACCEL_RAD_S2 放慢)
 *  末端小抖/噪声   → 略降 YAW_SPEED_KP 或回调滤波 alpha。 */
#define YAW_ANGLE_KP_RAD_S_PER_RAD        2.00f
#define YAW_ANGLE_KI_RAD_S_PER_RAD_S      0.02f
#define YAW_ANGLE_KD_RAD_S2_PER_RAD       0.10f
#define YAW_ANGLE_INTEGRAL_LIMIT_RAD_S    0.04f
#define YAW_MAX_SPEED_RAD_S               7.0f
/* Yaw 目标轨迹，轨迹单位为输出轴弧度。 */
#define YAW_TRAJECTORY_MAX_SPEED_RAD_S    6.5f
#define YAW_TRAJECTORY_MAX_ACCEL_RAD_S2   60.0f
/* 1.0：直接使用规划速度做速度前馈。 */
#define YAW_VELOCITY_FF_GAIN              1.00f
/* 可直接调节的力矩前馈：单位为 CAN 电流命令单位/输出轴 rad/s²。
 * 正值表示输出轴正加速度，控制器会应用 YAW_MOTOR_COMMAND_SIGN。 */
#define YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2 65.0f
#define YAW_ACCELERATION_FF_CURRENT_LIMIT  16384.0f
#define YAW_SPEED_KP_CURRENT_PER_RPM       100.0f
#define YAW_SPEED_KI_CURRENT_PER_RPM_S     15.0f
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
 * 电机轴转速环。输出限幅放满 10A：推弹瞬间负载大，8A 不够顶会掉速。
 * 之前 KP=10/KI=2 偏保守导致"实际比目标少 1~2 颗"(连发稳态掉速+起步滞后)，
 * 这里把 KP 提到 12、KI 提到 3、积分限幅 2000→3000，让速度环更跟手、稳态误差更小。
 * 注意别再加太多，否则弹丸顶开后积分+饱和 P 会把电机猛冲过 4800 撞下一颗弹。 */
#define LAUNCH_M2006_ID5_SPEED_KP         12.0f
#define LAUNCH_M2006_ID5_SPEED_KI         3.0f
#define LAUNCH_M2006_ID5_SPEED_KD         0.0f
#define LAUNCH_M2006_ID5_INTEGRAL_LIMIT   3000.0f
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
#define LAUNCH_M2006_ID5_ANGLE_MAX_SPEED_RPM   1000.0f /* 单动接近速度：250 太慢(约1s/发)，提到 1000(约0.3s/发) */
#define LAUNCH_M2006_ID5_ANGLE_MAX_SPEED_RPM_CONT 5000.0f /* 连发档限速：需追 4800rpm 目标 */
#define LAUNCH_M2006_ID5_AUTO_STEP_PERIOD_MS   50U        /* 连发档步进周期 = 20Hz */
#define LAUNCH_M2006_ID5_ANGLE_DEADBAND_DEG    1.5f  /* 到位死区：4°太大每发都短2~3°，累积成"少1~2颗"；收到1.5°让拨盘转到位 */
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
 * 幅值上限 = YAW_CURRENT_OUTPUT_LIMIT(16384, DM4310 协议上限)。
 * 实测：>~15Hz 后速度反馈与指令失去相关(±40rpm 抖动)属无效段；
 * 上限取 10Hz 可让全程数据都有效到最后一个采样点。若需更高频段，
 * 适当上调 FREQ_END_HZ 或降低幅值。 */
#define YAW_SYSID_MODE                    0U
#define YAW_SYSID_AMPLITUDE_CURRENT       8000.0f   /* 扫频幅值(≤16384) */
#define YAW_SYSID_FREQ_START_HZ           1.0f      /* 起始频率 */
#define YAW_SYSID_FREQ_END_HZ             10.0f     /* 结束频率(线性扫频,≤10Hz 数据有效) */
#define YAW_SYSID_DURATION_MS             20000U    /* 单次辨识时长 */

#endif
