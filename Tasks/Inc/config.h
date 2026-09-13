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
#define VOFA_PERIOD_MS                     5U
#define VOFA_PITCH_TUNING_MODE              0U
#define VOFA_YAW_TUNING_MODE                0U
#define VOFA_LAUNCH_TUNING_MODE             1U
/* 0：正常调参通道；1：临时打印 BMI088 三轴方向诊断通道。 */
#define VOFA_IMU_AXIS_DEBUG_MODE            0U
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
/* 遥控 Pitch 步进与实际 Pitch 正方向的关系。 */
#define PITCH_STICK_DIR                  (-1.0f)

/* ---------------- BMI088 安装方向与姿态滤波 ----------------
 * 轴编号对应数组下标：X=0、Y=1、Z=2。
 *
 * 注意历史命名：控制链路里 roll_rate_rad_s 被当作本机 Pitch 速度反馈使用。
 * 最新 VOFA 三轴诊断数据表明，本机 Pitch 主轴为 BMI088 raw X，且与 Pitch
 * 编码器展开角 I1 的增量同向；若用于闭环抑制大负载超调，roll_rate_rad_s
 * 应映射到 raw X、sign=+1。若临时改回其他轴做对比，必须同步看 VOFA I4
 * 是否仍与 d(I1)/dt 同向同幅。
 */
#define IMU_ACCEL_X_AXIS                  0U
#define IMU_ACCEL_Y_AXIS                  1U
#define IMU_ACCEL_Z_AXIS                  2U
#define IMU_ACCEL_X_SIGN                  1.0f
#define IMU_ACCEL_Y_SIGN                  1.0f
#define IMU_ACCEL_Z_SIGN                  1.0f
#define IMU_GYRO_ROLL_AXIS                0U
/* 控制链路沿用 roll_rate_rad_s 作为本机 Pitch 速度反馈，因此 roll 必须映射
 * 到实测 Pitch 主轴 raw X；pitch_rate_rad_s 保留 raw Z 用于姿态诊断。 */
#define IMU_GYRO_PITCH_AXIS               2U
#define IMU_GYRO_YAW_AXIS                 1U
#define IMU_GYRO_ROLL_SIGN                1.0f
#define IMU_GYRO_PITCH_SIGN               1.0f
#define IMU_GYRO_YAW_SIGN                 1.0f
#define ATTITUDE_ACCEL_WEIGHT             0.010f
#define ATTITUDE_MAX_DT_S                 0.010f

/* ---------------- 云台反馈低通滤波 ---------------- */
/* GM6020 编码器分辨率足以满足 ±0.2°；设为 1 直接使用新反馈，避免原 0.15
 * 在高速段产生约 5.7 ms 延迟和数度动态位置误差。 */
#define PITCH_ENCODER_LPF_ALPHA           1.0f
/* Pitch 速度环直接使用映射后的 BMI088 角速度反馈，不再额外低通。 */
#define PITCH_SPEED_LPF_ALPHA             1.0f
/* 旧 IMU Roll 前馈滤波宏；当前动态重力前馈应优先使用连续 Pitch 编码器角，
 * 避免融合角/低通在阶跃中滞后。 */
#define PITCH_GRAVITY_ROLL_LPF_ALPHA      0.05f
/* 安装非正交造成的 Yaw→Roll 陀螺串扰补偿；0 表示关闭，现场标定后再修改。 */
#define PITCH_ROLL_YAW_CROSS_RATE          0.0f
/* 将本机 Pitch 物理角速度转换到 GM6020 电机坐标。若 raw X 与编码器 I1
 * 同向，且编码器到 IMU 坐标符号为 -1，则电机速度反馈仍需乘该组合符号。 */
#define PITCH_ROLL_RATE_TO_SPEED_SIGN      (PITCH_MOTOR_SIGN * PITCH_ENCODER_TO_IMU_SIGN)
#define YAW_ENCODER_LPF_ALPHA             0.40f   /* 位置反馈滞后↓(30°快移用) */
#define YAW_SPEED_LPF_ALPHA               0.40f   /* 速度反馈抖动↓，兼顾 yaw 200ms 阶跃 */
/* IMU Yaw 是当前的位置环测量值。进入位置 PID 前先滤波；电机编码器仍作为
 * 内层速度反馈来源。 */
#define YAW_IMU_POSITION_LPF_ALPHA         0.02f
/* 最终目标保持使用滞回，轨迹运动期间不启用。退出阈值满足不超过 0.2° 的精度要求。 */
#define YAW_HOLD_ENTER_ERROR_RAD           (0.05f * TASK_DEG_TO_RAD)
#define YAW_HOLD_EXIT_ERROR_RAD            (0.20f * TASK_DEG_TO_RAD)
#define YAW_HOLD_ENTER_SPEED_RPM            0.35f
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
/* 重力前馈标定开关：1=仅输出重力前馈，Pitch 位置/速度闭环旁路、速度目标清零；
 * 标定完成后必须改回 0，仅前馈无法可靠保持全角度。 */
#define PITCH_GRAVITY_ONLY_ENABLE         0U
/* 新云台首次回零采用保守的外环：上电实测约 37° 偏差时，旧 800/90 组合
 * 会立即以最大速度贯穿整个行程。确认方向和阻尼后再逐步增加。 */
/* 宏名为历史遗留；Pitch 双环内部使用位置 °、速度 °/s。 */
#define PITCH_ANGLE_KP_RPM_PER_RAD          43.6f
#define PITCH_ANGLE_KI_RPM_PER_RAD_S        39.64f
/* 实测接近目标时外环 D 会把同向速度目标提前压成零，并与制动前馈叠加形成冲击。
 * Pitch 阻尼改由陀螺速度环和停车距离制动提供，外环 D 暂时关闭。 */
#define PITCH_ANGLE_KD_RPM_S_PER_RAD         0.0f
#define PITCH_ANGLE_INTEGRAL_LIMIT_RPM    30.0f
/* 仅抑制实测 -60° 处的 Pitch 位置 D 窄带共振。陷波不进入 P/I、
 * 速度环或前馈路径；3.33 Hz 增益保持 0.99 以上，避免影响 300 ms 阶跃要求。 */
#define PITCH_POSITION_D_NOTCH_ENABLE         1U
#define PITCH_POSITION_D_NOTCH_CENTER_HZ     32.0f
#define PITCH_POSITION_D_NOTCH_Q              1.5f
/* 新一轮 tuning mode 数据显示主振荡漂到 23.7 Hz，串联第二个陷波保留旧
 * 32 Hz 抑制能力，同时压当前 -60° 稳态限环。 */
#define PITCH_POSITION_D_NOTCH2_ENABLE        1U
#define PITCH_POSITION_D_NOTCH2_CENTER_HZ    23.7f
#define PITCH_POSITION_D_NOTCH2_Q             1.5f
/* 低角度区位置 D 衰减：只改变外环 D 项，P/I、速度环和前馈不变。
 * START 到 FULL 之间线性过渡；FULL 以下使用 SCALE。SCALE=1 表示不衰减。 */
#define PITCH_POSITION_D_LOW_ANGLE_SCALE_ENABLE 1U
#define PITCH_POSITION_D_LOW_ANGLE_START_DEG  (-45.0f)
#define PITCH_POSITION_D_LOW_ANGLE_FULL_DEG   (-55.0f)
#define PITCH_POSITION_D_LOW_ANGLE_SCALE        0.45f
/* Pitch 大阶跃起步段不靠位置 D 抑制：误差大于 DISABLE 时 D=0，
 * 误差小于 RESTORE 时恢复到低角度衰减后的 D，中间线性渐入。 */
#define PITCH_POSITION_D_STEP_FADE_ENABLE       1U
#define PITCH_POSITION_D_STEP_DISABLE_ERROR_DEG 5.0f
#define PITCH_POSITION_D_STEP_RESTORE_ERROR_DEG 2.0f
/* 位置环输出方向保护：误差仍大时，不允许 D 项把速度目标推成远离目标的方向。 */
#define PITCH_POSITION_OUTPUT_DIRECTION_GUARD_ENABLE 1U
#define PITCH_POSITION_OUTPUT_DIRECTION_GUARD_ERROR_DEG 0.6f
/* 到位保持带：硬置零会在残余速度未完全消失时放开控制，实测会形成到位后波动；
 * 默认关闭，仅保留为回退开关。 */
#define PITCH_SETTLE_HOLD_ENABLE             0U
#define PITCH_SETTLE_HOLD_ENTER_ERROR_DEG    0.18f
#define PITCH_SETTLE_HOLD_EXIT_ERROR_DEG     0.35f
#define PITCH_SETTLE_HOLD_ENTER_SPEED_DEG_S 18.0f
/* 旧近目标软限速在 0.6° 边界把速度指令从约 12 突然切回 26°/s，形成回摆；
 * 当前关闭，由连续位置 PI 和停车距离限速共同收速。 */
#define PITCH_NEAR_TARGET_SPEED_CLAMP_ENABLE 0U
#define PITCH_NEAR_TARGET_SPEED_CLAMP_ERROR_DEG 0.6f
#define PITCH_NEAR_TARGET_SPEED_LIMIT_DEG_S 12.0f
/* 小误差低速静态纠偏：死区外连续增加补偿，误差超过 MAX 后保持端点值，
 * 不再在 MAX 边界突然撤掉。输出在电机指令域。 */
#define PITCH_STATIC_ERROR_COMP_ENABLE      1U
#define PITCH_STATIC_ERROR_COMP_DEADBAND_DEG 0.08f
#define PITCH_STATIC_ERROR_COMP_MAX_ERROR_DEG 1.0f
#define PITCH_STATIC_ERROR_COMP_FULL_SPEED_DEG_S 1.0f
#define PITCH_STATIC_ERROR_COMP_FADE_SPEED_DEG_S 8.0f
#define PITCH_STATIC_ERROR_COMP_VOLT_PER_DEG 20000.0f
#define PITCH_STATIC_ERROR_COMP_LIMIT       3200.0f
/* -60° 区域的瞬时静差补偿会与机构 17.5 Hz 模态形成极限环，因此在低角度区
 * 使用较低刚度，并对所有角度的补偿输出限制变化率。 */
#define PITCH_STATIC_ERROR_COMP_LOW_ANGLE_TARGET_MAX_DEG (-45.0f)
#define PITCH_STATIC_ERROR_COMP_LOW_ANGLE_VOLT_PER_DEG 5000.0f
#define PITCH_STATIC_ERROR_COMP_LOW_ANGLE_LIMIT       1000.0f
/* 40000/s 等于每个 1 ms 控制周期最多变化 40 个电压指令单位。 */
#define PITCH_STATIC_ERROR_COMP_SLEW_VOLT_PER_S      40000.0f
/* 目标大步进会让误差 D 吃到目标阶跃，并把上一角度的积分带到下一角度。
 * 超过该阈值时重置 Pitch 位置环状态，仅清位置环 I/D 记忆，不改速度环和前馈。 */
#define PITCH_POSITION_STEP_RESET_RAD       (5.0f * TASK_DEG_TO_RAD)
/* Pitch 限速总开关：0=不限制轨迹速度/加速度，也不限制位置环速度目标；
 * 1=使用 PITCH_MAX_SPEED_RPM 和 PITCH_TRAJECTORY_* 做保守阶跃。 */
#define PITCH_SPEED_LIMIT_ENABLE           0U
#define PITCH_MAX_SPEED_RPM                70.0f
/* 接近目标动态限速：远离目标时允许 P 给大速度，接近目标时按含响应延迟的
 * 停车距离反解允许速度，避免靠继续加 D 来压超调。单位沿用本工程 Pitch
 * 外环的 deg/s。 */
#define PITCH_APPROACH_SPEED_LIMIT_ENABLE  1U
#define PITCH_APPROACH_BRAKE_ACCEL_DEG_S2  2500.0f
#define PITCH_APPROACH_MIN_SPEED_DEG_S       25.0f
/* 小误差区最小速度渐隐：避免接近目标后仍因 25°/s 下限来回摆动。 */
#define PITCH_APPROACH_MIN_SPEED_FADE_ENABLE 1U
#define PITCH_APPROACH_MIN_SPEED_FADE_ERROR_DEG 0.6f
#define PITCH_APPROACH_MIN_SPEED_NEAR_DEG_S  3.0f
/* 三类运动使用不同停车模型：低角度向上保留已验证的 0.60 衰减；高角度
 * 向上实际制动更强，因此提高等效减速度。普通下降与高角度上升都提前收速，
 * 分别压低 0→-30 的越线回摆和 -30→0 的末段静差。 */
#define PITCH_APPROACH_LOW_ANGLE_SCALE_ENABLE 1U
#define PITCH_APPROACH_LOW_ANGLE_TARGET_MAX_DEG (-25.0f)
#define PITCH_APPROACH_LOW_ANGLE_SCALE          0.60f
#define PITCH_APPROACH_LOW_TARGET_DOWN_MAX_DEG (-45.0f)
#define PITCH_APPROACH_DOWN_BRAKE_DELAY_S        0.030f
#define PITCH_APPROACH_LOW_TARGET_DOWN_BRAKE_DELAY_S 0.030f
#define PITCH_APPROACH_LOW_ANGLE_UP_BRAKE_DELAY_S 0.030f
#define PITCH_APPROACH_HIGH_ANGLE_UP_ACCEL_SCALE  2.00f
#define PITCH_APPROACH_HIGH_ANGLE_UP_BRAKE_DELAY_S 0.055f
/* 速度目标只向目标方向收缩，不生成反向速度。停车曲线计入控制响应延迟：
 * distance = |v|*delay + v^2/(2*a)。动态制动前馈与速度 PI 重复使用同一
 * 超速量，实测会造成到位前反转，因此关闭，仅由速度 PI 执行制动。 */
#define PITCH_APPROACH_BRAKE_FF_ENABLE      0U
#define PITCH_APPROACH_LOW_ANGLE_BRAKE_START_ERROR_DEG 6.0f
#define PITCH_APPROACH_LOW_ANGLE_BRAKE_FULL_ERROR_DEG  4.0f
#define PITCH_APPROACH_BRAKE_FF_STOP_SPEED_DEG_S 3.0f
#define PITCH_APPROACH_DOWN_BRAKE_FF_GAIN       120.0f
#define PITCH_APPROACH_DOWN_BRAKE_FF_LIMIT     8000.0f
#define PITCH_APPROACH_LOW_ANGLE_UP_BRAKE_FF_GAIN 180.0f
#define PITCH_APPROACH_LOW_ANGLE_UP_BRAKE_FF_LIMIT 8000.0f
#define PITCH_APPROACH_HIGH_ANGLE_UP_BRAKE_FF_GAIN 90.0f
#define PITCH_APPROACH_HIGH_ANGLE_UP_BRAKE_FF_LIMIT 4500.0f
/* 400000/s 等于每个 1 ms 控制周期最多变化 400 个电压指令单位。 */
#define PITCH_APPROACH_BRAKE_FF_SLEW_VOLT_PER_S 400000.0f
#define PITCH_SPEED_KP                    172.6f
#define PITCH_SPEED_KI                      12.0f
/* Pitch 速度环 D 项是“每 1 ms 拍的误差差量”：
 * kd*(e[k]-e[k-1])（不除以 dt）。切勿把它当“每秒导数/除以 dt”的增益：
 * dt=1 ms 时同数值会被放大约 1000 倍，一加 D 就满幅抖振。 */
#define PITCH_SPEED_KD                      0.0f
#define PITCH_SPEED_INTEGRAL_LIMIT          3000.0f
#define PITCH_SPEED_OUTPUT_LIMIT           30000.0f
/* 速度环积分分离(误差超过该值停止积分)。原 70 几乎等于全带积分，
 * 低频相位滞后大、外环 KP 一高就易起振。先收窄到 5，
 * 若积分抗静摩擦不足可再放回 10~20。 */
#define PITCH_SPEED_INTEGRAL_SEPARATION_RPM 5.0f
/* 起步助推“开关式”最小力矩：|速度指令|≥阈值 且 |PID输出|<下限时强制顶到 ±MIN，
 * 等于一个与 PID 增益无关的 bang-bang 继电器。
 * 实测它在目标附近保持时把 ±0.2~1° 误差变为 ±4~22°/s 指令（>1 阈值）后，
 * 输出被强制成 ±8000 满力矩来回打 → 形成 ~3.3~3.6 Hz、参数调不掉的非线性极限环
 * （pitch.csv）。故置 0 关闭。若阶跃起步出现静摩擦停顿，
 * 再开但把阈值提到 10~15°/s、下限降到 2000~4000 减小冲击。 */
#define PITCH_STARTUP_SPEED_THRESHOLD_RPM   1.0f
#define PITCH_STARTUP_MIN_VOLTAGE           0.0f
/* 位置环积分分离：大误差阶跃过程不积分，靠近目标后再用 I 消稳态误差。 */
#define PITCH_ANGLE_INTEGRAL_SEPARATION_RAD (1.5f * TASK_DEG_TO_RAD)
/* 预留的目标死区参数，当前未接入 Pitch 控制链。MISSION 要求保持在 ±0.2° 内，
 * 因此不能直接启用当前 0.5° 数值来掩盖稳态抖动。 */
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
/* 0：Pitch 上电授权后以 IMU 水平为零点，自动回到水平；
 * 1：Pitch 上电授权后以当前上电位置为零点，当前位置保持不动。 */
#define PITCH_HOME_TO_POWER_ON_POSITION     1U
/* Pitch 重力前馈 3 阶标定曲线，输出直接作为 GM6020 电压前馈。
 * PITCH_GRAVITY_C1_GR 是 C1_gr 的上电默认值，运行时可在 Watch 里改 C1_gr。 */
#define PITCH_GRAVITY_FIT_MIN_DEG          (-50.0f)
#define PITCH_GRAVITY_FIT_MAX_DEG            26.0f
#define PITCH_GRAVITY_POLY_C0             1678.105241f
#define PITCH_GRAVITY_C1_GR                161.466898f
#define PITCH_GRAVITY_POLY_C2             (-0.590091f)
#define PITCH_GRAVITY_POLY_C3               0.006451f
#define PITCH_GRAVITY_POLY_C4               0.0f
#define PITCH_MOTOR_SIGN                  1.0f
/* 编码器展开角度与 IMU Pitch 的增量方向。CSV 表明 I1 与 I11 反向：
 * I11=-64° 为最低、-139° 为最高，因此该符号必须为 -1。 */
#define PITCH_ENCODER_TO_IMU_SIGN        (-1.0f)
#define PITCH_SOFT_LIMIT_DEG              90.0f
/* 收到 Yaw 遥控步进后短暂锁存 Pitch，防止 Yaw→Roll 串扰进入 Pitch 速度环。 */
#define PITCH_LATCH_AFTER_YAW_CMD_MS       500U
#define PITCH_LATCH_YAW_SETTLED_DEG        0.5f
/* Pitch 有限加速度轨迹：仅在 PITCH_SPEED_LIMIT_ENABLE=1 时生效。 */
#define PITCH_TRAJECTORY_MAX_SPEED_RAD_S   1.2f
#define PITCH_TRAJECTORY_MAX_ACCEL_RAD_S2  8.0f

/* ---------------- Yaw：DM4310 外位置环 + 软件速度环 ----------------
 * 30° 阶跃整定组。上一组提高速度环 P 后出现满幅换向自激，当前
 * 回退到较软的速度环与低最小电流。最新 yaw.csv 中部分位置 240 ms
 * 附近仍有 0.3~0.45° 超调，当前小幅回收速度前馈与位置积分，减少
 * 进入目标时的尾速和过目标后的持续推力。
 * 现场微调方向：
 *  超调>0.2°      → 减 YAW_VELOCITY_FF_GAIN / 加 YAW_ANGLE_KD /
 *                   减 YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2
 *  到位偏慢/滞后大 → 先看减速段 I7 是否提前反向；若提前反向，减小
 *                   YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2 或增大速度环 P。
 *  末端小抖/噪声   → 略降 YAW_SPEED_KP 或回调滤波 alpha。 */
#define YAW_ANGLE_KP_RAD_S_PER_RAD        5.20f
#define YAW_ANGLE_KI_RAD_S_PER_RAD_S      0.18f
/* 位置环 D 使用“每拍误差差量”语义 kd*(e[k]-e[k-1])（不除以 dt）。
 * 原 0.10 是按“每秒导数”写的等效值（≈速度阻尼 0.1），换算为每拍语义：
 * 0.10 / 0.001 = 100，行为不变。 */
#define YAW_ANGLE_KD_RAD_S2_PER_RAD        88.0f
#define YAW_ANGLE_INTEGRAL_LIMIT_RAD_S    0.12f
#define YAW_MAX_SPEED_RAD_S               7.0f
/* Yaw 目标轨迹，轨迹单位为输出轴弧度。 */
#define YAW_TRAJECTORY_MAX_SPEED_RAD_S    8.8f
#define YAW_TRAJECTORY_MAX_ACCEL_RAD_S2   87.0f
/* 1.0：直接使用规划速度做速度前馈。 */
#define YAW_VELOCITY_FF_GAIN              0.89f
/* 可直接调节的力矩前馈：单位为 CAN 电流命令单位/输出轴 rad/s²。
 * 正值表示输出轴正加速度，控制器会应用 YAW_MOTOR_COMMAND_SIGN。 */
#define YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2 15.0f
#define YAW_ACCELERATION_FF_CURRENT_LIMIT  16384.0f
#define YAW_SPEED_KP_CURRENT_PER_RPM       190.0f
#define YAW_SPEED_KI_CURRENT_PER_RPM_S     15.0f
#define YAW_SPEED_KD_CURRENT_S_PER_RPM      0.0f
#define YAW_SPEED_INTEGRAL_LIMIT_CURRENT  16384.0f
/* 克服 DM4310 与机构静摩擦的最小启动电流；目标速度为零时不生效。 */
#define YAW_STARTUP_SPEED_THRESHOLD_RPM    0.20f
#define YAW_STARTUP_MIN_CURRENT              80.0f
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
#define LAUNCH_M3508_ID2_SPEED_KP         15.0f
#define LAUNCH_M3508_ID2_SPEED_KI         0.5f
#define LAUNCH_M3508_ID2_SPEED_KD         0.0f
#define LAUNCH_M3508_ID2_INTEGRAL_LIMIT   16384.0f
#define LAUNCH_M3508_ID2_OUTPUT_LIMIT     16384.0f
#define LAUNCH_M3508_ID2_INTEGRAL_SEPARATION_RPM 1000.0f
#define LAUNCH_M3508_ID2_SPEED_LPF_ALPHA  0.20f
#define LAUNCH_M3508_ID2_DIRECTION        1.0f

/* ---------------- 发射 M3508 ID3 速度环 PID ---------------- */
#define LAUNCH_M3508_ID3_SPEED_KP         15.0f
#define LAUNCH_M3508_ID3_SPEED_KI         0.5f
#define LAUNCH_M3508_ID3_SPEED_KD         0.0f
#define LAUNCH_M3508_ID3_INTEGRAL_LIMIT   16384.0f  
#define LAUNCH_M3508_ID3_OUTPUT_LIMIT     16384.0f
#define LAUNCH_M3508_ID3_INTEGRAL_SEPARATION_RPM 1000.0f
#define LAUNCH_M3508_ID3_SPEED_LPF_ALPHA  0.20f
#define LAUNCH_M3508_ID3_DIRECTION       (-1.0f)

/* ---------------- 拨弹 M2006 ID5 速度环 PID ----------------
 * 电机轴转速环。最新 2006.csv 显示保持段速度环正负满电流换向，说明零速
 * 制动过硬。当前降低 P/I 与输出限幅，优先消除到位抖动。 */
#define LAUNCH_M2006_ID5_SPEED_KP         24.0f
#define LAUNCH_M2006_ID5_SPEED_KI         1.2f
#define LAUNCH_M2006_ID5_SPEED_KD         0.0f
#define LAUNCH_M2006_ID5_INTEGRAL_LIMIT   1000.0f
#define LAUNCH_M2006_ID5_OUTPUT_LIMIT     6500.0f
#define LAUNCH_M2006_ID5_INTEGRAL_SEPARATION_RPM 0.0f
/* 速度滤波 alpha 越小越稳。拨盘到位附近优先抑制速度噪声放大。 */
#define LAUNCH_M2006_ID5_SPEED_LPF_ALPHA  0.35f
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
#define LAUNCH_M2006_ID5_CONT_PLL_KP_RPM_PER_DEG 60.0f
#define LAUNCH_M2006_ID5_ANGLE_KP_RPM_PER_DEG 85.0f   /* 输出°→电机rpm：40°误差→3400rpm */
#define LAUNCH_M2006_ID5_ANGLE_MAX_SPEED_RPM   3400.0f /* 单动补偿降内环后的速度损失 */
#define LAUNCH_M2006_ID5_ANGLE_MAX_SPEED_RPM_CONT 5600.0f /* 连发档提高追相位余量 */
#define LAUNCH_M2006_ID5_AUTO_STEP_PERIOD_MS   50U        /* 连发档步进周期 = 20Hz */
#define LAUNCH_M2006_ID5_ANGLE_DEADBAND_DEG    0.8f  /* 单发必须走满 40°；松手后继续追目标，死区只留防微抖余量 */
#define M2006_STOP_DEADBAND_RPM                35.0f /* M2006 零速断电阈值，避免低速反复刹车 */
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
