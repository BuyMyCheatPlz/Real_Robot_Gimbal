# FreeRTOS 任务与项目效果说明

## Data_Process 数据处理任务

`Data_Process` 默认每 1 ms 运行一次，负责：

- 原子读取 BMI088 DMA 完整采样结果。
- 推进等待中的 BMI088 DMA 阶段，并在单段超时后执行任务态恢复。
- 使用加速度计和陀螺仪互补滤波计算 Roll、Pitch，并积分得到相对 Yaw。
  该 BMI088 转了 90°（Y 轴朝上），姿态角参考垂直轴 Y：pitch=atan2(-az,ay)、
  roll=atan2(-ax,ay)，轴映射见 `config.h` 的 `IMU_*`。
- 处理大疆 D-BUS 遥控数据和失联状态。
- 检测 CH4/CH1 摇杆越过死区边沿，生成 Pitch/Yaw 的 ±30° 增量命令。
- 将姿态和角度增量通过 `Target_Angle` 队列发送给 `PID_calc`。
- 解析 UART4 收到的在线 PID 调参命令，并通过 `Update_PID_para` 队列下发。
- 将 S1、S2 三档映射后的发射机构目标速度通过 `Update_launch_para` 下发。

## 云台控制任务

`PID_calc` 以 1 kHz 运行：

- Pitch 位置环用 GM6020 编码器展开增量换算后的本机 Pitch 坐标作为实际值；上电/遥控
  重连时用当前映射到 `roll_rad` 的 BMI088 姿态角建立零偏。默认回到 IMU 水平零点；
  `PITCH_HOME_TO_POWER_ON_POSITION=1` 时改为以上电位置为零点并保持当前位置。
  位置环输出目标转速，进入 GM6020 速度 PID。
- `PITCH_POSITION_DEADZONE_RAD` 当前仅为预留参数，未接入 Pitch 控制链；不能用
  0.5° 死区掩盖抖动，否则会超过 MISSION 的 ±0.2°保持精度。
- Pitch 位置环 D 当前设为 0。实测中该 D 项在约 2° 误差处过早抵消 P 项，造成速度目标
  先归零、实际轴在目标前制动后回摆；阻尼改由陀螺仪速度环和停车距离控制承担。
  两级位置 D 陷波与低角度衰减代码保留，重新启用位置 D 时才参与控制。
- Pitch 目标变化超过 `PITCH_POSITION_STEP_RESET_RAD` 时会重置位置环 I/D 记忆；
  位置环积分只在 `PITCH_ANGLE_INTEGRAL_SEPARATION_RAD` 内启用，避免大阶跃积分残留导致超调。
- `PITCH_APPROACH_SPEED_LIMIT_ENABLE=1` 时，位置环速度目标按剩余误差动态限速；
  停车曲线同时计入速度环响应延迟和制动距离，限速结果只允许朝向目标或为零，不用
  反向速度目标制造制动。动态制动前馈与速度 PI 会对同一超速量重复制动、实测造成
  到位前反转，其实现与宏已删除；向下、低角度向上和高角度向上仍使用独立停车模型。
  普通向下计入 30 ms、低目标向下和低角度向上计入 30 ms、高角度向上计入 55 ms 响应距离，
  降低 0° 与 -30° 两端的越线速度，并把末段制动提前分摊。
- Pitch 编码器位置直接使用最新反馈，不再用 0.15 低通引入约 5.7 ms 延迟。
  旧的 0.6° 近目标限速存在增益跳变，其实现已删除，由连续位置 PI 完成最后接近。
- 小误差静差补偿改为带变化率限制的状态量；每 1 ms 最多变化 40 个电压单位，
  -45° 以下使用 5000 电压/°、1000 限幅，避免在 -60° 附近激励 17.5 Hz 摆动。
- Pitch 目标直接钳位到实测机械限位（`PITCH_LIMIT_MIN_RAD`/`PITCH_LIMIT_MAX_RAD`，
  IMU Pitch -139°（最高）~-64°（最低）），不靠卡限位检测。
- `PITCH_SPEED_LIMIT_ENABLE=0` 时直接跟踪阶跃目标，不限制轨迹速度/加速度和位置环
  速度目标；设为 1 时才使用 `PITCH_MAX_SPEED_RPM` 与 `PITCH_TRAJECTORY_*`。
- Pitch 速度环反馈使用映射到 `roll_rate_rad_s` 的 BMI088 角速度（rad/s→deg/s），
  直接测云台真实角速度，不受减速/背隙/柔性影响。最新三轴诊断确认本机 Pitch
  主轴是 raw X，且与编码器实际角同向。
- 正常运行时用连续的 Pitch 编码器角计算 3 阶标定重力电压前馈，避免 IMU 融合角和低通
  滤波在阶跃中滞后（符号/幅值由 `PITCH_GRAVITY_*` 宏配置）。
- IMU Yaw 归一化为 `[-180°, +180°)`；DM4310 编码器 yaw 保持连续展开，
  因而控制过编码器零点时不会跳变。
- 上电时使用首次映射到 `roll_rad` 的 BMI088 姿态角给 GM6020 编码器建立零偏；
  Pitch 默认目标为 IMU 水平零点，也可配置为保持上电位置。
- `YAW_COMMISSIONING_MODE=0` 时两个云台轴允许输出；
  `LAUNCH_MOTOR_OUTPUT_ENABLE=0` 会继续向发射机构发送零命令，避免调试云台时误启动。
- Yaw 使用 DM4310 编码器位置作为实际值。限速度、限加速度轨迹同时生成位置、
  速度和加速度参考；`YAW_VELOCITY_FF_GAIN` 调节速度前馈，
  `YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2` 以“CAN 电流命令单位/rad/s²”直接调节
  加速度力矩前馈。最终电流命令受 `YAW_CURRENT_OUTPUT_LIMIT` 限制。

Yaw 前馈的建议整定顺序：先把 `YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2` 保持为
`0`，固定轨迹最大速度和加速度后，将 `YAW_VELOCITY_FF_GAIN` 从 `0` 以 `0.2`
递增至不再明显减小跟踪误差；再将加速度前馈系数从 `0` 以小步增加。若 VOFA 中
I4 仍为朝向目标的速度目标、但 I7 已经提前反向，说明加速度前馈过强，应先减小
该系数或增大速度环 P。当前 yaw.csv 调参后使用 8.0 rad/s、75 rad/s² 轨迹，
位置环 `KP=5.2, KI=0.18, KD=88`、速度环 `KP=190, KI=15`、速度前馈 `0.89`、
加速度前馈 `15`；末端用 `0.20 rpm` 启动阈值和 `80` 最小电流处理静摩擦，
避免最小电流在目标附近形成硬推力。
- DM4310 驱动对小数 PID/前馈输出使用误差扩散量化。例如连续请求 `0.25` 时
  只间歇发送单帧 `1`，长期平均为 `0.25`；既避免整数截断死区，也不会突破
  单帧安全限幅。
- DM4310 的 0～8191 单圈位置会先跨零展开，再进行低通滤波，避免编码器过零时
  角度跳变。
- Yaw 保持死区只在轨迹已经到达最终目标、轨迹速度接近零且电机低速时进入。
  进入阈值为 0.1°，退出阈值为 0.2°；PID 只在状态切换时重置，移动轨迹中不会
  因为暂时跟上轨迹点而反复刹停。
- 上电稳定确认后的 DM4310 编码器位置作为 Yaw 初始保持位置，不会强制回协议零位。
- 上电授权后先保持 Yaw 零电流；只有连续 200 ms 的新鲜 CAN 反馈同时满足低速和
  位置稳定条件，才按当下编码器位置捕获初始目标并开启闭环。等待时间本身不能
  代替新反馈，避免电机仍在运动时锁定旧位置并立即产生反向满幅电流。
- Pitch 与 Yaw 分别建立目标和授权状态。Yaw 等待稳定或 DM4310 离线时只把 Yaw
  电流强制为零，不阻塞健康的 Pitch 根据 BMI088 零点运行；任一轴离线也不会再
  清除另一轴已经建立的目标。
- D-BUS Failsafe/丢帧当帧撤权；遥控超时、CAN故障或控制超期调用
  `CanMotorBus_StopAll()`。
- GM6020、DM4310 或 BMI088 不新鲜时调用 `CanMotorBus_StopGimbal()`，只停云台，
  不覆盖独立发射机构目标。
- 掉线时清除云台PID、编码器展开和旧目标；恢复后以当前位置无扰重建。
- `osDelayUntil()`若发现deadline已过期，会重置调度基准并先执行安全停机，
  不连续补跑遗漏的1 ms周期。

## 发射机构任务

S2 用 D-BUS 三档离散值控制摩擦轮转速：

```text
开关值 1 -> 停止
开关值 2 -> 低速
开关值 3 -> 最高速
```

S2=1 时两颗 M3508 停止，S2=2 使用低速目标，S2=3 使用最高速目标。两颗 M3508
只要至少一颗在线即可运行，ID2 和 ID3 的方向分别由
`LAUNCH_M3508_ID2_DIRECTION`、`LAUNCH_M3508_ID3_DIRECTION` 设置。任意一颗
掉线都会同时停止两颗电机。

两颗 M3508 速度环按"0→6000 rpm 阶跃 ≤150 ms（从接到指令到稳态）"整定：
KP=18、KI=20、KD=100（每拍差量语义，等效微分时间约 5.6 ms）、速度低通
alpha=0.50、积分分离 1000 rpm。3508.csv 实测旧参数为 210 ms 且过冲 +5.4%；
辨识结果是被控对象 k_i=4.98 rpm/s per 电流单位、电调命令→力矩约 8~9 ms 滞后、
过冲≈上升加速度×低通滞后，故对策是去低通滞后并用 KD 做超前补偿。仿真在
延时 6/8/10 ms 与负载 300/400/600 共 9 工况下最坏 ±1% 稳态 136 ms、±0.5%
稳态 141 ms、过冲 +0.21%。详细依据见 `config.h` 的 M3508 段注释。

S1 控制 M2006 ID5：S1=1 角度环保持当前位置；S1=2 连发（PLL 锁相，名义
4800 rpm≈20 Hz）；S1=3 单动（角度环，每次从 1 拨到 3 触发一步 40° 输出，
电机转 4 圈）。M2006 离线时不会释放启动信号量。

三颗发射电机都有独立的速度 PID、速度低通、积分限幅、输出限幅和方向配置；
M2006 额外有角度环（编码器在电机轴，拨盘在 P36 输出端，36:1 减速）。速度环
按相位裕度整定：KP=12、LPF alpha=0.7、输出限幅 6500、到位死区 0.8°。
原 KP=24/alpha=0.35 的穿越频率 80 Hz、相位裕度只剩约 6°（计电调+CAN 反馈
约 2 ms 延时为负），连发巡航段 I6 有 47.6% 采样顶在 ±6500、约每 2.5 ms 反向
一次、I5 抖 ±1000 rpm；改成 KP=12/alpha=0.7 后仿真电流饱和占比 1.5%→0.2%、
反向 41 Hz→1 Hz、转速脉动 std 88→30 rpm。注意 alpha 与直觉相反：低通滞后
本身就是振荡来源，越小相位裕度越低。
单动角度环按 2006.csv 实测重整定：KP=85→300、限速 3400→4200 rpm，实机单发
40° 进死区已由实测 275~320 ms 降到 85~130 ms（入死区速度更低、过冲 ≤0.3°）。
整定依据与物理下限（6.5 A 时约 76 ms，50 ms 需 >13 A）写在 `config.h` 对应宏上方。
连发 PLL 当前 KP=60、速度上限 5600 rpm，用于补足 2006.csv 中相位长期落后的跟踪余量。

## VOFA 与在线调参

UART4 使用 115200 波特率和 RX/TX DMA。`vofa` 任务以绝对节拍每 5 ms 发送 8 个
JustFloat 通道，帧尾为 `00 00 80 7F`。

`VOFA_IMU_AXIS_DEBUG_MODE=1` 时用于 BMI088 三轴方向确认，通道含义为：

| VOFA 通道 | IMU 轴向诊断内容 |
|---|---|
| 1 | Pitch 目标角，° |
| 2 | Pitch 编码器实际角，° |
| 3 | BMI088 原始 X 轴角速度，°/s |
| 4 | BMI088 原始 Y 轴角速度，°/s |
| 5 | BMI088 原始 Z 轴角速度，°/s |
| 6 | 当前映射后的 Roll 角速度，°/s |
| 7 | 当前映射后的 Pitch 角速度，°/s |
| 8 | 当前映射后的 Yaw 角速度，°/s |

`VOFA_IMU_AXIS_DEBUG_MODE=0`、`VOFA_PITCH_TUNING_MODE=0` 且
`VOFA_YAW_TUNING_MODE=0` 时，默认综合状态页通道
含义为：

| VOFA 通道 | 默认综合状态内容 |
|---|---|
| 1 | Pitch 目标角，° |
| 2 | Pitch 实际角，° |
| 3 | Yaw 目标角，° |
| 4 | Yaw 实际角，° |
| 5 | M2006 目标输出角，°；连发模式下每 50 ms 按 40° 阶梯递增 |
| 6 | M2006 实际输出角，°；允许非整数 |
| 7 | M3508 ID2 实测转速，rpm |
| 8 | M3508 ID3 实测转速，rpm |

`VOFA_IMU_AXIS_DEBUG_MODE=0` 且 `VOFA_PITCH_TUNING_MODE=1` 时，通道 3~8 改为
Pitch 调参页：Pitch 轨迹角、速度目标、IMU 实际速度、速度 PID 输出、电机坐标系总前馈、
最终 CAN 电压命令。

`VOFA_IMU_AXIS_DEBUG_MODE=0` 且 `VOFA_YAW_TUNING_MODE=1` 时，8 路通道改为 Yaw
调参页：Yaw 目标角、编码器实际角、IMU 实际角、轨迹角、速度目标、编码器实际速度、
轨迹速度前馈、最终 CAN 电流命令。采集 `pitch.csv` 时同样按 I0~I7 保存，便于先看
目标/实际/轨迹/速度/输出之间的相位和限幅关系。

`YAW_SYSID_MODE=1` 辨识固件时，通道 7/8 改为：7=给 DM4310 的电流指令、8=yaw 原始
速度(rpm)，且仅在辨识运行期间打印，运行结束自动静默。

`VOFA_LAUNCH_TUNING_MODE=1` 时，8 路通道改为拨盘单发/连发调参页：

| 通道 | 含义 |
|---|---|
| 1 | S1 模式：1=保持，2=连发，3=单动 |
| 2 | M2006 拨盘目标输出角，°；连发时按 40° 阶梯显示 |
| 3 | M2006 拨盘实际输出角，° |
| 4 | M2006 拨盘角度误差，° |
| 5 | M2006 目标电机转速，rpm |
| 6 | M2006 滤波实际电机转速，rpm |
| 7 | M2006 最终 C610 电流命令 |
| 8 | M2006 实际累计发数，输出角/40° 取整累计 |

`VOFA_LAUNCH_M3508_TUNING_MODE=1` 时（与上面三个调参页互斥，`vofa.c` 是 `#elif`
链，同一时刻只能开一个；**当前该宏为 0，走默认综合页**），8 路通道改为摩擦轮
M3508 阶跃调参页，用于整定“遥控 S2 从 1 拨到 3、目标 0→6000 rpm 阶跃在 150 ms
内完成”（该页已完成整定，参数见 `config.h` 的 M3508 段）：

| 通道 | 含义 |
|---|---|
| 1 | 固件侧时间轴，ms（自 VOFA 初始化起算）|
| 2 | M3508 ID2 目标转速，rpm（S2=3 → +6000）|
| 3 | M3508 ID2 原始反馈转速，rpm（C620 上报）|
| 4 | M3508 ID2 滤波后转速，rpm（速度环实际用的反馈）|
| 5 | M3508 ID2 最终 CAN 电流命令（含使能/标定模式门控后的真实下发值）|
| 6 | M3508 ID2 实际转矩电流反馈（C620 上报，与命令同量纲 ±16384）|
| 7 | M3508 ID3 原始反馈转速，rpm |
| 8 | M3508 ID3 最终 CAN 电流命令 |

该页直接只读电机与总线状态（`can1_m3508_id2/3`、`CanMotorBus_GetStatus`），
不调用 PID、不写任何控制量。**通道 1 是固件侧时间轴，不要用上位机行号×5 ms 推时间**：
`2006.csv` 证明 VOFA 任务的实际发送间隔有 ±1 ms 抖动，用 5 ms 差分算出来的速度
会出现物理上不可能的值。判读口径（按上表 1~8）：通道 4 与 3 的差 = 速度低通滞后代价；
通道 5 长期贴 ±16384 = 转矩/电流受限而非增益问题；通道 6 明显小于 5 = C620 内部
限流或反电动势压顶；通道 3 到位后有静差 = 积分（含积分分离）没补上；通道 7/8 是
另一颗轮子，用于确认镜像方向与两颗轮子一致性。

若单次发送或调度导致 deadline 已过期，任务会以当前 tick 重建下一帧的绝对节拍，
不会连续补发历史帧。

UART4 命令接收保持开启（无换行时按接收空闲自动结束一条命令）。支持以下 ASCII 命令：

**在线调参（`键=值`，大写无空格，值 0~100000）**

- `PITCH_KP_POS` / `PITCH_KI_POS` / `PITCH_KD_POS`：Pitch 位置环
- `PITCH_KP_SPD` / `PITCH_KI_SPD` / `PITCH_KD_SPD`：Pitch 速度环
- `YAW_KP_POS` / `YAW_KI_POS` / `YAW_KD_POS`：Yaw 位置环
- `YAW_KP_SPD` / `YAW_KI_SPD` / `YAW_KD_SPD`：Yaw 速度环
- `C1_GR` / `PITCH_GRAVITY_C1_GR`：Pitch 重力前馈一次项系数

**调试命令**

- `YAWTEST=电流`：Yaw 方向测试，输出固定电流（幅值限 `YAW_DIRECTION_TEST_MAX_CURRENT`，
  持续 `YAW_DIRECTION_TEST_DURATION_MS`）。
- `identify_on`：仅 `YAW_SYSID_MODE=1` 时有效，触发一次 yaw 线性扫频正弦辨识。

每条调参命令只修改指定轴、指定环路。旧的无轴名命令（例如 `KP_POS=...`）会被拒绝。
在线命令只修改运行时 PID 或 `C1_gr`，`config.h` 中的宏仍作为下次复位后的初始值。

## 调试/操作宏

以下宏集中在 `Tasks/Inc/config.h`，改后需重新编译烧录：

| 宏 | 取值 | 作用 |
|---|---|---|
| `YAW_SYSID_MODE` | 0/1 | 0=正常闭环；1=yaw 系统辨识固件（pitch 不输出、yaw 直通正弦扫频，`identify_on` 触发，I6=电流指令、I7=原始速度） |
| `PITCH_GRAVITY_ONLY_ENABLE` | 0/1 | 0=正常 Pitch 位置/速度闭环；1=仅重力前馈（位置环旁路、速度目标=0），用于单独标定重力前馈，完成后必须改回 0 |
| `YAW_COMMISSIONING_MODE` | 0/1 | 0=双轴+发射正常；1=仅调试 yaw（pitch 与发射停发命令） |
| `LAUNCH_MOTOR_OUTPUT_ENABLE` | 0/1 | 发射机构（M3508/M2006）是否允许输出；调试云台时设 0 防误启动 |
| `YAW_CLOSED_LOOP_ENABLE` | 0/1 | 0=Yaw 开环（配合 `YAWTEST` 方向测试）；1=Yaw 位置闭环 |
| `YAW_HOME_TO_IMU_ZERO_ON_AUTHORIZE` | 0/1 | 0=授权时保持当前 Yaw；1=授权/遥控重连时自动回 BMI Yaw 零点 |
| `PITCH_HOME_TO_POWER_ON_POSITION` | 0/1 | 0=Pitch 授权时回 IMU 水平零点；1=以上电位置为零点并保持当前位置 |
| `PITCH_SPEED_LIMIT_ENABLE` | 0/1 | 0=放开 Pitch 轨迹速度/加速度和位置环速度目标限幅；1=使用 `PITCH_MAX_SPEED_RPM` 和 `PITCH_TRAJECTORY_*` |

Yaw 系统辨识参数：`YAW_SYSID_AMPLITUDE_CURRENT`（扫频幅值）、
`YAW_SYSID_FREQ_START_HZ`、`YAW_SYSID_FREQ_END_HZ`、`YAW_SYSID_DURATION_MS`（单次时长）。

Pitch 重力前馈参数：`PITCH_GRAVITY_FIT_MIN_DEG`、`PITCH_GRAVITY_FIT_MAX_DEG`、
`PITCH_GRAVITY_POLY_C0`、`PITCH_GRAVITY_C1_GR`、`PITCH_GRAVITY_POLY_C2`、
`PITCH_GRAVITY_POLY_C3`、`PITCH_GRAVITY_POLY_C4`。
`PITCH_GRAVITY_C1_GR` 是上电默认值，运行时可在 Watch 里修改全局变量 `C1_gr`，
也可用在线命令 `C1_GR=数值` 修改。
标定区间外按端点值补偿，不做多项式外推。

Pitch 机械限位：`PITCH_LIMIT_MIN_RAD`（最高，IMU -139°）、
`PITCH_LIMIT_MAX_RAD`（最低，IMU -64°），目标直接钳位到该区间。

## 集中参数配置

所有需要根据实车调整的任务参数集中在 `Tasks/Inc/config.h`，包括：

- 任务周期、遥控器失联时间和控制保护时间。
- D-BUS 通道、输入范围、云台步进角度和发射机构最大转速。
- BMI088 安装轴映射、方向和互补滤波系数。
- Pitch/Yaw 的位置 PID、速度 PID、速度/电流限幅、软限位、方向和重力前馈。
- GM6020、两颗 M3508、M2006 的速度低通系数。
- 三颗发射电机各自的 KP/KI/KD、积分限幅、输出限幅和方向。

首次调试应悬空或拆除负载，从较小 KP 开始，暂时把 KI、KD 和重力前馈设为
0，确认电机反馈方向正确后再逐项增加。
