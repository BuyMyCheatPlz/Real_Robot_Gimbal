# 实车云台控制工程

基于 STM32F405RGTx、STM32 HAL、FreeRTOS/CMSIS-RTOS2 的双轴云台与发射机构控制工程。
工程包含 CAN 电机驱动、BMI088 姿态解算、大疆 D-BUS 遥控（DR16/DT7 接收器）、双环 PID、重力与
速度前馈、VOFA JustFloat 波形输出、串口在线调参以及电机和遥控器失联保护。

## 0. Pitch 调参 / 问题修复记录（近期，重要）

### 0.1 抖动治理（保持段 ~3.3~3.6 Hz 极限环）
- 关闭 GM6020“起步助推”bang-bang 继电器：`PITCH_STARTUP_MIN_VOLTAGE` 8000→**0**。
  该继电器会把小速度指令强制成 ±8000 满力矩来回切换，是“怎么调 PID 都抖”的主因。
- 速度环积分分离 70→**5**，减少全带积分带来的相位滞后。
- `pitch.csv` 佐证：抖动为固定频率极限环、与增益无关。

### 0.2 D 项语义
- 本工程速度环和位置环 D 项均为 `kd*(e[k]-e[k-1])`（每 1 ms 拍误差差量，**不除以 dt**）；
  原实现 `-(meas-prev)/dt` 会把同数值放大 ~1000 倍 → 一加 D 就满幅抖振。
- `MotorSpeedPid`（速度环）与 `PositionPid_t`（位置环）均已改为每拍差量语义。
- 数值：`PITCH_SPEED_KD=0`；`PITCH_ANGLE_KD=100`（保留轻阻尼，避免阶跃时重刹车）；
  `YAW_ANGLE_KD` 0.10→**100**（纯等值重标定，yaw 行为不变）。

### 0.3 遥控
- Pitch 方向宏 `PITCH_STICK_DIR=-1`（`data_process.c` 使用；向上打 pitch 向下 → 取反）。
- 当前通道：pitch=`channel[3]`(CH4)、yaw=`channel[0]`(CH1)，摇杆越死区一次 ±30°。

### 0.4 yaw–pitch 解耦（yaw 转动让 pitch 误动的修正）
- 旧配置曾把 Pitch 速度环接到 BMI raw Z；最新三轴诊断确认真实 Pitch 主轴是 raw X，
  方向与编码器实际角同向。控制链路沿用历史变量名 `Roll` 作为本机 Pitch 物理轴，
  因此闭环时应让 `IMU_GYRO_ROLL_AXIS` 指向 raw X。
- **方案：只在收到 yaw 遥控指令后锁存 pitch**（两轴不会同时指令）：
  `PITCH_LATCH_AFTER_YAW_CMD_MS=500`、`PITCH_LATCH_YAW_SETTLED_DEG=0.5`；
  GM6020 新增 `output_hold/held_output/last_output`；失权/故障/复位自动清锁存。
- 备选：`PITCH_ROLL_YAW_CROSS_RATE=0`，在线命令 `PITCH_YAW_CROSS=±x` 做陀螺通道串扰补偿。

### 0.5 Pitch 大负载防超调基线
- 速度 PID 现在按“最终输出限幅减去前馈”得到动态上下限；重力前馈占用输出余量时，
  积分器能够看到真正的饱和边界，不再发生二次裁剪导致的积分饱和累积。
- `PITCH_SPEED_LIMIT_ENABLE=0` 时放开 Pitch 阶跃轨迹速度/加速度限幅和位置环速度目标限幅；
  若需要回到保守整定，再设为 1 使用 `1.2 rad/s、8 rad/s²` 轨迹和 `70°/s` 速度限幅。
- 限速开关打开时，可在线修改 `PITCH_TRAJ_SPEED`、`PITCH_TRAJ_ACCEL`、`PITCH_MAX_SPEED`、
  无需反复编译；Pitch 当前不再叠加轨迹速度前馈和加速度前馈。

### 0.6 现场数据结论（`pitch.csv`）与下一步
- 最近完整阶跃中，`0→-30°` 峰值约 `-43.8°`，`-30→0°` 峰值约 `+24.8°`；
  由位置差分得到的峰值速度约 `266/307°/s`，远高于当时 `90°/s` 的速度目标。
- 因此先处理速度闭环、最终输出饱和和重力前馈，不用外环大 D 掩盖内环问题。
- `VOFA_IMU_AXIS_DEBUG_MODE=1` 时 8 通道依次为：Pitch 目标角、Pitch 实际角、BMI088 原始
  X/Y/Z 角速度、当前映射后的 Roll/Pitch/Yaw 角速度。确认方向后把该宏改回 0，
  `VOFA_PITCH_TUNING_MODE=0` 会恢复默认综合状态页：Pitch/Yaw 目标实际角、M2006
  目标实际输出角、两颗 M3508 实际速度。
- `PITCH_GRAVITY_ONLY_ENABLE=1` 用于单独标定重力前馈：Pitch 位置/速度闭环旁路、
  速度目标清零，只看 I6/I7 的前馈输出；标定完成后改回 0。
- 若 +30° 方向实测到不了 ~30°（停在 ~26° 附近），优先检查该方向机械限位/线束干涉。

### 0.7 发射机构（M2006）
- VOFA I4/I5 改为打印 **M2006 拨盘目标/实际输出角度曲线(°)**
  （`vofa.c` 用 `m2006_target_deg/m2006_actual_deg`，不再打印发弹数）。
- 目标角度阶梯（`launch.c`）：单发 S1:1→3 触发 +40°；**连发 S1=2 时角度环作为锁相环**——
  目标相位按 40°/50ms(=800°/s)连续推进，速度指令 = 基准 4800rpm + 相位误差×
  `LAUNCH_M2006_ID5_CONT_PLL_KP_RPM_PER_DEG(60)`，I4 按 40° 取整成阶梯，I5 为实际曲线。
- M2006 当前按 2006.csv 抑制抖动：速度环 KP=24、KI=1.2、输出限幅 6500、
  速度反馈 LPF alpha=0.35；单发角度 KP=85、限速 3400 rpm、到位死区 0.8°，
  松开 S1 后继续追完本次 +40° 目标；连发 PLL KP=60、追相位上限 5600 rpm。

## 项目已实现功能

### 1. 硬件驱动

- 同时运行 CAN1、CAN2 两条 1 Mbps CAN 总线。
- 接收并解析两颗 M3508、一颗 GM6020、一颗 M2006 和一颗 DM4310 的反馈。
- 统一生成 DJI 电机控制帧，并按 DM4310 电流协议发送 `0x3FE` 聚合控制帧。
- 通过 SPI1 DMA 连续读取 BMI088 的三轴加速度、三轴角速度和温度。
- 通过 BMI088 数据就绪引脚触发采样，减少任务轮询和阻塞时间。
- BMI088 首次同步读取完成后才开放 DRDY；DMA 分段等待双流就绪，并带 5 ms
  超时 abort 恢复。
- 通过 USART2 DMA + 空闲中断接收大疆 D-BUS 遥控数据。
- 通过 UART4 RX/TX DMA 同时完成 VOFA 输出和在线调参命令接收。

### 2. 云台控制

- Pitch 位置环将 GM6020 编码器展开增量换算到本机 Pitch 坐标作为实际值，速度环使用
  映射到 `roll_rate_rad_s` 的 BMI088 角速度作为反馈（直接测云台真实角速度，不受减速/背隙/柔性影响）。
- Pitch 编码器反馈经过圈数展开和一阶低通滤波，可跨越编码器零点连续计算角度。
- 上电/遥控重连后通过首次映射到 `roll_rad` 的 BMI088 姿态给 GM6020 编码器建立角度零偏。
- Pitch 默认回到 IMU 水平零点，而不是回电机编码器机械零点；也可通过
  `PITCH_HOME_TO_POWER_ON_POSITION=1` 改为以上电位置为零点并保持当前位置。
- 使用连续 Pitch 编码器角计算 3 阶标定重力电压前馈，降低云台负载对 PID 的影响，并避免 IMU 融合角滞后进入前馈。
- Pitch 目标直接钳位到实测机械限位（`PITCH_LIMIT_MIN_RAD`/`PITCH_LIMIT_MAX_RAD`，
  IMU Pitch 为 -139°（最高）至 -64°（最低））。
- Yaw 使用 DM4310 编码器作为实际位置，不使用会漂移的 IMU Yaw 作为闭环反馈。
- Yaw 授权/遥控重连时回到 BMI088 Yaw 测量零点（`YAW_HOME_TO_IMU_ZERO_ON_AUTHORIZE=1`）；
  设为 0 则改为保持当前编码器位置。
- Yaw 外位置 PID 叠加目标轨迹速度前馈，MCU 软件速度 PID 生成 DM4310 电流命令；
  当前 200 ms 阶跃参数为轨迹 8.8 rad/s、87 rad/s²，位置环 KP=5.2、KI=0.18、KD=88，
  速度环 KP=190、KI=15，速度反馈 LPF alpha=0.40，速度前馈 0.89，目标附近最小电流 80，hold 进入误差 0.05°，用于压低部分位置 240 ms 附近的过目标超调。
- Pitch 当前默认放开目标速度限幅，仍保留位置积分限幅、电机输出限幅和角度软限位；
  Yaw 保持目标速度、电流输出和角度软限位。

### 3. 发射机构控制

- S1 三档控制 M2006 拨弹（电机轴带 1:36 减速箱）：
  1 保持、2 连发（纯速度环 4800 rpm≈20 Hz）、3 单动（从 1 拨到 3 触发一次 40°输出）。
- 两颗 M3508 使用独立速度 PID、独立低通滤波和独立输出限幅。
- 两颗 M3508 默认反向旋转，适用于左右摩擦轮结构。
- 只要相应发射组中至少一台 M3508 在线，且遥控新鲜，便允许该组运行；离线电机单独停机。
- S2 三档控制两颗 M3508 摩擦轮的目标转速。
- M2006 为输出轴角度外环 + 电机速度内环双环 PID（按 1:36 减速换算），带积分限幅、转速限幅、低通与失联/离线保护。
- 步进角度 40°、连发频率 20 Hz、减速比、环参数均在 `config.h` 中集中配置。

### 4. 遥控与交互

- CH4 作为 Pitch 摇杆，越过死区边沿时生成一次 ±30° 目标步进。
- CH1 作为 Yaw 摇杆，越过死区边沿时生成一次 ±30° 目标步进。
- 通道编号、死区和单次角度增量均可在 `config.h` 中修改。
- VOFA 当前默认显示 Pitch 目标/实际角、轨迹、速度、PID 输出、前馈和最终 CAN 输出。
- 支持 UART4 在线修改 Pitch/Yaw 位置环、速度环 KP/KI/KD 和 Pitch 重力前馈 `C1_gr`。

### 5. 安全保护

- 遥控器未上线时不允许电机产生有效输出。
- D-BUS 进入 Failsafe、丢帧或超过设定时间未更新时停止全部电机。
- CAN 电机反馈超时后自动标记为离线。
- CAN 发送连续失败或 bus-off 时锁存全零输出；连续发送 10 组零帧成功后恢复。
- 云台传感器故障只停止 Pitch/Yaw，不再清除独立发射机构目标。
- 两个发射启动信号量上电时均为不可用状态，避免任务启动瞬间误转；只要对应发射组存在在线电机即可放行。
- 电机离线时对应速度 PID 不输出控制量。
- 队列满时丢弃旧数据，优先保留最新控制状态。
- 各控制环均具备积分限幅和输出限幅，降低积分饱和及过流风险。

## 当前运行效果

在接线、方向和 PID 参数正确的前提下，工程能够实现以下效果：

| 操作或状态 | 运行效果 |
|---|---|
| 系统上电，遥控器未连接 | 所有电机保持停止，不执行云台或发射动作 |
| 遥控器上线 | Pitch 默认回到 IMU 水平零点，Yaw 回到 BMI Yaw 零点 |
| CH4 摇杆越过死区边沿 | Pitch 在当前目标基础上步进 ±30°并闭环保持 |
| CH1 摇杆越过死区边沿 | Yaw 在当前目标基础上步进 ±30°并闭环保持 |
| S1=2 | M2006 连发：纯速度环 4800 rpm（约 20 Hz） |
| S1=3 | M2006 单动：从 S1=1 拨到 3 触发一次 40°输出后保持 |
| 任意一颗 M3508 离线 | 离线电机停止，在线电机仍按 S2 目标运行 |
| S2=3 | 两颗 M3508 反向同步运行，达到配置的最大转速 |
| M2006 离线 | 拨弹停止（角度/电流归零） |
| 遥控器失联或 Failsafe | 云台和发射机构全部停止输出 |
| 连接 VOFA | 实时显示 Pitch/Yaw 目标和实际角度，可观察超调、振荡和稳态误差 |
| UART4 发送 PID 命令 | 新参数在下一个控制周期生效，无需重新编译和下载 |

以上是软件已经具备的控制行为。实际跟随速度、超调量、稳态误差、抗扰能力和
重力补偿效果取决于机构惯量、减速比、重心、摩擦、电机方向、BMI088 安装方向
以及 `config.h` 中的实车参数。首次装车必须在机构受支撑、低输出限幅的条件下
完成方向确认和 PID 整定，不能直接使用默认参数进行满功率带载测试。

## 一、硬件组成

| 总线 | 设备 | 配置 |
|---|---|---|
| CAN1 | M3508 + C620 | 电机 ID 2、ID 3，发射摩擦轮 |
| CAN1 | GM6020 | 电机 ID 2，反馈 `0x206`、控制帧 `0x1FF` 的 ID2 槽（DATA[2]～DATA[3]），Pitch 云台 |
| CAN2 | M2006 + C610 | 电机 ID 5，拨弹机构 |
| CAN2 | DM4310 电流控制版 | 电机 ID 1，控制帧 `0x3FE`，反馈帧 `0x301`，Yaw 云台 |
| USART2 | 大疆 DR16 / DT7 接收器 | D-BUS，100000 波特率、8E2、DMA + 空闲中断 |
| UART4 | VOFA 上位机 | 115200 波特率、RX/TX DMA |
| SPI1 | BMI088 | 标准 SPI、TX/RX DMA、数据就绪外部中断 |

CAN1 和 CAN2 的时序均为 Prescaler=3、BS1=10TQ、BS2=3TQ。在当前 42 MHz
APB1 CAN 时钟下，CAN 波特率为 1 Mbps。

## 二、引脚定义

| 功能 | STM32 引脚 |
|---|---|
| CAN1 RX / TX | PA11 / PA12 |
| CAN2 RX / TX | PB12 / PB13 |
| USART2 TX / RX | PA2 / PA3 |
| UART4 TX / RX | PC10 / PC11 |
| SPI1 SCK / MISO / MOSI | PA5 / PA6 / PA7 |
| BMI088 陀螺仪 CS | PA4，`CS_Gyro` |
| BMI088 加速度计 CS | PC4，`CS_Accel` |
| BMI088 陀螺仪 INT3 | PC5，`INT_Gyro` |
| BMI088 加速度计 INT1 | PB0，`INT_Accel` |
| SWDIO / SWCLK | PA13 / PA14 |

STM32F405 串口不支持硬件 RX 极性反转。D-BUS 采用标准串口协议，因此 PA3 必须
接收接收器输出的正确串口信号；如硬件线上有反相/电平问题，需在 PA3 前端按实际接线修正。

## 三、软件结构

```text
Real_Robot_Gimbal/
├─ Core/                    CubeMX 生成的外设、IRQ 和 FreeRTOS 初始化
├─ Hardware_Drivers/        CAN 电机、D-BUS、BMI088 驱动
├─ Tasks/
│  ├─ Inc/config.h          全部实车调节参数的集中入口
│  ├─ Inc/gimbal_control.h  队列消息和任务公共类型
│  └─ Src/
│     ├─ data_process.c     姿态、遥控、命令和发射参数处理
│     ├─ pid_calc.c         Pitch/Yaw 云台控制
│     ├─ launch.c           M3508/M2006 发射机构速度控制
│     └─ vofa.c             JustFloat 发送和 UART4 命令接收
├─ MDK-ARM/                 Keil MDK 工程
└─ Real_Robot_Gimbal.ioc    STM32CubeMX 工程
```

## 四、FreeRTOS 任务与通信

| 任务 | 默认周期/触发方式 | 功能 |
|---|---|---|
| `Data_Process` | 1 ms | BMI088 姿态解算、D-BUS 处理、在线命令解析、队列发布 |
| `PID_calc` | 1 ms | Pitch/Yaw 闭环、反馈新鲜度、CAN故障仲裁和超期保护 |
| `Launch_Task` | 最长等待 2 ms | 两颗 M3508 和 M2006 的独立速度 PID 与在线保护 |
| `VOFA_print` | 5 ms 绝对周期 | UART4 DMA 发送 8 通道 JustFloat |
| `defaultTask` | 1 ms 延时 | CubeMX 默认空闲任务 |

任务间使用以下 RTOS 对象：

- `Target_Angle`：传输 BMI088 姿态和 Pitch/Yaw 单次角度增量。
- `Update_PID_para`：传输 UART4 在线 PID 参数。
- `Update_launch_para`：传输 CH4/CH5 映射后的发射目标速度和在线状态。
- `wake_launch`：M3508 组中至少一台在线时允许摩擦轮转动。
- `wake_launch_motor`：M2006 在线时允许拨弹电机转动。

两个启动信号量上电时均为不可用状态，避免任务创建后直接启动电机。

## 五、BMI088 与姿态解算

陀螺仪 INT3 以 1 kHz 触发一次完整 DMA 采样链：

```text
加速度计 → 陀螺仪 → 温度 → 完整样本就绪
```

初始化期间先屏蔽并清除 BMI088 EXTI，完成同步首读后才打开陀螺仪 DRDY。
DMA 接收完成时仅在 SPI、RX DMA、TX DMA 全部就绪后推进下一段；否则由
`Data_Process` 延后续传。单段超过 5 ms 会 abort，并等待下一次 DRDY 重启。

`Data_Process` 对完整样本进行原子快照，使用加速度计和陀螺仪互补滤波计算
Roll/Pitch，并积分映射后的 Yaw 角速度得到相对 Yaw。BMI088 没有磁力计，因此
IMU Yaw 会随时间漂移；实际 Yaw 闭环使用 DM4310 编码器，不依赖 IMU Yaw。

BMI088 的安装轴、轴符号和滤波权重均可在 `Tasks/Inc/config.h` 中修改。

## 六、云台控制

### Pitch：GM6020

- 使用 GM6020 编码器展开角度并进行低通滤波。
- 外层位置 PID 输出目标转速；`PITCH_POSITION_DEADZONE_RAD` 当前仅为预留参数，
  未接入控制链，避免 0.5° 死区破坏 MISSION 要求的 ±0.2°保持精度。
- 位置环 D 通道串联 23.7 Hz 与 32 Hz 陷波器，抑制重载机构在 -60° 附近的窄带共振；
  P/I、速度环和重力前馈不经过该滤波，低频阶跃与稳态位置精度保持不变。
- 低 Pitch 角度区位置 D 通过 `PITCH_POSITION_D_LOW_ANGLE_SCALE` 做可调衰减；
  默认 -45° 开始线性过渡，-55° 以下按该比例保留 D 项，P/I、速度环和前馈不变。
- Pitch 目标变化超过 `PITCH_POSITION_STEP_RESET_RAD` 时会重置位置环 I/D 记忆；
  位置环积分只在 `PITCH_ANGLE_INTEGRAL_SEPARATION_RAD` 内启用，避免大阶跃积分残留导致超调。
- `PITCH_APPROACH_SPEED_LIMIT_ENABLE=1` 时，位置环速度目标按剩余误差动态限速；
  远处仍能快速阶跃，接近目标时按刹车加速度提前收速，避免继续加 D 导致抖动。
  低 Pitch 角度区可通过 `PITCH_APPROACH_LOW_ANGLE_SCALE` 单独提前刹车，专门处理 -60° 超调。
- 内层速度 PID 输出 GM6020 电压命令，速度反馈用映射到 `roll_rate_rad_s` 的 BMI088 角速度；本机 Pitch 主轴经三轴诊断确认为 raw X。
- 根据连续 Pitch 编码器角计算 3 阶标定重力电压前馈，标定区间外按端点补偿。
- 上电后用首次有效、映射到 `roll_rad` 的 BMI088 姿态角给编码器建立零偏；
  默认目标为 IMU 水平零点，`PITCH_HOME_TO_POWER_ON_POSITION=1` 时改为保持上电位置。
- `PITCH_SPEED_LIMIT_ENABLE=0` 时不限制阶跃轨迹速度/加速度和位置环速度目标；
  仍保留积分、输出保护，以及实测机械限位钳位（IMU Pitch -139°~-64°）。

### Yaw：DM4310

- 解析 `0x300 + 电机 ID` 反馈：0～8191 单圈位置、速度原始值除以 100 rpm、
  扭矩电流 mA、绕组温度和 PCB 温度。
- 编码器跨零展开为连续位置实际值并进行低通滤波。
- 外层位置 PID 叠加斜坡目标生成的速度前馈。
- MCU 内层速度 PID 生成电流控制量；DM4310 的每个 16 位命令槽采用小端格式
  （低 8 位在前），与 DJI 电机的大端命令格式不同。电流命令限制为
  `-20`～`20`，ID 1～4 使用 `0x3FE`，ID 5～8 使用 `0x4FE`。
- 授权/遥控重连时回到 BMI088 Yaw 测量零点（`YAW_HOME_TO_IMU_ZERO_ON_AUTHORIZE=1`）。
- 使用最大速度、速度环积分、电流输出和角度软限位保护。此前命令 `20` 按大端
  发成 `00 14`，被 DM4310 按小端解释为 `0x1400=5120`，这是剧烈振荡的根因。
  修复后 `20` 为 `14 00`，驱动层、闭环和方向点动入口均统一限制为
  `-20`～`20`。

DM4310 反馈频率由固件固定为 1000 Hz；更换电机固件后必须重新确认协议。

## 七、遥控器功能

D-BUS 通道在代码中使用零基数组下标，`config.h` 当前将 Pitch/Yaw 控制对应到
CH4/CH1。摇杆越过死区边沿时生成一次 30° 步进：

| 通道 | 功能 |
|---|---|
| S1 | M2006 拨弹：1 保持，2 连发(4800 rpm≈20 Hz)，3 单动(1→3 触发 40°) |
| S2 | 两颗 M3508 目标转速：1 停止，2 低速，3 为最高速 |
| CH4 | Pitch 摇杆越过死区边沿后，目标步进 ±30° |
| CH1 | Yaw 摇杆越过死区边沿后，目标步进 ±30° |

S2 使用三档离散映射控制摩擦轮转速：

```text
开关值 1 -> 停止
开关值 2 -> 低速
开关值 3 -> 最高速
```

修改以下宏即可改变摩擦轮最高速：

```c
#define LAUNCH_M3508_TARGET_MAX_SPEED_RPM 6000.0f
```

M2006 拨弹已改为角度步进控制（不是目标转速）：步进角度 40°、连发频率 20 Hz、
减速比与角度/速度环参数见 `config.h` 的 `M2006_*` / `LAUNCH_M2006_*` 宏。

## 八、发射机构控制（M3508 速度环 + M2006 角度-速度双环）

两颗 M3508 各自具有独立的：

- KP、KI、KD。
- PID 积分状态。
- 速度反馈低通系数。
- 积分限幅和输出限幅。
- 电机方向宏。

M2006(ID5) 输出轴带 1:36 减速箱，采用输出轴角度外环 + 电机速度内环双环：
- 编码器在电机轴，输出角度 = 电机轴角度/36；
- S1=2 连发（纯速度环 4800 rpm），S1=3 单动步进 40°；
- 角度/速度 PID、积分与输出限幅、方向宏集中在 `config.h`。

M3508 发射组中至少一颗电机在线即可运行，默认方向相反；离线电机单独停机。
M2006 只有在线时才会运行。Launch 任务还会独立检查 100 ms 遥控数据超时。

## 九、VOFA JustFloat 与在线调参

UART4 每 5 ms 按绝对周期发送 8 个小端 float，随后发送帧尾 `00 00 80 7F`。当前重新打开
`VOFA_LAUNCH_TUNING_MODE=1` 用于拨盘单发/连发调参；需要 Yaw 阶跃调参时再切到
`VOFA_YAW_TUNING_MODE=1`。关闭专用调参页后默认综合页为 Pitch/Yaw 角度、M2006 目标/实际角和 M3508 转速。完整通道映射、命令清单和调试/操作宏见
[Tasks/README.md](Tasks/README.md)。

UART4 同时接收以回车或换行结尾的 ASCII 命令：

```text
PITCH_KP_POS=90
PITCH_KI_POS=0
PITCH_KD_POS=2
PITCH_KP_SPD=80
PITCH_KI_SPD=8
PITCH_KD_SPD=0
YAW_KP_POS=0.35
YAW_KP_SPD=1
```

`Data_Process` 校验命令后通过 `Update_PID_para` 队列下发，`PID_calc` 在下一个
控制周期更新运行时 PID。命令必须包含 `PITCH_` 或 `YAW_` 轴名，避免误改另一轴；
`config.h` 中的宏仍是下次复位后的初始值。

## 十、集中参数配置

实车调试主要修改 [Tasks/Inc/config.h](Tasks/Inc/config.h)：

- 任务周期、遥控超时和安全时间。
- 遥控通道下标、输入范围、云台单次步进角度。
- M3508 和 M2006 最大目标转速。
- BMI088 安装轴、方向和姿态滤波参数。
- Pitch/Yaw 位置/速度 PID、Pitch 限速总开关、Yaw 速度限幅、积分限幅、电流输出限幅和角度软限位。
- 电机方向、重力前馈和全部速度反馈低通参数。
- 三颗发射电机各自的速度 PID 和输出保护参数。
- 调试/操作宏：`YAW_SYSID_MODE`、`PITCH_GRAVITY_ONLY_ENABLE`、`YAW_COMMISSIONING_MODE`、
  `LAUNCH_MOTOR_OUTPUT_ENABLE`、`YAW_CLOSED_LOOP_ENABLE`、`YAW_HOME_TO_IMU_ZERO_ON_AUTHORIZE`
  （详见 Tasks/README）。

修改低通系数时，`0` 表示完全不跟随新反馈，`1` 表示不滤波直接使用新反馈。

## 十一、失联与安全保护

- S.BUS Failsafe/丢帧标志当帧撤销控制权；无有效数据超过设定时间后同样停止。
- GM6020、DM4310 或 BMI088 当前反馈不新鲜时，只发送云台零命令；发射机构继续
  使用自己的遥控、反馈和超时保护。
- 云台反馈恢复后清除旧PID、旧圈数和旧目标，以当前位置重新建基准；不会追赶
  失联前目标。
- CAN发送失败会计数；连续3次失败或bus-off后锁存全零，连续10组零帧成功才解锁。
- 1 ms控制deadline过期后重置调度基准并先发送安全零命令，不补跑历史周期。
- M3508 发射组按单电机在线状态独立保护，离线电机不输出。
- M2006 离线时停止拨弹输出。
- CAN 电机反馈超过 10 ms 未更新时标记为离线（每个电机独立判定）。
- 队列满时优先丢弃旧状态，保留最新控制数据。
- 所有启动信号量初始不可用。
- Pitch 默认放开目标速度限幅但保留角度软限位；Yaw 带软件角度和速度限幅。
- 各速度 PID 均带积分和输出限幅。

## 十二、编译与使用

1. 使用 STM32CubeMX 打开 `Real_Robot_Gimbal.ioc`，检查芯片为
   STM32F405RGTx，避免随意重新分配现有 DMA 流和中断引脚。
2. 使用 Keil MDK 打开 `MDK-ARM/Real_Robot_Gimbal.uvprojx`。
3. 在 `Tasks/Inc/config.h` 中确认电机方向、IMU 安装方向、PID 和限幅。
4. 首次测试时悬空机构或拆除负载，将 KI、KD 和重力前馈暂时设为 0。
5. 先确认编码器反馈方向与目标方向一致，再逐步增加 KP、KI、KD。
6. 接通遥控器前确认 S1、S2 位于停止档（值 1）。
7. 通过 VOFA 观察目标角度与实际角度，再进行在线调参。

## 十三、重要注意事项

- `PITCH_MOTOR_SIGN` 或 `YAW_MOTOR_SIGN` 错误会形成正反馈，必须优先确认。
- 重力前馈曲线或 `C1_gr` 错误会持续推动 Pitch，首次测试建议先减小 `C1_gr` 验证方向。
- M3508、M2006 输出限幅不能超过对应电调协议允许范围。
- 修改 CH5 最大转速宏前，应确认 M2006 减速比、机构允许速度和供弹安全性。
- 在线调参不会写入 Flash，复位后恢复 `config.h` 中的编译初值。
- BMI088 的相对 Yaw 不能代替磁力计航向；本工程使用 DM4310 编码器控制 Yaw。

更细的模块说明参见：

- [硬件驱动说明](Hardware_Drivers/README.md)
- [FreeRTOS 任务说明](Tasks/README.md)
