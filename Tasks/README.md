# FreeRTOS 任务与项目效果说明

## Data_Process 数据处理任务

`Data_Process` 默认每 1 ms 运行一次，负责：

- 原子读取 BMI088 DMA 完整采样结果。
- 推进等待中的 BMI088 DMA 阶段，并在单段超时后执行任务态恢复。
- 使用加速度计和陀螺仪互补滤波计算 Roll、Pitch，并积分得到相对 Yaw。
  该 BMI088 转了 90°（Y 轴朝上），姿态角参考垂直轴 Y：pitch=atan2(-az,ay)、
  roll=atan2(-ax,ay)，轴映射见 `config.h` 的 `IMU_*`。
- 处理大疆 D-BUS 遥控数据和失联状态。
- 检测 CH6～CH9 突变，生成 Pitch/Yaw 的 ±30° 增量命令。
- 将姿态和角度增量通过 `Target_Angle` 队列发送给 `PID_calc`。
- 解析 UART4 收到的在线 PID 调参命令，并通过 `Update_PID_para` 队列下发。
- 将 S1、S2 三档映射后的发射机构目标速度通过 `Update_launch_para` 下发。

## 云台控制任务

`PID_calc` 以 1 kHz 运行：

- Pitch 位置环用 GM6020 编码器展开增量换算后的 IMU Pitch 坐标作为实际值（上电/遥控
  重连时锚定 BMI Pitch）；上电回零目标 = IMU 水平(`PITCH_GRAVITY_ZERO_RAD`=-90°)。
  位置环输出目标转速，进入 GM6020 速度 PID。
- Pitch 位置环带目标死区（`PITCH_POSITION_DEADZONE_RAD`，默认 0.5°）：误差小于死区时
  位置环输出 0 并复位位置 PID，靠重力前馈+速度环稳住，防止齿距背隙在目标附近高频抖动。
- Pitch 目标直接钳位到实测机械限位（`PITCH_LIMIT_MIN_RAD`/`PITCH_LIMIT_MAX_RAD`，
  IMU Pitch -139°（最高）~-64°（最低）），不靠卡限位检测。
- Pitch 速度环反馈默认用 BMI088 陀螺 pitch 角速度（rad/s→rpm），直接测云台真实
  角速度，不受减速/背隙/柔性影响；GM6020 编码器转速作为复位/离线时的回落反馈。
- BMI088 解算出的 Pitch（带 -90° 零偏，水平=0）用于计算正弦重力电压前馈
  （符号/幅值由 `PITCH_GRAVITY_*` 宏配置）。
- IMU Yaw 归一化为 `[-180°, +180°)`；DM4310 编码器 yaw 保持连续展开，
  因而控制过编码器零点时不会跳变。
- 上电时使用首次 BMI Pitch 给 GM6020 编码器建立零偏，Pitch 目标设为 IMU 水平(-90°)。
- `YAW_COMMISSIONING_MODE=0` 时两个云台轴允许输出；
  `LAUNCH_MOTOR_OUTPUT_ENABLE=0` 会继续向发射机构发送零命令，避免调试云台时误启动。
- Yaw 使用 DM4310 编码器位置作为实际值。限速度、限加速度轨迹同时生成位置、
  速度和加速度参考；`YAW_VELOCITY_FF_GAIN` 调节速度前馈，
  `YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2` 以“CAN 电流命令单位/rad/s²”直接调节
  加速度力矩前馈。两者均受速度/电流限幅保护，MCU 软件速度 PID 根据放大 100 倍的
  转速反馈生成最终电流命令；驱动层和任务层均硬限制为 `-1000`～`1000`。

Yaw 前馈的建议整定顺序：先把 `YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2` 保持为
`0`，固定轨迹最大速度和加速度后，将 `YAW_VELOCITY_FF_GAIN` 从 `0` 以 `0.2`
递增至不再明显减小跟踪误差；再将加速度前馈系数从 `0` 以小步增加。该系数的单位是
“DM4310 CAN 电流命令单位 / 输出轴 rad/s²”，`YAW_ACCELERATION_FF_CURRENT_LIMIT`
独立限制其幅值，最终总电流仍受 `YAW_CURRENT_OUTPUT_LIMIT` 限制。
- 当前保守实车初值使用位置环 `KP=0.35, KI=0.02`、速度环
  `KP=1.0, KI=0.5`、速度前馈 `0.6`。加速度前馈在最大 `0.2 rad/s²`
  轨迹加速度下只贡献 `0.1` 个平均命令单位，总命令仍硬限制为 `-1000～1000`。
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

S1 控制 M2006 ID5：S1=1 角度环保持当前位置；S1=2 连发（纯速度环
4800 rpm≈20 Hz）；S1=3 单动（角度环，每次从 1 拨到 3 触发一步 40° 输出，
电机转 4 圈）。M2006 离线时不会释放启动信号量。

三颗发射电机都有独立的速度 PID、速度低通、积分限幅、输出限幅和方向配置；
M2006 额外有角度环（编码器在电机轴，拨盘在 P36 输出端，36:1 减速）。

## VOFA 与在线调参

UART4 使用 115200 波特率和 RX/TX DMA。`vofa` 任务以绝对节拍每 10 ms 发送 9 个
JustFloat 通道，帧尾为 `00 00 80 7F`：

| VOFA 通道 | 正常模式内容 |
|---|---|
| 1 | Pitch 目标角，° |
| 2 | Pitch 实际角，° |
| 3 | Yaw 目标角，° |
| 4 | Yaw 实际角，° |
| 5 | M2006 目标发弹数（整数） |
| 6 | M2006 实际发弹数（整数） |
| 7 | M3508 ID2 实测转速，rpm |
| 8 | M3508 ID3 实测转速，rpm |
| 9 | 调参解析计数（每成功解析一条串口命令 +1） |

`YAW_SYSID_MODE=1` 辨识固件时，通道 7/8 改为：7=给 DM4310 的电流指令、8=yaw 原始
速度(rpm)，且仅在辨识运行期间打印，运行结束自动静默。

若单次发送或调度导致 deadline 已过期，任务会以当前 tick 重建下一帧的绝对节拍，
不会连续补发历史帧。

UART4 命令接收保持开启（无换行时按接收空闲自动结束一条命令）。支持以下 ASCII 命令：

**在线调参（`键=值`，大写无空格，值 0~100000）**

- `PITCH_KP_POS` / `PITCH_KI_POS` / `PITCH_KD_POS`：Pitch 位置环
- `PITCH_KP_SPD` / `PITCH_KI_SPD` / `PITCH_KD_SPD`：Pitch 速度环
- `YAW_KP_POS` / `YAW_KI_POS` / `YAW_KD_POS`：Yaw 位置环
- `YAW_KP_SPD` / `YAW_KI_SPD` / `YAW_KD_SPD`：Yaw 速度环
- `PITCH_GRAVITY_FF`（别名 `PITCH_GRAVITY_FF_MAX_VOLTAGE`）：Pitch 重力前馈电压幅值

**调试命令**

- `YAWTEST=电流`：Yaw 方向测试，输出固定电流（幅值限 `YAW_DIRECTION_TEST_MAX_CURRENT`，
  持续 `YAW_DIRECTION_TEST_DURATION_MS`）。
- `identify_on`：仅 `YAW_SYSID_MODE=1` 时有效，触发一次 yaw 线性扫频正弦辨识。

每条调参命令只修改指定轴、指定环路。旧的无轴名命令（例如 `KP_POS=...`）会被拒绝。
在线命令只修改运行时 PID，`config.h` 中的宏仍作为下次复位后的初始值。

## 调试/操作宏

以下宏集中在 `Tasks/Inc/config.h`，改后需重新编译烧录：

| 宏 | 取值 | 作用 |
|---|---|---|
| `YAW_SYSID_MODE` | 0/1 | 0=正常闭环；1=yaw 系统辨识固件（pitch 不输出、yaw 直通正弦扫频，`identify_on` 触发，I6=电流指令、I7=原始速度） |
| `PITCH_GRAVITY_ONLY_ENABLE` | 0/1 | 0=正常 Pitch 位置环；1=仅重力前馈（位置环旁路、速度目标=0），用于单独调试重力前馈 |
| `YAW_COMMISSIONING_MODE` | 0/1 | 0=双轴+发射正常；1=仅调试 yaw（pitch 与发射停发命令） |
| `LAUNCH_MOTOR_OUTPUT_ENABLE` | 0/1 | 发射机构（M3508/M2006）是否允许输出；调试云台时设 0 防误启动 |
| `YAW_CLOSED_LOOP_ENABLE` | 0/1 | 0=Yaw 开环（配合 `YAWTEST` 方向测试）；1=Yaw 位置闭环 |
| `YAW_HOME_TO_IMU_ZERO_ON_AUTHORIZE` | 0/1 | 0=授权时保持当前 Yaw；1=授权/遥控重连时自动回 BMI Yaw 零点 |

Yaw 系统辨识参数：`YAW_SYSID_AMPLITUDE_CURRENT`（扫频幅值）、
`YAW_SYSID_FREQ_START_HZ`、`YAW_SYSID_FREQ_END_HZ`、`YAW_SYSID_DURATION_MS`（单次时长）。

Pitch 重力前馈参数：`PITCH_GRAVITY_FF_MAX_VOLTAGE`（电压幅值，运行时可由
`PITCH_GRAVITY_FF` 命令覆盖）、`PITCH_GRAVITY_ZERO_RAD`（水平零点，默认 -90°）、
`PITCH_GRAVITY_SIGN`（符号，方向反了会往下掉，取反即可）。

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
