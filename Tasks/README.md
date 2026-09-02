# FreeRTOS 任务与项目效果说明

## Data_Process 数据处理任务

`Data_Process` 默认每 1 ms 运行一次，负责：

- 原子读取 BMI088 DMA 完整采样结果。
- 推进等待中的 BMI088 DMA 阶段，并在单段超时后执行任务态恢复。
- 使用加速度计和陀螺仪互补滤波计算 Roll、Pitch，并积分得到相对 Yaw。
- 处理大疆 D-BUS 遥控数据和失联状态。
- 检测 CH6～CH9 突变，生成 Pitch/Yaw 的 ±30° 增量命令。
- 将姿态和角度增量通过 `Target_Angle` 队列发送给 `PID_calc`。
- 解析 UART4 收到的在线 PID 调参命令，并通过 `Update_PID_para` 队列下发。
- 将 S1、S2 三档映射后的发射机构目标速度通过 `Update_launch_para` 下发。

## 云台控制任务

`PID_calc` 以 1 kHz 运行：

- Pitch 使用 GM6020 编码器展开角度作为实际值，经过低通滤波后进入位置环；
  位置环输出目标转速，再进入 GM6020 速度 PID。
- BMI088 解算出的 Pitch 用于计算余弦重力电压前馈。
- 上电时使用首次 BMI Pitch 给 GM6020 编码器建立零偏，Pitch 目标设为 0°。
- Yaw 使用 DM4310 编码器位置作为实际值。限速度、限加速度轨迹同时生成位置、
  速度和加速度参考；`YAW_VELOCITY_FF_GAIN` 调节速度前馈，
  `YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2` 以“CAN 电流计数/rad/s²”直接调节
  加速度力矩前馈。两者均受速度/电流限幅保护，MCU 软件速度 PID 根据放大 100 倍的
  转速反馈生成最终 `-16384`～`16384` 电流命令。

Yaw 前馈的建议整定顺序：先把 `YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2` 保持为
`0`，固定轨迹最大速度和加速度后，将 `YAW_VELOCITY_FF_GAIN` 从 `0` 以 `0.2`
递增至不再明显减小跟踪误差；再将加速度前馈系数从 `0` 以小步增加。该系数的单位是
“DM4310 CAN 电流命令计数 / 输出轴 rad/s²”，`YAW_ACCELERATION_FF_CURRENT_LIMIT`
独立限制其幅值，最终总电流仍受 `YAW_CURRENT_OUTPUT_LIMIT` 限制。
- DM4310 的 0～8191 单圈位置会先跨零展开，再进行低通滤波，避免编码器过零时
  角度跳变。
- 上电第一帧 DM4310 编码器位置作为 Yaw 初始保持位置，不会强制回协议零位。
- D-BUS Failsafe/丢帧当帧撤权；遥控超时、CAN故障或控制超期调用
  `CanMotorBus_StopAll()`。
- GM6020、DM4310 或 BMI088 不新鲜时调用 `CanMotorBus_StopGimbal()`，只停云台，
  不覆盖独立发射机构目标。
- 掉线时清除云台PID、编码器展开和旧目标；恢复后以当前位置无扰重建。
- `osDelayUntil()`若发现deadline已过期，会重置调度基准并先执行安全停机，
  不连续补跑遗漏的1 ms周期。

## 发射机构任务

S1/S2 使用 D-BUS 三档离散值控制发射机构：

```text
开关值 1 -> 停止
开关值 2 -> 低速
开关值 3 -> 最高速
```

S1=1 时 M3508 停止，S1=2 使用低速目标，S1=3 使用最高速目标。两颗 M3508
只要至少一颗在线即可运行，ID2 和 ID3 的方向分别由
`LAUNCH_M3508_ID2_DIRECTION`、`LAUNCH_M3508_ID3_DIRECTION` 设置。任意一颗
掉线都会同时停止两颗电机。

S2 使用相同的三档关系控制 M2006 ID5。修改
`LAUNCH_M2006_ID5_MAX_SPEED_RPM` 后，S2=3 对应新的最大转速，S2=1 保持停止。
M2006 离线时不会释放启动信号量。

三颗发射电机都有独立的速度 PID、速度低通、积分限幅、输出限幅和方向配置。

## VOFA 与在线调参

UART4 使用 115200 波特率和 RX/TX DMA。`vofa` 任务以绝对节拍每 20 ms 发送 3 个
JustFloat 通道，帧尾为 `00 00 80 7F`：

| VOFA 通道 | 设备 |
|---|---|
| 1 | BMI088 Roll 解算角度，单位为度 |
| 2 | BMI088 Pitch 解算角度，单位为度 |
| 3 | BMI088 Yaw 解算角度，单位为度 |

若单次发送或调度导致 deadline 已过期，任务会以当前 tick 重建下一帧的绝对节拍，
不会连续补发历史帧。

支持以下以回车或换行结尾的 ASCII 命令：

- `KP_POS=2`
- `KI_POS=0`
- `KD_POS=0`
- `KP_SPD=80`
- `KI_SPD=8`
- `KD_SPD=0`

位置环参数会更新 Pitch 和 Yaw 外位置环；速度环参数更新 GM6020 软件速度环。
DM4310 的 Yaw 速度环使用 `config.h` 中独立的 `YAW_SPEED_*` 参数。在线命令
直接修改运行时 PID，`config.h` 中的宏仍作为下一次复位后的初始值。

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
