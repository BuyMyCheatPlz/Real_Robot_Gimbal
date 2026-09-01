# 硬件驱动说明

## 已配置硬件

- CAN1：M3508/C620，ID 2 和 ID 3；GM6020，ID 2。
- CAN2：M2006/C610，ID 5；DM4310 标准 CAN 版，电机 ID 1，主机 ID `0x11`。
- USART2：富斯 S.BUS，`100000 8E2`，DMA + 空闲中断接收。
- SPI1：BMI088 标准 SPI 协议，使用 DMA。陀螺仪 CS 为 PA4，加速度计
  CS 为 PC4，陀螺仪 INT3 为 PC5，加速度计 INT1 为 PB0。

`HardwareDrivers_Init()` 会启动两路 CAN、S.BUS 接收并完整初始化 BMI088。
初始化阶段不会给 DJI 电机加载非零 PID，也不会让电机产生非零输出，避免上电误动作。

## 电机驱动

任务层从 `Tasks/Inc/config.h` 加载实车 PID 参数并设置目标速度，随后由
`PID_calc` 每 1 ms 调用 `CanMotorBus_Update(dt_s)` 统一发送 CAN 控制帧。

紧急停止使用 `CanMotorBus_StopAll()`。每个反馈结构体均包含 `online` 和
`last_update_ms`，用于判断电机是否在线。

## S.BUS 电气要求

STM32F405 的串口没有硬件 RX 极性反转功能，而标准 S.BUS 波形为反相电平。
因此 USART2 RX 必须连接接收机的非反相 S.BUS 输出，或者在 PA3 前增加外部
反相器。将反相 S.BUS 波形直接接入 PA3 时无法解析出有效遥控帧。

## BMI088

板级初始化会自动调用：

```c
BMI088_Init(&bmi088, &hspi1,
            CS_Accel_GPIO_Port, CS_Accel_Pin,
            CS_Gyro_GPIO_Port, CS_Gyro_Pin);
```

陀螺仪 INT3 以 1 kHz 触发加速度、角速度和温度的三级 DMA 读取。加速度计
INT1 同时启用并记录中断次数，便于诊断。默认量程为加速度计 ±6 g、800 Hz，
陀螺仪 ±2000 °/s。输出单位分别为 m/s²、rad/s 和摄氏度。
