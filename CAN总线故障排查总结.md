# 云台 CAN 总线故障排查总结

> 记录一次完整的上电无反应 / 电机频繁掉线问题排查过程与最终修复。
> 工程：Real_Robot_Gimbal（STM32F405 + FreeRTOS，双轴云台 + 发射机构）
> 排查对象：CAN1（M3508×2 + GM6020）、CAN2（M2006 + DM4310）、VOFA 诊断、D-BUS 遥控

---

## 一、现象回顾

| 阶段 | 现象 | 表现 |
|---|---|---|
| 最初 | 上电电机无反应，遥控器"在线" | CAN 总线上一个帧都没有 |
| 中途 | CAN2(Yaw) 一直不转 | CAN2 发不出帧、DM4310 收不到 0x3FE |
| 中途 | CAN1 电机跑着跑着掉线 | GM6020 反馈丢失，标记离线 |
| 修复后 | 全部正常 | CAN1/CAN2 均不再 bus-off，电机不掉线 |

---

## 二、排查手段

- 对比 git 工作区与上次提交（`git diff`），定位"这版代码改了什么"。
- 读 Keil 生成的 `.map`，确认弱符号（中断/默认 handler）的实际落点。
- 用 UART4 + VOFA JustFloat 抓 CSV 诊断曲线（在线状态、撤权掩码、bus-off/恢复计数、错误码、邮箱占用等）。
- 逐条核对 HAL 层 `HAL_CAN_AddTxMessage / GetState / Stop / Start / IRQHandler` 与 `stm32f4xx_hal_can.c` 的 bus-off 处理。

---

## 三、根因 1：BMI088 EXTI 中断处理函数丢失（系统启动即挂死）

### 现象
上电后调度器都起不来，D-BUS/遥控看似"在线"，但 CAN 一帧都不发，电机毫无反应。

### 根因
CubeMX 重新生成 `.ioc` 时，把 `NVIC.EXTI0_IRQn / NVIC.EXTI9_5_IRQn` 去掉了，导致生成代码删除了
`EXTI0_IRQHandler` / `EXTI9_5_IRQHandler`。但业务代码仍依赖这两个中断：

- `gpio.c` 仍把 PB0(INT_Accel)、PC5(INT_Gyro) 配成 `GPIO_MODE_IT_RISING`；
- `HardwareDrivers_Init()` 仍执行 `HAL_NVIC_EnableIRQ(EXTI0/EXTI9_5)`；
- 却**没有处理函数**。

编译产物 `.map` 显示两个符号都落在启动文件弱 `Default_Handler`（`B .` 死循环，地址 0x0800031f）。
BMI088 陀螺仪 DRDY 一旦使能，第一个数据就绪脉冲触发 EXTI9_5 → 跳进死循环 → 系统在
`osKernelStart()` 前就挂死。

### 修复
- `stm32f4xx_it.c`：补回 `EXTI0_IRQHandler`（→ `HAL_GPIO_EXTI_IRQHandler(INT_Accel_Pin)`）
  与 `EXTI9_5_IRQHandler`（→ `HAL_GPIO_EXTI_IRQHandler(INT_Gyro_Pin)`）。
- `stm32f4xx_it.h`：补回两个声明。
- `Real_Robot_Gimbal.ioc`：补回两条 NVIC 配置，避免下次 CubeMX 生成又删。
- `hardware_drivers.c`：把 EXTI 优先级设为 5（原代码被生成覆盖成了默认优先级 0，会抢占 CAN RX0）。

---

## 四、根因 2：软件 Bus-off 恢复逻辑制造死循环（两总线频繁掉线）

### 现象
修复根因 1 后能跑起来，但：
- CAN2(Yaw/DM4310) 跑一小段就进 bus-off，Yaw 一直不转；
- 随后 CAN1(GM6020) 在 ~4.49s 也进 bus-off，GM6020 掉线。

### CSV 数据关键点
- CAN2 bus-off / 恢复计数每 10ms 狂涨（错误计数被粘滞 BOF 标志虚增，真实 bus-off 率约 1.4k 次/秒）。
- 起初误判为"CAN1 正常"——只看前 12 行；拉全量后确认 **CAN1 在 ~4.49s 也开始 bus-off**。
- CAN2 bus-off 在先（早于录制），CAN1 在后，二者为同一共同根因。
- CAN1/CAN2 配置唯一差异：`CAN1.SJW=2TQ`（上次生成已改），`CAN2.SJW=1TQ`。SJW 只解释了
  "CAN2 为什么比 CAN1 先挂"，不是共同根因。

### 根因
这版代码在 bus-off 检测后新增软件恢复 `recover_can_if_needed()`，做 `HAL_CAN_Stop→Start`。
而老代码（可正常运行的版本）没有这段，只依赖硬件 ABOM(AutoBusOff) 自动恢复。

机制：
1. CAN bus-off → ErrorCallback 置 pending。
2. ABOM 在 ~1.4ms 内硬件自动恢复，并**立刻重新开始重发**之前卡住的帧。
3. 软件恢复在**任务上下文延迟执行**（晚于 ABOM，最多 4ms），此时才 `HAL_CAN_Stop`。
4. 这一 Stop 打断正在重发的半截帧、进出 init mode，把**残缺帧留在总线上**。
5. 其它节点收到残缺帧 → 发错误帧 → TEC 再涨 → 再次 bus-off → 又触发恢复……
6. 两条总线都被拖进 1k+ Hz 的 bus-off 死循环，电机频繁掉线。

### 修复
- `can_motor_bus.c`：`recover_can_if_needed()` 改为只清 pending，**不做软件 Stop→Start**，
  恢复完全交给硬件 ABOM（回到老代码行为）。
- `can.c` + `.ioc`：把 CAN2 的 `SJW` 从 1TQ 对齐到 2TQ（消除 CAN1/CAN2 不对称，增强位同步容差）。

### 验证
- CAN2 bus-off / 恢复计数不再增长。
- CAN1 不再在 4.49s 掉线，GM6020 在线状态保持。
- CAN2 总线稳定后 0x3FE 能持续喂给 DM4310，Yaw 正常授权转动。
- 全部电机不掉线，云台功能正常。

---

## 五、本次新增的诊断通道（UART4 VOFA，JustFloat 帧尾 00 00 80 7F）

| 通道 | 内容 |
|---|---|
| 1 | BMI088 Pitch 角，° |
| 2 | Pitch 重力前馈设定值 |
| 3 | 重力前馈计算值 |
| 4 | Pitch 编码器角，° |
| 5 | GM6020 速度，rpm |
| 6 | GM6020 在线状态 |
| 7 | 控制撤权位掩码 |
| 8 | CAN1 空闲发送邮箱数 |
| 9 | DM4310(Yaw) 在线状态 |
| 10 | DM4310 反馈帧累计 |
| 11 | CAN2 空闲发送邮箱数 |
| 12 | CAN2 最近收到的标准帧 ID |
| 13 | CAN 总线错误累计 |
| 14 | CAN1 bus-off 次数 |
| 15 | CAN1 恢复次数 |
| 16 | CAN2 bus-off 次数 |
| 17 | CAN2 恢复次数 |
| 18 | CAN2 最后错误码 |

> 判读提示：bus-off / 恢复计数若持续增长说明总线在反复 bus-off。
> 错误码：`32`=无 ACK，`64`=位隐性错误，`4`=bus-off。

---

## 六、经验 / 注意点

1. **不要轻易在 `.ioc` 里关掉正在用的外部中断**，否则 CubeMX 重新生成会删 handler，而业务代码
   仍使能 NVIC → 系统静默挂死或 IMU 不采样。
2. **总线恢复优先靠 ABOM**；软件 Stop→Start 恢复要非常小心时序，不要在 ABOM 已恢复、正在重发时
   打断，否则会造成 bus-off 死循环。
3. **看 CSV/曲线要拉全量时间轴**，只看开头几行容易误判"某条总线正常"。
4. **在线调参不写 Flash**，复位后仍用 `config.h` 初值。

---

## 七、在线调参命令（UART4，回车/换行结尾）

| 命令 | 含义 |
|---|---|
| `PITCH_GRAVITY_FF=1500` | 设 Pitch 重力前馈最大电压（默认 1200） |
| `PITCH_KP_POS=90` | Pitch 位置环 KP |
| `PITCH_KP_SPD=80` | Pitch 速度环 KP |
| `YAW_KP_POS=0.5` | Yaw 位置环 KP |
| `YAW_KP_SPD=2` | Yaw 速度环 KP |

> Pitch 重力前馈也可用别名 `PITCH_GRAVITY_FF_MAX_VOLTAGE=<值>`。
