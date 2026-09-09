# Pitch 轴 PID 控制链路对比：cloud-platform-main（模板） vs Real_Robot_Gimbal（本工程）

> 前提（你已确认）：由于 IMU 安装方向不同，本工程里 **BMI088 的 Roll 通道（Z 轴，取反）就是物理俯仰轴的测量值**；代码中凡写 "pitch 用 Roll / 本机 Pitch 对应 BMI088 Roll" 的地方都指这条映射。
> 本文按 **物理俯仰轴** 对齐两套工程逐级对比，重点标出"因安装方向导致的差异"与"纯粹的工程增强差异"。

---

## 0. 总览结论

两套工程是**同一种架构的 1 kHz 双闭环**，逐级形态完全一致：

```
目标(角度) ──► [位置环 PID] ──速度目标──► [速度环 PID] ──电流──► GM6020(CAN1,ID2) ──► 物理俯仰轴
                   ▲                                             ▲
                   │                                             │
          测量: GM6020 编码器(多圈展开)                  反馈: IMU 陀螺角速度(物理俯仰轴)
                                                        (模板: pitch 通道; 本工程: roll 通道 Z,-1)
重力前馈: k·sin(俯仰角) 直接叠到速度环输出/驱动层 (模板扣减: PID−FF；本工程驱动内 PID+FF，符号取反等效)
```

| 环节 | cloud-platform-main（模板） | Real_Robot_Gimbal（本工程） | 差异根源 |
|---|---|---|---|
| 位置测量 | GM6020 编码器 8192/圈，多圈展开成 ° | 同左，但 ×(−1)、转 rad、编码器低通 0.15 | 符号=安装方向；滤波=增强 |
| 位置环反馈语义 | 编码器绝对角度（机械坐标系），首帧锁目标 | 授权时用 IMU Roll 建零偏 → 之后只走编码器（相对水平回零） | 回零/水平参考 = 安装方向补偿 |
| 速度环反馈 | IMU **pitch 通道** 陀螺 °/s（无滤波） | IMU **roll 通道** 陀螺 °/s（×(−1) 转电机坐标，无滤波） | **纯安装方向差异** |
| 重力前馈输入 | IMU **pitch** 角 − 9.9°（静态偏置） | IMU **roll** 角滤波(0.05)，零点 Roll=0 | **纯安装方向差异**（9.9° 变成了显式水平回零） |
| 输出合成 | 任务内：`speedPID.output − FF` → int16 电流 | 驱动内：`speedPID.output + voltage_feedforward`（符号取反后等效 −FF） | 组织方式不同，数值等效 |

除安装方向/符号/回零外，本工程在模板基础上做了大量**控制增强**：D 项取舍、积分分离+抗饱和、反馈源管理与重置、限幅 90 vs 200、离线/总线/CAN 故障保护、启动助推等（见 §5）。

---

## 1. 参考坐标系与通道映射（两工程本质差异所在）

### 模板 cloud-platform-main：芯片坐标系 == 物理轴，无需重映射
- 传感器 BMI088，`IMUTask`（`Core/Tasks/Src/IMUTask.c`）读原始 `gyro[3]/accel[3]`，仅做**开机静止零偏**，Mahony 融合输出标准欧拉角：
  - `imu_data.pitch_angle = pitch`（四元数→欧拉）
  - `imu_data.pitch_gyro = gyro[1]`（原始 Y 轴角速度，rad/s）
  - `imu_data.yaw_gyro   = gyro[2]`
- 物理俯仰运动恰好落在芯片 pitch(Y) 通道 → 代码里所有 "pitch" 名字直接对应物理俯仰，无需换轴、无需改符号。
- 误差：把机械水平与重力零位之差**固化成一个静态常数** `ff.input = pitch − 9.9°`（见 CloudPlatformTask.c §"前馈"），机械软限位也写成编码器绝对角 `168~238°`。

### 本工程 Real_Robot_Gimbal：芯片 Z（roll）承载物理俯仰，全部靠宏重映射
- `config.h` IMU 安装区：
  - `IMU_GYRO_ROLL_AXIS=2`、`IMU_GYRO_ROLL_SIGN=-1`：**Z 轴取反 = 物理俯仰角速度**
  - `IMU_GYRO_PITCH_AXIS=0(X)`、`IMU_GYRO_YAW_AXIS=1(Y)`：仅作姿态诊断
  - 加速度计仍用芯片三轴原值做 `atan2` 姿态，弱修正（`ATTITUDE_ACCEL_WEIGHT=0.01`）
- `data_process.c / update_attitude()`：
  - `message.roll_rad` ← 积分 `gx = −rate[Z]` + accel 弱修正（**这是物理俯仰角**）
  - `message.roll_rate_rad_s` ← `gx`（**这是物理俯仰角速度，喂给速度环**）
  - `message.pitch_rate_rad_s` ← X 轴，注释明言“供姿态诊断”
- 因此**速度环的"反馈通道"整体从模板的 pitch(X/Y) 搬到了 roll(Z)，并加了 −1 符号**，这是“我的 roll 是 pitch 的测量值”这一安装事实在代码里的全部体现。

---

## 2. 位置测量 / 位置环（级 1）

### 模板
`CloudPlatformTask.c Gimbal_Run()`：
1. CAN 反馈 `feedback.angle`（8192 圈内 14bit）在临界区内取走；
2. 首帧锁 `angle_raw_last`，`angle_360 = raw*360/8192` 并 `angle_pid.target = angle_360`；
3. 之后每 1 ms 做**最短路径展开**：`delta=(raw−last)`，越 ±4096 补 ±8192，`angle_360 += delta*360/8192`；
4. `angle_pid.current = angle_360`（**无滤波**），`PID_Calculate`。

位置环参数（`PID_Init(pid,kp,ki,kd, integral_limit, output_limit, deadzone)`，pid.c 语义：`|e|≤dz` 才积分否则清零，输出 = kp·e+ki·∫+kd·(e−e_last)，双向限幅）：
- `Kp=22.4, Ki=0.01, Kd=0, Ilim=0.8, Olim=200, dz=0.4`（误差单位 °；输出=速度目标 °/s）
- 目标更新：SWA 使能上升沿重锁（目标=当前位置、清积分）；SWC 中→上/下 ±30° 步进；**右摇杆 pitch_rate 连续速率**每 1 ms `target += pitch_rate*0.001`；软限位夹 `[170,236]°`（编码器绝对坐标系）。

### 本工程
`pid_calc.c PID_calc()`：
1. 编码器多圈展开同上，但立即 `×PITCH_ENCODER_TO_IMU_SIGN(−1)` 转成 rad，并做**一阶低通 `PITCH_ENCODER_LPF_ALPHA=0.15`**（对展开后的绝对弧度滤波）；
2. **首次获得控制权时**（`pitch_control_permitted && 编码器已初始化 && 目标未初始化`）：
   ```c
   pitch_encoder_offset = pitch_encoder_filtered − imu_roll; // imu_roll = 物理俯仰(Roll通道)
   pitch_angle_actual   = imu_roll;
   pitch_home = 0; pitch_target = 0 + pending_pitch_delta;
   ```
   即：**用“Roll=0 即水平”锚定编码器零偏，目标=0 → 上电自动回正到水平**；
3. 此后运行态只走编码器：`pitch_angle_actual = pitch_encoder_filtered − offset`（注释：避免姿态解算跳变直接进位置环）；
4. `pitch_speed_target_rpm = position_pid(pitch_target*RAD_TO_DEG, pitch_angle_actual*RAD_TO_DEG)`。

位置环实现为自制 `PositionPid_t`（D=每拍误差差量 kd·(e[k]−e[k-1]) 与模板一致、条件积分、**抗饱和回算**）：
- `Kp=22.4, Ki=0.01, Kd=0`，`Ilim=30`，`Olim=PITCH_MAX_SPEED_RPM=90`（`° /s`，宏名 RPM 为历史遗留，见 config.h 注释）
- 目标：D-Bus CH4（右杆俯仰）**每次越过死区变向** ±30°（`GIMBAL_COMMAND_STEP_DEG`）步进，软限位 ±90°（相对水平 home）
- `PITCH_POSITION_DEADZONE_RAD(0.5°)`、`PITCH_LIMIT_*`、`PITCH_HOME_STABLE_TIME_MS` 等宏目前**只在 config.h 定义、未见引用**（意图文档，未激活代码）

### 差异小结（级1）
| 项目 | 模板 | 本工程 |
|---|---|---|
| 编码器→弧度/° | 无符号、无滤波、° | ×(−1)、rad、LPF 0.15 |
| 运行期位置源 | 编码器（无 IMU 参与） | 编码器（仅授权瞬间用 Roll 建零偏） |
| 零位/回零 | 无（锁当前角）+ ff 静态 −9.9° | 回正到 Roll=0=水平（home=0） |
| 位置环 Olim | 200 | 90 |
| 积分策略 | \|e\|≤0.4 积分否则清零，Ilim 0.8 | 积分分离=0(常积)，Ilim 30，抗饱和回算 |
| 软限位 | 编码器绝对 170–236° | 相对水平 ±90° |

---

## 3. 速度环（级 2）——安装方向的核心差异点

### 模板
`CloudPlatformTask.c`：
```c
if (gimbal_index == PITCH)
    speed_pid.current = imu_data.pitch_gyro * 57.2958f;   // rad/s→°/s，物理俯仰=芯片 pitch 通道
speed_pid.target = angle_pid.output;
PID_Calculate(&speed_pid);
```
- 反馈**来自 IMU pitch 陀螺**（不是 GM6020 编码器转速）
- `Kp=194.44, Ki=24.3, Kd=60.85, Ilim=3000, Olim=25000, dz=5`（误差单位 °/s；I 只在 |e|≤5 累积；输出=电流 int16）
- 无低通、无积分分离、无离线重置（积分的唯一清零点在使能重锁时）

### 本工程
`pid_calc.c` + `gm6020.c` + `motor_common.c`：
```c
/* 位置环输出(° /s)→驱动目标转速 */
GM6020_SetSpeed(&can1_gm6020_id2, PITCH_CONTROL_TO_MOTOR_SIGN * pitch_speed_target_rpm);
/*        PITCH_CONTROL_TO_MOTOR_SIGN = PITCH_MOTOR_SIGN(1) × PITCH_ENCODER_TO_IMU_SIGN(−1) = −1 */

/* 速度环反馈：BMI Roll 陀螺 = 物理俯仰角速度，必须转到电机坐标 */
can1_gm6020_id2.external_speed_rpm =
    PITCH_ROLL_RATE_TO_SPEED_SIGN * bmi_roll_rate_rad_s * RAD_TO_DEG;
/*        PITCH_ROLL_RATE_TO_SPEED_SIGN = PITCH_MOTOR_SIGN × PITCH_ENCODER_TO_IMU_SIGN = −1 */
can1_gm6020_id2.use_external_speed_feedback = 1;
```
驱动内（`GM6020_Update`）：
- 反馈选择：`use_external_speed_feedback ? external_speed_rpm : 编码器 rpm`，速度环反馈一阶低通 `PITCH_SPEED_LPF_ALPHA=0.50`（原 1.0=模板直采不滤波；本次按需开启滤波抑制高频噪声/毛刺）
- `MotorSpeedPid_Calculate(target, filtered)`：`Kp=194.44, Ki=24.3, Kd=60.85`（D=每拍误差差量 kd·(e[k]−e[k-1])，模板语义，**不除以 dt**），积分分离 `|e|≤70`，Ilim=12000，Olim=±25000
- 输出 = 速度环输出 + `voltage_feedforward`（重力前馈在 §4），再 `±25000` 限幅
- **离线立即清积分+重初始化滤波器**（防恢复瞬间猛转）；未授权/复位时 `reset_pitch_control()` 把反馈回落到编码器并清 external（见 can_motor_bus.c 注释：“下次授权会切换到 BMI Roll 速度反馈；强制首帧重新初始化，避免旧编码器 rpm 与新 °/s 混合”）

### 差异小结（级2）
| 项目 | 模板 | 本工程 |
|---|---|---|
| 速度反馈物理量 | IMU pitch(Y) 陀螺 °/s | **IMU roll(Z) 陀螺 °/s × (−1)** ← 安装方向 |
| 速度反馈在控制链中的位置 | 任务内直接算 | 下沉到 GM6020 驱动层、可注入外部反馈/可回落编码器 |
| Kp / Ki | 194.44 / 24.3 | 194.44 / 24.3（同源参数） |
| Kd | 60.85：D=每拍误差差量 `kd·(e[k]−e[k-1])`，**不除以 dt**（pid.c 原始语义） | **60.85：已按同一"每拍差量"语义实现并启用**（原 MotorSpeedPid 除以 dt，同数值放大 1000× → 一加 D 就抖振） |
| 积分 | \|e\|≤5 才积, Ilim3000 | 分离 \|e\|≤70, Ilim12000, 抗饱和回算 |
| 离线/复位 | 无（仅使能重锁清零） | 离线即清积分/滤波/回落反馈源 |
| 单位 | °/s 反馈 vs °/s 目标 | 数值同为 °/s（变量名 rpm 是历史遗留） |

> 结论：**速度环“用陀螺在物理俯仰轴上闭环”这一做法两工程一致**；本工程只是把"哪一根陀螺轴、什么符号"显式参数化——`IMU_GYRO_ROLL_AXIS=Z/-1 → external_speed_rpm → −1`。

---

## 4. 重力前馈（级 3）——安装方向的第二个核心差异点

### 模板
`CloudPlatformTask.c`：
```c
ff.input = imu_data.pitch_angle − 9.9f;     // IMU pitch(°) 减静态水平偏置
Feedforward_Calculate();                    // out = k·sin(input°), k=13519, 限±25000
...
pitch_output = (int16)(speed_pid.output − ff.output);  // 电流 = PID − 前馈
Motor_Write(&hcan1, GM6020_STDID_FRONT, 0, pitch_output, 0, 0);
```
- 角度取自 **IMU pitch（Mahony 融合）**，无滤波；9.9° 是把"编码器/机械水平"与"IMU 姿态零位"之差写成常数；
- 正弦零点 = `pitch=9.9°` 处；正值（上扬）→ 输出正 ff 后**减去**。

### 本工程
`pid_calc.c`：
```c
/* 角度取自滤波后的 Roll（物理俯仰），Roll=0° 即过零 */
pitch_gravity_roll_filtered += 0.05 * 最短角差(imu_roll, ...);      // PITCH_GRAVITY_ROLL_LPF_ALPHA
gravity_feedforward = pitch_gravity_ff_voltage *                    // config 现值 8519(模板原值 13519)
                      PITCH_GRAVITY_SIGN(−1) * sin(roll − 0);       // PITCH_GRAVITY_ZERO_RAD=0
GM6020_SetVoltageFeedforward(&can1_gm6020_id2, −PITCH_MOTOR_SIGN(1) * gravity_feedforward);
// 驱动内: 电流 = 速度环输出 + voltage_feedforward  → 等效 速度环输出 − 幅值·sin(roll)
```
- **输入是 Roll 通道角度**（=物理俯仰），单独低通 0.05（与速度环隔离，不给速度/位置环加滞后）；
- 过零点显式化为 `PITCH_GRAVITY_ZERO_RAD=0`，符号 `PITCH_GRAVITY_SIGN=−1`，再在调用处取负以匹配模板的"PID−FF"；
- 模板的静态 −9.9° 在安装重映射后被“编码器零偏建在 Roll=0（水平）”吸收掉了。

### 差异小结（级3）
| 项目 | 模板 | 本工程 |
|---|---|---|
| 角度源 | IMU **pitch** 融合角 − 9.9° | IMU **roll** 滤波角 − 0（物理俯仰）← 安装方向 |
| 零点处理 | 静态常数 9.9° | 显式 `ZERO_RAD=0` + 水平回零一致化 |
| 滤波 | 无 | Roll 专用 LPF 0.05 |
| 符号 | 隐含在“向上为正 + 相减” | 宏 `PITCH_GRAVITY_SIGN=−1` + 调用处取负 |
| 幅值/限幅 | k=13519, 限 25000 | config 现值 `PITCH_GRAVITY_FF_MAX_VOLTAGE=8519`（调参中下调过，模板原值 13519），驱动再限 ±25000 |
| 叠加位置 | 任务内相减后整帧下发 | 驱动内 `PID + voltage_feedforward` |

---

## 5. 其它（非安装方向）链路差异一览

| 差异 | 模板 | 本工程 | 作用 |
|---|---|---|---|
| 使能/回正时机 | 首帧锁当前角度，使能上升沿重锁目标 | 授权时回正到 Roll=0（水平）+ 目标=0；重锁在使能条件内 | 回正语义更明确 |
| 目标命令 | SWC ±30° 步进 + 右杆连续速率(300°/s 满量程) | D-Bus CH4/CH1 越死区 ±30° 步进（无连续速率） | 遥控逻辑 |
| 周期 | CloudPlatformTask 1 ms | PID_calc 1 ms（osDelayUntil），Data_Process 1 ms 供姿态 | 相同 |
| 任务内安全 | 命令超时 100 tick 断电、SWA/flag_stop 门控 | IMU 20ms/遥控 150ms/总线超时、CAN 故障锁存与恢复、control_overrun 检测、未初始化抑制位 | 更完整 |
| 滤波 | 无 | 编码器 0.15、重力 Roll 0.05、**速度环反馈 0.50**（本次开启；原 1.0=模板直采） | 位置/速度/前馈降噪 |
| 抗饱和 | 简单 I 限幅 + \|e\|≤dz 积分 | 积分分离 + 条件积分 + 输出饱和回算 | 防积分饱和猛冲 |
| 启动 | 无 | 曾有 `|目标|≥1°/s 且 |输出|<8000 → 顶 ±8000` 继电器（本轮已置 0 关闭，等同模板） | 该继电器是保持段 ~3.3Hz 极限环的疑似根源 |
| D 项 | 速度环 60.85（每拍差量） | 速度环 60.85（同语义，本次已对齐启用）；位置环 0（同模板） | 与模板一致 |
| 前馈 | sin(°)，任务层相减 | sin(rad)，驱动层 +ff 取负等效 | 等效 |
| 诊断 | VOFA 全链路调参 | VOFA + 状态结构体 + 各环可在线改参 | 同 |

---

## 6. 若要在两工程间移植模板的整定组 / 复核方向

1. **物理量对齐后再比参数**：模板整定组是“位置 ° → °/s；速度 °/s → 电流”。本工程同名宏带 RPM 字样但注释明确"位置 °、速度 °/s"（`config.h` 100–147 行）。两边 `Kp位置=22.4、Kp速度=194.44、Ki速度=24.3` 数值同源。
2. **速度环 60.85 的 D 已完成对齐（本次修改）**：模板 pid.c 的 D 是"每拍误差差量" `kd·(e[k]−e[k-1])`（不除以 dt）；本工程原 `MotorSpeedPid` 写成 `-(meas-prev)/dt`（每秒导数），同数值被放大 `1/dt≈1000` 倍 → 一加 D 就顶到 ±25000 抖振。已在 `motor_common.c` 改为与模板相同的每拍差量语义，并把 `PITCH_SPEED_KD` 置为 60.85。若坚持用"每秒导数"形式，等效数值只有 60.85×0.001≈0.06，且无法再直接照抄模板整定表。
3. **位置环 D 的语义也已统一为"每拍差量"（本次修改）**：模板 pitch 角度环 D=0，阻尼完全来自速度环；但若确实要给 Pitch 位置环加外环阻尼，现在 `PositionPid_t` 与速度环同为模板语义，不会再被放大 ~1000 倍。按"每拍差量"标定：位置环 D 在 ° 域上做速度阻尼时系数 = Kd×0.001，起步建议 Kd≈100~300（对应 0.1~0.3 速度阻尼，模板 yaw 的 296.92 就是这一量级）。yaw 原 0.10（每秒导数）已等价重标定为 100.0（每拍差量），行为不变。
4. **复核符号链**（都已在 config.h 集中，改一处即可）：
   - `PITCH_ENCODER_TO_IMU_SIGN=−1`（编码器增量与 IMU 相反，实测 CSV）
   - `IMU_GYRO_ROLL_SIGN=−1`（roll 通道取反后与加速度计右倾方向一致）
   - `PITCH_ROLL_RATE_TO_SPEED_SIGN = 1×(−1) = −1`（陀螺 °/s 转电机坐标）
   - `PITCH_CONTROL_TO_MOTOR_SIGN = −1`（速度目标进驱动前的符号）
   - `PITCH_GRAVITY_SIGN=−1` + 调用处 `−PITCH_MOTOR_SIGN*ff`（等效模板"PID−FF"）
5. **水平回零 vs 模板静态 −9.9°**：本工程以“授权时 Roll=0 建编码器零偏”替代模板的固定前馈偏置；若机构水平时 IMU 读 Roll≠0（安装没调平），应在 `imu_roll` 处加常量偏置或把 `PITCH_GRAVITY_ZERO_RAD` 填成该角度，而不是改回 9.9°。
6. **两级反馈通道不混用**：速度环外部反馈只在 `pitch_target_initialized && use_external_speed_feedback=1` 后生效；复位/掉线会回落编码器 rpm（`reset_pitch_control`）。若观察到刚使能瞬间速度环反馈跳变，多半是编码器 rpm 与 roll °/s 混用窗口，属正常保护逻辑。

---

## 7. 主要代码位置索引

### cloud-platform-main（模板）
- `Core/Tasks/Src/CloudPlatformTask.c`：目标更新/重锁、编码器展开、角度环、速度环（pitch 用 `imu_data.pitch_gyro`）、前馈合成、输出
- `Core/Tasks/Src/IMUTask.c`：BMI088→Mahony→`pitch/pitch_gyro/yaw_gyro`
- `Core/Src/pid.c` + `Core/Inc/pid.h`：PID_Init/PID_Calculate（GBK 编码，可用编辑器转码查看）
- `Core/Src/feedfoward.c` + `Core/Inc/feedforward.h`：`k·sin(°)`
- `Core/Tasks/Src/CommandTask.c`：SWA/SWB/SWC/摇杆 → 目标增量
- `Core/Src/motor.c`：Motor_Write/Motor_Read（GM6020 大端 slot2；DM4310 小端）

### Real_Robot_Gimbal（本工程）
- `Tasks/Inc/config.h`：轴映射/符号/全部增益（38–57、100–147 行为 pitch 相关）
- `Tasks/Src/data_process.c`：`update_attitude()` → roll_rad/roll_rate_rad_s 即为物理俯仰
- `Tasks/Src/pid_calc.c`：`PID_calc()` 级 1–3（609–623 授权回正、796–828 pitch 位置环+Roll 速度反馈+前馈）
- `Hardware_Drivers/Gm6020/Src/gm6020.c`：速度环+voltage_feedforward+启动助推
- `Hardware_Drivers/Can/Src/motor_common.c`：MotorSpeedPid_Calculate（抗饱和/分离）
- `Hardware_Drivers/Can/Src/can_motor_bus.c`：离线重置/反馈源回落/发送与故障
- `Hardware_Drivers/Gm6020/Inc/gm6020.h`：external_speed_rpm 说明（“本项目 Pitch 使用 BMI088 Roll 的 °/s”）

---

## 8. 本次修改记录（D 项抖动修复）

### 根因
D 的数值语义与模板不一致（两处：速度环 `MotorSpeedPid`、位置环 `PositionPid_t` 都是"每秒导数/除以 dt"）：
- 模板 `pid.c`：`output += kd*(error - error_last)` —— **每 1 ms 一拍取误差差量，不除以 dt**；1 kHz 下 60.85 是弱阻尼量级（几十~几百电压单位）。
- 本工程原 `MotorSpeedPid_CalculateFloat` / `position_pid`：`derivative = -(meas-prev)/dt; output += kd*derivative` —— 把误差按**每秒导数**（除以 dt）算。同样填模板数值，作用被放大 `1/dt≈1000` 倍 → 正常速度波动/噪声直接顶到输出限幅 → "一带上 D 就剧烈抖动"，并且因立刻饱和根本无法体现阻尼。
- Pitch 位置环的测量是 1 kHz 滤波后的 GM6020 编码器（14bit，0.044°/计数，齿隙/抖动时 ±1 计数翻转）→ 原语义下每拍差分 /dt 产生 ±几十°/s 的量化尖峰，位置环 D>0 即被打成 bang-bang。

### 改动（含第二轮：位置环）
| 文件 | 改动 |
|---|---|
| `Hardware_Drivers/Can/Inc/motor_common.h` | `MotorSpeedPid_t.previous_measurement` → `previous_error`（D 存上一拍误差） |
| `Hardware_Drivers/Can/Src/motor_common.c` | 速度环 D 项改为模板语义 `kd*(e[k]-e[k-1])`（不除以 dt）；Init/Reset 同步 |
| `Tasks/Src/pid_calc.c` | 位置环 `PositionPid_t` D 项同样改为每拍误差差量语义（struct/计算/复位同步） |
| `Tasks/Inc/config.h` | `PITCH_SPEED_KD` 0.0 → 60.85（模板值）；`YAW_ANGLE_KD` 0.10 → 100.0（纯重标定，保持行为不变：0.10/dt=100） |
| `Tasks/Inc/config.h` | （第三轮）`PITCH_SPEED_LPF_ALPHA` 1.0 → 0.50：开启速度环反馈(Roll 陀螺 °/s)一阶低通 |

### 注意事项
- Pitch 位置环 D 默认保持 0：与模板一致（模板 pitch 角度环 D=0，俯仰阻尼全部来自速度环 D=60.85）。
- 若确实要试 Pitch 外环阻尼：语义已与模板一致，`PITCH_KD_POS` 起步建议 **100~300**（≈0.1~0.3 速度阻尼系数，模板 yaw 296.92 属同一量级）；不再会像"每秒导数"语义那样一加就满幅抖。
- 在线 VOFA 调 `PITCH_KD_SPD` / `PITCH_KD_POS` 都走每拍差量语义：`SPD` 直接填 60.85 起步；`POS` 按上面量级。
- yaw 角度环 D 已等价重标定为 100.0（原来 0.10 × 1000），上电行为不变；yaw/摩擦轮/拨弹速度环 Kd 仍为 0，不受影响。以后任何轴加 D 都按"每拍差量"标定（模板值可直接用），或把"每秒导数"值 ×0.001 折算。
- 速度反馈低通 `PITCH_SPEED_LPF_ALPHA` 已由 1.0（模板直采、不滤波）改为 **0.50**：抑制速度环高频噪声/毛刺。注意这是反馈滤波，会给 D 引入轻微相位滞后——若 D 起振或响应变钝，先把 alpha 回调到 0.7~1.0，或减小 D；若毛刺仍明显可降到 0.3~0.4。重力前馈 Roll 仍由 `PITCH_GRAVITY_ROLL_LPF_ALPHA=0.05` 独立滤波。
- 另注：分析期间检测到你下调了 `PITCH_GRAVITY_FF_MAX_VOLTAGE`（13519 → 8519），文档已同步标注；此改动与 D 修复无关，保留你的现值。

---

## 9. pitch.csv 现象与本轮修改（第四轮：疑似根因 = 起步助推继电器）

### 数据指纹（`Real_Robot_Gimbal/pitch.csv`，VOFA 10 ms/点，I1=Pitch 实际角 °）
- 目标 0° 保持时实际角出现**持续 ~3.3~3.6 Hz 极限环**，尾部（行 6459–6498）±1.1° 永不衰减；
- 不同区段幅度 ±0.3°~±6°，但频率基本不变；换 PID 参数幅度会变、**频率/不收敛特性不变** → 非线性极限环（与增益解耦）。

### 模板与本工程的结构性差异（能产生这种与 PID 无关的振荡）
| # | 模板 | 本工程(改动前) | 说明 |
|---|---|---|---|
| 1 | GM6020 输出=速度环 PID+前馈，无助推 | `gm6020.c` 起步助推：`|速度指令|≥1°/s 且 |输出|<8000 → 输出=±8000` | **bang-bang 继电器**：保持段速度指令常 >1，输出被强制成固定 ±8000 满力矩来回切换 → 周期由力矩/惯量决定（数 Hz），P/I/D 调不掉 |
| 2 | 速度环积分 `|e|>5 → I=0`（近似 PD） | 积分分离阈值 70，几乎全带积分（Ilim 12000） | 全带积分在 ~3Hz 相位滞后大，容易与 #1 一起维持极限环 |
| 3 | 位置环积分 `|e|>0.4° → I=0` | 分离=0（常积） | 同上，程度较轻 |
| 4 | 方向由模板机实测隐含 | 方向由宏给出：`PITCH_ENCODER_TO_IMU_SIGN=-1`、`PITCH_ROLL_RATE_TO_SPEED_SIGN=-1` 等 | 若任一根陀螺/编码器方向符号反了 = 负阻尼 → 会在机械谐振频率产生固定频率极限环，同样"调参无用"（需正负电流方向测试排除） |
| 5 | 前馈 k=13519（模板机标定） | 现值 8519 | 重力残余由积分兜底，数值错了只会偏置，一般不直接引起 3Hz 环 |

### 本轮修改
`Tasks/Inc/config.h`：`PITCH_STARTUP_MIN_VOLTAGE` 8000 → **0.0**（关闭起步助推继电器，等同模板"无助推"）。阈值宏保留未用。

### 下一步 A/B 与备选旋钮（按顺序做，每次只改一项并重录 pitch）
1. **关助推后复测**：若 3.3~3.6Hz 环消失/明显变小 → 确认根因；若阶跃起步发闷（静摩擦）再把助推开回来，但阈值 ≥10~15°/s、幅值 2000~4000，别用 1°/s+8000。
2. 若仍抖：把 `PITCH_SPEED_INTEGRAL_SEPARATION_RPM` 70 → 5（模板 dz=5 的近似），并考虑把"超出分离即清零"做到与模板一致（需改 MotorSpeedPid，小心 yaw 共用）。
3. 方向验证（排除负阻尼）：像 yaw 做过的正负电流试验，确认"正电流→编码器/IMU 角增方向"与三个符号宏一致；若反了，修正宏即可根治。
4. 位置到位死区 `PITCH_POSITION_DEADZONE_RAD=0.5°` 已定义但代码未引用——若只剩 ±1 计数齿隙抖动，可把它真正实现（误差 <0.3~0.5° 时位置环输出 0，靠前馈+速度环稳住）。

---

## 10. 第五轮：关助推后抖动变小，但加大 KP 抖动复发、D 压不住（相位裕度问题）

### 现象与机理
- 关掉起步助推继电器后，保持段极限环明显变小 → 确认上一轮结论；
- 随后把位置环 KP 与速度环 KP 都调大，抖动复现且内环 D/位置环 D 都“压不住”。
- 机理：位置环 KP 升高 → 外环开环增益升高、穿越频率上移；而外环相位预算被**全带积分（原分离 70）**与**速度反馈滤波（0.5）**持续吃掉 → 到某个 KP 后相位裕度变负起振。此时**内环（速度环）D 只能阻尼速度环自身**，对外环低中频振荡几乎无贡献；要压住它必须有**外环自己的阻尼**（位置环 D），或还回相位预算。

### 本轮改动（config.h，全部可逆）
| 项 | 原值 | 新值 | 意图 |
|---|---|---|---|
| `PITCH_ANGLE_KD_RPM_S_PER_RAD`（位置环 D） | 0 | **300** | 每拍差量语义下 ≈0.3 速度阻尼系数（模板 yaw 296.92 同量级），给外环加阻尼以支撑更高 KP |
| `PITCH_SPEED_INTEGRAL_SEPARATION_RPM` | 70 | **5** | 逼近模板 dz=5：外环穿越前把积分收窄，还相位裕度 |
| `PITCH_SPEED_LPF_ALPHA` | 0.50 | **0.80** | 折中：保留轻度滤波、把 D/外环相位滞后降到很小（模板=1.0 直采） |

### 验证方法
1. 重新烧录，先用你“会抖的那个 KP”，保持水平记录 pitch.csv。
2. 若抖动消失/明显受控：可继续小幅加 KP；若阶跃变软（接近速度变慢），把 `PITCH_ANGLE_KD` 降到 150~200。
3. 若仍抖：把 `PITCH_SPEED_LPF_ALPHA` 回调 **1.0**（彻底同模板直采）再测；仍不行把 KP 退回模板 22.4/194.44，抖动通常即消失（模板位置环 D=0 的前提就是 KP 不高）。
4. 回退说明：若回到模板 KP 整定组，可把 `PITCH_ANGLE_KD` 置回 0，三项改动不影响模板参数移植。

---

## 11. 遥控方向与 yaw–pitch 串扰（现场反馈修复）

### 问题 1：Pitch 遥控方向反了
- 现象：遥控器 pitch 杆“向上打”云台 pitch 反而向下。
- 改动：`config.h` 新增 `PITCH_STICK_DIR (-1.0f)`；`data_process.c` 的 pitch 增量按 `PITCH_STICK_DIR * COMMAND_STEP_RAD` 计算。换遥控器/改通道后又反了就把该宏改回 `+1.0f`。

### 问题 2：动 yaw 轴时 pitch 自行出力（pitch 目标不变、轴自行动 → 传感器耦合）
- 机理：pitch 速度环反馈用 BMI Roll(芯片 Z) 陀螺。若安装/轴系存在非正交残余，yaw(芯片 Y)转动会漏进 Z 轴读数 → pitch 速度环“以为 pitch 在动”而自行输出力矩。
- 改动（解耦）：
  - `config.h`：`PITCH_ROLL_YAW_CROSS_RATE`（默认 0 = 不补偿）；
  - `data_process.c`：从 roll 通道角速度减掉 `cross × yaw角速度`（`gx -= cross*gz`），干净值同时用于姿态估计与 pitch 速度环反馈；
  - 在线标定命令：`PITCH_YAW_CROSS=±0.05`（范围 ±0.5，典型 0.01~0.1）。
- 标定方法：pitch 锁水平（目标 0°），快速打 yaw 数次，观察 pitch 是否自行动/出力；从 0 增大 `|PITCH_YAW_CROSS|`，符号与幅值调到 pitch 不再动为止（符号不对就换号）。

### 问题 2 最终方案：yaw 遥控指令触发 pitch 锁存（无需标定）
利用"yaw/pitch 不会被同时遥控"这一事实，只在**收到 yaw 遥控指令**后锁存 pitch 输出，从源头切断串扰路径：
- 触发：消费到 `GIMBAL_MSG_YAW_DELTA`（yaw 指令步进）时把锁存窗口设到 `now + PITCH_LATCH_AFTER_YAW_CMD_MS(500ms)`；
- 未收到 yaw 指令 → `pitch_hold_until_ms=0`，pitch 全程正常响应（修复"正常打 pitch 被锁死"）；
- 锁存期间：GM6020 进入 `output_hold`，沿用进入时的电机输出，PID/滤波状态冻结；
- 解锁：yaw 基本到位（目标-轨迹 ≤ `PITCH_LATCH_YAW_SETTLED_DEG(0.5°)`、轨迹速度与实测转速很小）或 500ms 超时兜底；
- 离线/失权/故障/CAN 复位时一律解除锁存与窗口，防止旧输出复活。
- 说明：旧版用"yaw 是否在动"判定会被轨迹容差/转速噪声误判成一直运动 → pitch 锁死，已废弃。

改动文件：`Hardware_Drivers/Gm6020/Inc/gm6020.h`、`Src/gm6020.c`（output_hold/held_output/last_output）、`Hardware_Drivers/Can/Src/can_motor_bus.c`（复位清锁存）、`Tasks/Src/pid_calc.c`（yaw 指令触发锁存状态机）、`Tasks/Inc/config.h`（两个锁存宏）。

`PITCH_YAW_CROSS` 在线解耦保留为备选：若锁存后发现仍需在 yaw 运动时也让 pitch 参与校正，再改回用系数补偿。

---

## 12. Pitch 放开限幅 + 阶跃轨迹（目标 30°≤200ms / 超调≤0.2°）

### 放开内容（除积分外）
| 项 | 原值 | 新值 | 说明 |
|---|---|---|---|
| `PITCH_MAX_SPEED_RPM`（位置环输出速度上限） | 90 | 1200 | 仅数值安全兜底，实际速度由轨迹限制 |
| `PITCH_SOFT_LIMIT_DEG`（目标软限位） | ±90 | 0（不限） | 需配合 `pid_calc.c` 的 `DEG>0` 启用判断（否则 0 会把目标夹死在 home±0 → 遥控加的目标全被清 0，pitch 不动） |
| `PITCH_SPEED_OUTPUT_LIMIT` / `GM6020_VOLTAGE_LIMIT`（力矩饱和） | 25000 | 30000 | 驱动/GM6020 指令上限 |
| 位置环积分限 `30` / 速度环积分限 `12000` / 分离 `5` | 保持 | 保持 | 按需求不动 |

### 配套：Pitch 有限加速度轨迹（仿 yaw，`pid_calc.c`）
- 新增 `PITCH_TRAJECTORY_MAX_SPEED_RAD_S=8.0`、`PITCH_TRAJECTORY_MAX_ACCEL_RAD_S2=80.0`、`PITCH_TRAJ_VEL_FF_GAIN=1.0`；
- 位置环改跟踪 `pitch_profile_target`（从当前位置起步、有限加/减速驶向目标），并加速度前馈 `velFF`；
- 只放开限幅不加轨迹时，纯 P 阶跃超调只会更大、永远进不了 0.2°——轨迹才是同时满足“≤200ms + 超调≤0.2°”的关键；
- 回正/遥控 ±30° 阶跃都由同一轨迹平滑完成，不再有“起始大误差贯穿行程”的问题。

### 现场微调
- 超调 >0.2° → 加 `PITCH_ANGLE_KD`（300 起步）或降 `PITCH_TRAJECTORY_MAX_ACCEL_RAD_S2`；
- 到位 >200ms → 提高 `PITCH_TRAJECTORY_MAX_ACCEL_RAD_S2`（→100~120）或 `PITCH_TRAJECTORY_MAX_SPEED_RAD_S`（→10）；加速度前馈暂未用，若速度环跟不上再加。
