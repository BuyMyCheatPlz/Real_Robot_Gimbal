#include "gimbal_control.h"
#include "cmsis_os2.h"
#include "can_motor_bus.h"
#include "gm6020.h"
#include "dm4310.h"
#include "config.h"
#include "yaw_hold.h"
#include "yaw_startup.h"
#include <math.h>
#include <string.h>

extern osMessageQueueId_t Target_AngleHandle;
extern osMessageQueueId_t Update_PID_paraHandle;

#define DJI_ENCODER_COUNTS                8192
#define TWO_PI_F                          6.2831853071795864769f
#define RAD_S_TO_RPM                      (60.0f / TWO_PI_F)
#define RPM_TO_RAD_S                      (TWO_PI_F / 60.0f)
#define RAD_TO_DEG                        (1.0f / TASK_DEG_TO_RAD)
#define PITCH_CONTROL_TO_MOTOR_SIGN       \
    (PITCH_MOTOR_SIGN * PITCH_ENCODER_TO_IMU_SIGN)
#define PITCH_SOFT_LIMIT_RAD              (PITCH_SOFT_LIMIT_DEG * TASK_DEG_TO_RAD)
#define YAW_SOFT_LIMIT_RAD                (YAW_SOFT_LIMIT_DEG * TASK_DEG_TO_RAD)

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float integral_separation_error;
    float output_limit;
    /* D 项上一拍误差 e[k-1]。D = kd*(e[k]-e[k-1])（每 1 ms 一拍，不除以 dt），
     * 与模板 pid.c 语义一致，避免把 1 kHz 编码器差分放大 1/dt≈1000 倍。 */
    float previous_error;
    uint8_t initialized;
} PositionPid_t;

volatile GimbalControlState_t gimbal_control_state;
static volatile int16_t yaw_test_request_current;
static volatile uint32_t yaw_test_request_ms;

void Gimbal_YawTest_Request(int16_t current)
{
    uint32_t primask = __get_PRIMASK();
    if (current > YAW_DIRECTION_TEST_MAX_CURRENT)
        current = YAW_DIRECTION_TEST_MAX_CURRENT;
    if (current < -YAW_DIRECTION_TEST_MAX_CURRENT)
        current = -YAW_DIRECTION_TEST_MAX_CURRENT;
    __disable_irq();
    yaw_test_request_current = current;
    yaw_test_request_ms = HAL_GetTick();
    if (primask == 0U) __enable_irq();
}

static int16_t yaw_test_current_get(uint32_t now_ms, uint8_t *active)
{
    int16_t current;
    uint32_t request_ms;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    current = yaw_test_request_current;
    request_ms = yaw_test_request_ms;
    if (primask == 0U) __enable_irq();
    *active = (uint8_t)((current != 0) &&
        ((now_ms - request_ms) <= YAW_DIRECTION_TEST_DURATION_MS));
    return (*active != 0U) ? current : 0;
}

#if (YAW_SYSID_MODE != 0U)
/* ---------------- Yaw 系统辨识：线性扫频正弦激励 ----------------
 * identify_on 触发；f(t)=FREQ_START+(FREQ_END-FREQ_START)*t/时长。
 * 运行中 DM4310 直通该电流(旁路速度 PID)，打印 I6=指令 I7=原始速度。 */
static volatile uint8_t yaw_sysid_running;
static volatile uint32_t yaw_sysid_start_ms;
static float yaw_sysid_phase;

void Gimbal_YawSysid_Start(void)
{
    yaw_sysid_start_ms = HAL_GetTick();
    yaw_sysid_phase = 0.0f;
    yaw_sysid_running = 1U;
}

void Gimbal_YawSysid_Abort(void)
{
    yaw_sysid_running = 0U;
    yaw_sysid_phase = 0.0f;
}

uint8_t Gimbal_YawSysid_IsRunning(void)
{
    return yaw_sysid_running;
}

float Gimbal_YawSysid_ElapsedSeconds(void)
{
    if (yaw_sysid_running == 0U) return 0.0f;
    return (float)(HAL_GetTick() - yaw_sysid_start_ms) * 0.001f;
}

float Gimbal_YawSysid_Update(uint32_t now_ms, float dt_s)
{
    float elapsed_s;
    float duration_s;
    float fraction;
    float frequency_hz;
    float amplitude;
    float command;
    if (yaw_sysid_running == 0U) return 0.0f;
    elapsed_s = (float)(now_ms - yaw_sysid_start_ms) * 0.001f;
    duration_s = (float)YAW_SYSID_DURATION_MS * 0.001f;
    if (elapsed_s >= duration_s)
    {
        /* 时长到，自动停止，等待下一次 identify_on */
        yaw_sysid_running = 0U;
        yaw_sysid_phase = 0.0f;
        return 0.0f;
    }
    if (dt_s <= 0.0f) return 0.0f;
    fraction = elapsed_s / duration_s;
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    frequency_hz = YAW_SYSID_FREQ_START_HZ +
        (YAW_SYSID_FREQ_END_HZ - YAW_SYSID_FREQ_START_HZ) * fraction;
    yaw_sysid_phase += TWO_PI_F * frequency_hz * dt_s;
    while (yaw_sysid_phase > TWO_PI_F)
        yaw_sysid_phase -= TWO_PI_F;
    amplitude = YAW_SYSID_AMPLITUDE_CURRENT;
    if (amplitude > DM4310_CURRENT_COMMAND_LIMIT)
        amplitude = DM4310_CURRENT_COMMAND_LIMIT;
    command = amplitude * sinf(yaw_sysid_phase) * YAW_MOTOR_COMMAND_SIGN;
    return command;
}
#endif /* YAW_SYSID_MODE */

static float clampf(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float normalize_yaw_rad(float angle)
{
    while (angle >= (0.5f * TWO_PI_F)) angle -= TWO_PI_F;
    while (angle < -(0.5f * TWO_PI_F)) angle += TWO_PI_F;
    return angle;
}

static float yaw_angle_difference(float target, float measurement)
{
    return normalize_yaw_rad(target - measurement);
}

static float position_pid(PositionPid_t *pid, float target, float measurement,
                          float dt)
{
    float derivative = 0.0f;
    float error = target - measurement;
    float proportional_derivative;
    float candidate_integral;
    uint8_t integrate;
    /* D 项 = kd*(e[k]-e[k-1])，与模板 pid.c 一致（每拍误差差量，不除以 dt）。
     * 模板 pitch 位置环不用 D（=0），阻尼在速度环；若确实要位置阻尼，
     * 增益按“每拍差量”标定（模板 yaw 用 296.92 属此量级），
     * 不要用 -(meas-prev)/dt 的每秒导数语义，否则同数值放大 ~1000 倍。 */
    if (pid->initialized != 0U)
        derivative = error - pid->previous_error;
    else
        pid->initialized = 1U;
    pid->previous_error = error;
    proportional_derivative = error * pid->kp + derivative * pid->kd;
    integrate = (uint8_t)((pid->integral_separation_error <= 0.0f) ||
                          ((error <= pid->integral_separation_error) &&
                           (error >= -pid->integral_separation_error)));
    if (integrate != 0U)
    {
        candidate_integral = clampf(pid->integral + error * pid->ki * dt,
                                    pid->integral_limit);
        if ((((proportional_derivative + candidate_integral) < pid->output_limit) &&
             ((proportional_derivative + candidate_integral) > -pid->output_limit)) ||
            (((proportional_derivative + candidate_integral) >= pid->output_limit) &&
             (error < 0.0f)) ||
            (((proportional_derivative + candidate_integral) <= -pid->output_limit) &&
             (error > 0.0f)))
            pid->integral = candidate_integral;
    }
    return clampf(proportional_derivative + pid->integral, pid->output_limit);
}

static void reset_position_pid(PositionPid_t *pid)
{
    if (pid == 0) return;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->initialized = 0U;
}

static float signf(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

static float pitch_gravity_feedforward_scale(float pitch_rad)
{
    /* 本机 Pitch 坐标的正弦前馈：Pitch=0° 为参考零点。 */
    return PITCH_GRAVITY_SIGN *
           sinf(pitch_rad - PITCH_GRAVITY_ZERO_RAD);
}

/* 有限加速度轨迹同时提供速度和加速度参考，避免对不连续的位置阶跃求导。 */
static void yaw_trajectory_step(float *position, float *speed,
                                float *acceleration, float target,
                                float max_speed, float max_acceleration,
                                float dt)
{
    float distance;
    float direction;
    float previous_speed;
    float next_speed;
    float next_position;
    float stopping_distance;

    if ((position == 0) || (speed == 0) || (acceleration == 0) ||
        (max_speed <= 0.0f) || (max_acceleration <= 0.0f) || (dt <= 0.0f))
        return;

    distance = target - *position;
    direction = signf(distance);
    previous_speed = *speed;
    if (direction == 0.0f)
        *acceleration = -signf(*speed) * max_acceleration;
    else if ((*speed * direction) < 0.0f)
        *acceleration = direction * max_acceleration;
    else
    {
        stopping_distance = (*speed * *speed) / (2.0f * max_acceleration);
        *acceleration = (fabsf(distance) <= stopping_distance) ?
            -signf(*speed) * max_acceleration : direction * max_acceleration;
    }

    next_speed = *speed + *acceleration * dt;
    next_speed = clampf(next_speed, max_speed);
    next_position = *position + next_speed * dt;
    if ((direction != 0.0f) &&
        ((target - next_position) * direction < 0.0f))
    {
        next_position = target;
        next_speed = 0.0f;
    }
    *acceleration = (next_speed - previous_speed) / dt;
    *position = next_position;
    *speed = next_speed;
}

void PID_calc(void *argument)
{
    TargetAngleMessage_t message;
    PidParameterUpdate_t parameter_update;
    PositionPid_t pitch_angle_pid = {
        PITCH_ANGLE_KP_RPM_PER_RAD, PITCH_ANGLE_KI_RPM_PER_RAD_S,
        PITCH_ANGLE_KD_RPM_S_PER_RAD, 0.0f, PITCH_ANGLE_INTEGRAL_LIMIT_RPM,
        PITCH_ANGLE_INTEGRAL_SEPARATION_RAD * RAD_TO_DEG,
        PITCH_MAX_SPEED_RPM, 0.0f, 0U
    };
    PositionPid_t yaw_angle_pid = {
        YAW_ANGLE_KP_RAD_S_PER_RAD, YAW_ANGLE_KI_RAD_S_PER_RAD_S,
        YAW_ANGLE_KD_RAD_S2_PER_RAD, 0.0f, YAW_ANGLE_INTEGRAL_LIMIT_RAD_S,
        YAW_ANGLE_INTEGRAL_SEPARATION_RAD, YAW_MAX_SPEED_RAD_S, 0.0f, 0U
    };
    const YawHoldConfig_t yaw_hold_config = {
        YAW_HOLD_ENTER_ERROR_RAD,
        YAW_HOLD_EXIT_ERROR_RAD,
        YAW_HOLD_ENTER_SPEED_RPM,
        YAW_PROFILE_SETTLED_POSITION_RAD,
        YAW_PROFILE_SETTLED_SPEED_RAD_S
    };
    const YawStartupConfig_t yaw_startup_config = {
        YAW_STARTUP_SETTLE_TIME_MS,
        YAW_STARTUP_MAX_FEEDBACK_AGE_MS,
        YAW_STARTUP_MAX_SPEED_RPM,
        YAW_STARTUP_MAX_POSITION_DRIFT_RAD
    };
    YawHoldState_t yaw_hold_state = {0U};
    YawStartupState_t yaw_startup_state = {0U};
    int32_t pitch_total_count = 0;
    uint16_t pitch_previous_count = 0U;
    int32_t yaw_total_count = 0;
    uint16_t yaw_previous_count = 0U;
    float pitch_encoder_filtered = 0.0f;
    float pitch_angle_actual = 0.0f;
    float pitch_encoder_offset = 0.0f;
    float yaw_angle_filtered = 0.0f;
    float pitch_target = 0.0f;
    float yaw_target = 0.0f;
    float yaw_profile_target = 0.0f;
    float yaw_profile_speed = 0.0f;
    float yaw_profile_acceleration = 0.0f;
    float pitch_home = 0.0f;
    /* Pitch 阶跃轨迹状态(仿 yaw)：位置环跟踪有限加速度轨迹实现 30°/≤200ms/≤0.2° */
    float pitch_profile_target = 0.0f;
    float pitch_profile_speed = 0.0f;
    float pitch_profile_acceleration = 0.0f;
    float pitch_gravity_ff_voltage = PITCH_GRAVITY_FF_MAX_VOLTAGE;
    float pitch_trajectory_max_speed = PITCH_TRAJECTORY_MAX_SPEED_RAD_S;
    float pitch_trajectory_max_accel = PITCH_TRAJECTORY_MAX_ACCEL_RAD_S2;
    float pitch_max_speed_deg_s = PITCH_MAX_SPEED_RPM;
    float pitch_velocity_ff_gain = PITCH_TRAJ_VEL_FF_GAIN;
    float pitch_accel_ff_gain = PITCH_ACCEL_FF_VOLTAGE_PER_RAD_S2;
    float pitch_acceleration_ff = 0.0f;
    float pitch_motor_feedforward = 0.0f;
    float yaw_home = 0.0f;
    float imu_roll = 0.0f;
    float imu_yaw = 0.0f;
    float bmi_roll_rate_rad_s = 0.0f;
    float yaw_imu_filtered = 0.0f;
    float pending_pitch_delta = 0.0f;
    float pending_yaw_delta = 0.0f;
    uint32_t last_remote_ms = 0U;
    uint32_t last_imu_ms = 0U;
    uint32_t previous_control_ms;
    uint32_t now_ms;
    uint32_t wake_tick;
    uint32_t control_overrun_count = 0U;
    float control_dt_s = CONTROL_PERIOD_S;
    uint8_t pitch_encoder_initialized = 0U;
    uint8_t pitch_target_initialized = 0U;
    uint8_t yaw_target_initialized = 0U;
    uint8_t yaw_encoder_initialized = 0U;
    uint8_t remote_valid = 0U;
    uint8_t imu_initialized = 0U;
    uint8_t yaw_imu_filter_initialized = 0U;
    uint8_t overrun_pending = 0U;
    uint8_t pitch_hold_active = 0U;        /* yaw 运动期间锁存 pitch（两轴不同时指令） */
    float pitch_held_gravity_ff = 0.0f;    /* 保持期间用于遥测/恢复的前馈值 */
    uint32_t pitch_hold_until_ms = 0U;     /* 收到 yaw 指令后的锁存到期时刻(0=无锁存请求) */
    (void)argument;
    memset((void *)&gimbal_control_state, 0, sizeof(gimbal_control_state));
    MotorSpeedPid_Init(&can1_gm6020_id2.speed_pid, PITCH_SPEED_KP,
                       PITCH_SPEED_KI, PITCH_SPEED_INTEGRAL_LIMIT,
                       PITCH_SPEED_OUTPUT_LIMIT);
    MotorSpeedPid_SetGains(&can1_gm6020_id2.speed_pid, PITCH_SPEED_KP,
                           PITCH_SPEED_KI, PITCH_SPEED_KD);
    MotorSpeedPid_SetIntegralSeparation(&can1_gm6020_id2.speed_pid,
                                        PITCH_SPEED_INTEGRAL_SEPARATION_RPM);
    GM6020_SetSpeedFilterAlpha(&can1_gm6020_id2, PITCH_SPEED_LPF_ALPHA);
    MotorSpeedPid_Init(&can2_dm4310_id1.speed_pid,
                       YAW_SPEED_KP_CURRENT_PER_RPM,
                       YAW_SPEED_KI_CURRENT_PER_RPM_S,
                       YAW_SPEED_INTEGRAL_LIMIT_CURRENT,
                       YAW_CURRENT_OUTPUT_LIMIT);
    MotorSpeedPid_SetGains(&can2_dm4310_id1.speed_pid,
                           YAW_SPEED_KP_CURRENT_PER_RPM,
                           YAW_SPEED_KI_CURRENT_PER_RPM_S,
                           YAW_SPEED_KD_CURRENT_S_PER_RPM);
    MotorSpeedPid_SetIntegralSeparation(&can2_dm4310_id1.speed_pid,
                                        YAW_SPEED_INTEGRAL_SEPARATION_RPM);
    DM4310_SetSpeedFilterAlpha(&can2_dm4310_id1,
                               YAW_SPEED_LPF_ALPHA);
    wake_tick = osKernelGetTickCount();
    previous_control_ms = HAL_GetTick();

    for (;;)
    {
        uint32_t elapsed_ms;
        now_ms = HAL_GetTick();
        elapsed_ms = now_ms - previous_control_ms;
        previous_control_ms = now_ms;
        if (elapsed_ms == 0U)
            control_dt_s = CONTROL_PERIOD_S;
        else
            control_dt_s = (float)elapsed_ms * 0.001f;
        if (control_dt_s > CONTROL_MAX_DT_S)
        {
            control_dt_s = CONTROL_MAX_DT_S;
            overrun_pending = 1U;
            ++control_overrun_count;
        }
        while (osMessageQueueGet(Update_PID_paraHandle, &parameter_update,
                                 0, 0U) == osOK)
        {
            switch (parameter_update.id)
            {
                case PID_PARAM_PITCH_KP_POS:
                    pitch_angle_pid.kp = parameter_update.value;
                    break;
                case PID_PARAM_PITCH_KI_POS:
                    pitch_angle_pid.ki = parameter_update.value;
                    break;
                case PID_PARAM_PITCH_KD_POS:
                    pitch_angle_pid.kd = parameter_update.value;
                    break;
                case PID_PARAM_PITCH_KP_SPD:
                    can1_gm6020_id2.speed_pid.kp = parameter_update.value;
                    break;
                case PID_PARAM_PITCH_KI_SPD:
                    can1_gm6020_id2.speed_pid.ki = parameter_update.value;
                    break;
                case PID_PARAM_PITCH_KD_SPD:
                    can1_gm6020_id2.speed_pid.kd = parameter_update.value;
                    break;
                case PID_PARAM_YAW_KP_POS:
                    yaw_angle_pid.kp = parameter_update.value;
                    break;
                case PID_PARAM_YAW_KI_POS:
                    yaw_angle_pid.ki = parameter_update.value;
                    break;
                case PID_PARAM_YAW_KD_POS:
                    yaw_angle_pid.kd = parameter_update.value;
                    break;
                case PID_PARAM_YAW_KP_SPD:
                    can2_dm4310_id1.speed_pid.kp = parameter_update.value;
                    break;
                case PID_PARAM_YAW_KI_SPD:
                    can2_dm4310_id1.speed_pid.ki = parameter_update.value;
                    break;
                case PID_PARAM_YAW_KD_SPD:
                    can2_dm4310_id1.speed_pid.kd = parameter_update.value;
                    break;
                case PID_PARAM_PITCH_GRAVITY_FF:
                    pitch_gravity_ff_voltage = parameter_update.value;
                    if (pitch_gravity_ff_voltage > GM6020_VOLTAGE_LIMIT)
                        pitch_gravity_ff_voltage = GM6020_VOLTAGE_LIMIT;
                    break;
                case PID_PARAM_PITCH_TRAJ_SPEED:
                    if (parameter_update.value > 0.0f)
                        pitch_trajectory_max_speed = parameter_update.value;
                    break;
                case PID_PARAM_PITCH_TRAJ_ACCEL:
                    if (parameter_update.value > 0.0f)
                        pitch_trajectory_max_accel = parameter_update.value;
                    break;
                case PID_PARAM_PITCH_MAX_SPEED:
                    if (parameter_update.value > 0.0f)
                        pitch_max_speed_deg_s = parameter_update.value;
                    break;
                case PID_PARAM_PITCH_VEL_FF:
                    pitch_velocity_ff_gain = parameter_update.value;
                    break;
                case PID_PARAM_PITCH_ACCEL_FF:
                    pitch_accel_ff_gain = parameter_update.value;
                    break;
                default:
                    break;
            }
        }
        while (osMessageQueueGet(Target_AngleHandle, &message, 0, 0U) == osOK)
        {
            if ((message.flags & GIMBAL_MSG_ATTITUDE) != 0U)
            {
                imu_roll = message.roll_rad;
                imu_yaw = message.yaw_rad;
                bmi_roll_rate_rad_s = message.roll_rate_rad_s;
                if (yaw_imu_filter_initialized == 0U)
                {
                    yaw_imu_filtered = imu_yaw;
                    yaw_imu_filter_initialized = 1U;
                }
                else
                {
                    /* 对最短角度差进行滤波，避免 IMU Yaw 在 +/-pi 处跨界时被误认为
                     * 发生 360° 跳变。 */
                    yaw_imu_filtered = normalize_yaw_rad(yaw_imu_filtered +
                        YAW_IMU_POSITION_LPF_ALPHA *
                        yaw_angle_difference(imu_yaw, yaw_imu_filtered));
                }
                imu_initialized = 1U;
                last_imu_ms = now_ms;
                gimbal_control_state.imu_roll_rad = message.roll_rad;
                gimbal_control_state.imu_pitch_rad = message.pitch_rad;
                gimbal_control_state.imu_yaw_rad = message.yaw_rad;
                gimbal_control_state.imu_gyro_raw_x_rad_s =
                    message.gyro_raw_x_rad_s;
                gimbal_control_state.imu_gyro_raw_y_rad_s =
                    message.gyro_raw_y_rad_s;
                gimbal_control_state.imu_gyro_raw_z_rad_s =
                    message.gyro_raw_z_rad_s;
                gimbal_control_state.imu_roll_rate_rad_s =
                    message.roll_rate_rad_s;
                gimbal_control_state.imu_pitch_rate_rad_s =
                    message.pitch_rate_rad_s;
                gimbal_control_state.imu_yaw_rate_rad_s =
                    message.yaw_rate_rad_s;
            }
            if ((message.flags & GIMBAL_MSG_REMOTE_OK) != 0U)
            {
                last_remote_ms = message.timestamp_ms;
                remote_valid = 1U;
            }
            if ((message.flags & GIMBAL_MSG_REMOTE_BAD) != 0U)
                remote_valid = 0U;
            if ((message.flags & GIMBAL_MSG_PITCH_DELTA) != 0U)
            {
                if (pitch_target_initialized != 0U)
                    pitch_target += message.pitch_delta_rad;
                else
                    pending_pitch_delta += message.pitch_delta_rad;
            }
            if ((message.flags & GIMBAL_MSG_YAW_DELTA) != 0U)
            {
                if (yaw_target_initialized != 0U)
                {
                    yaw_target += message.yaw_delta_rad;
                    /* 收到 yaw 遥控指令 → 触发 pitch 锁存窗口（yaw/pitch 不同时指令） */
                    pitch_hold_until_ms = now_ms + PITCH_LATCH_AFTER_YAW_CMD_MS;
                }
                else
                    pending_yaw_delta += message.yaw_delta_rad;
            }
        }

        CanMotorBus_CheckOffline(now_ms);

        if (can1_gm6020_id2.feedback.online != 0U)
        {
            uint16_t count = can1_gm6020_id2.feedback.encoder;
            if (pitch_encoder_initialized == 0U)
            {
                pitch_previous_count = count;
                pitch_total_count = (int32_t)count;
                pitch_encoder_filtered = PITCH_ENCODER_TO_IMU_SIGN * (float)count *
                                         TWO_PI_F / (float)DJI_ENCODER_COUNTS;
                pitch_encoder_initialized = 1U;
            }
            else
            {
                int32_t delta = (int32_t)count - (int32_t)pitch_previous_count;
                if (delta > (DJI_ENCODER_COUNTS / 2)) delta -= DJI_ENCODER_COUNTS;
                if (delta < -(DJI_ENCODER_COUNTS / 2)) delta += DJI_ENCODER_COUNTS;
                pitch_total_count += delta;
                pitch_previous_count = count;
                pitch_encoder_filtered += PITCH_ENCODER_LPF_ALPHA *
                    (PITCH_ENCODER_TO_IMU_SIGN * (float)pitch_total_count * TWO_PI_F /
                     (float)DJI_ENCODER_COUNTS - pitch_encoder_filtered);
            }
        }
        if (can2_dm4310_id1.online != 0U)
        {
            uint16_t count = can2_dm4310_id1.encoder;
            if (yaw_encoder_initialized == 0U)
            {
                yaw_previous_count = count;
                yaw_total_count = (int32_t)count;
                yaw_angle_filtered = YAW_ENCODER_SIGN * (float)count *
                                     TWO_PI_F /
                                     ((float)DM4310_ENCODER_COUNTS *
                                      YAW_ENCODER_TO_OUTPUT_RATIO);
                yaw_encoder_initialized = 1U;
            }
            else
            {
                int32_t delta = (int32_t)count -
                                (int32_t)yaw_previous_count;
                if (delta > ((int32_t)DM4310_ENCODER_COUNTS / 2))
                    delta -= (int32_t)DM4310_ENCODER_COUNTS;
                if (delta < -((int32_t)DM4310_ENCODER_COUNTS / 2))
                    delta += (int32_t)DM4310_ENCODER_COUNTS;
                yaw_total_count += delta;
                yaw_previous_count = count;
                yaw_angle_filtered += YAW_ENCODER_LPF_ALPHA *
                    (YAW_ENCODER_SIGN * (float)yaw_total_count * TWO_PI_F /
                     ((float)DM4310_ENCODER_COUNTS *
                      YAW_ENCODER_TO_OUTPUT_RATIO) -
                     yaw_angle_filtered);
            }
        }

        {
            uint8_t imu_fresh = (uint8_t)((imu_initialized != 0U) &&
                ((now_ms - last_imu_ms) <= IMU_DATA_TIMEOUT_MS));
            uint8_t feedback_healthy = (uint8_t)(
                (can2_dm4310_id1.online != 0U) &&
                (((YAW_COMMISSIONING_MODE != 0U) ||
                  (can1_gm6020_id2.feedback.online != 0U))) &&
                (imu_fresh != 0U));
            uint8_t remote_fresh = (uint8_t)((remote_valid != 0U) &&
                ((now_ms - last_remote_ms) <= REMOTE_COMMAND_TIMEOUT_MS));
            uint8_t can_healthy = CanMotorBus_TxHealthy();
            uint8_t yaw_test_active;
            int16_t yaw_test_current = yaw_test_current_get(now_ms,
                                                              &yaw_test_active);
            uint8_t common_control_permitted = (uint8_t)(
                (imu_fresh != 0U) && (can_healthy != 0U) &&
                (overrun_pending == 0U) && (remote_fresh != 0U));
            uint8_t pitch_control_permitted = (uint8_t)(
                (common_control_permitted != 0U) &&
                (YAW_COMMISSIONING_MODE == 0U) &&
                (can1_gm6020_id2.feedback.online != 0U));
            uint8_t yaw_control_permitted = (uint8_t)(
                (common_control_permitted != 0U) &&
                (YAW_CLOSED_LOOP_ENABLE != 0U) &&
                (can2_dm4310_id1.online != 0U));
            uint32_t control_inhibit_flags = 0U;
            HAL_StatusTypeDef motor_status = HAL_ERROR;
            CanMotorBusStatus_t bus_status;
            float pitch_speed_target_rpm = 0.0f;
            float yaw_speed_target_rad_s = 0.0f;
            float yaw_velocity_feedforward = 0.0f;
            float yaw_acceleration_feedforward = 0.0f;

            if ((YAW_COMMISSIONING_MODE == 0U) &&
                (can1_gm6020_id2.feedback.online == 0U))
                control_inhibit_flags |= GIMBAL_INHIBIT_PITCH_OFFLINE;
            if (can2_dm4310_id1.online == 0U)
                control_inhibit_flags |= GIMBAL_INHIBIT_YAW_OFFLINE;
            if (imu_fresh == 0U)
                control_inhibit_flags |= GIMBAL_INHIBIT_IMU_STALE;
            if (remote_fresh == 0U)
                control_inhibit_flags |= GIMBAL_INHIBIT_REMOTE_STALE;
            if (can_healthy == 0U)
                control_inhibit_flags |= GIMBAL_INHIBIT_CAN_TX_FAULT;
            if (overrun_pending != 0U)
                control_inhibit_flags |= GIMBAL_INHIBIT_OVERRUN;

            if (remote_fresh == 0U)
            {
                pending_pitch_delta = 0.0f;
                pending_yaw_delta = 0.0f;
            }

            if (pitch_control_permitted == 0U)
            {
                pitch_target_initialized = 0U;
                pitch_encoder_initialized = 0U;
                reset_position_pid(&pitch_angle_pid);
                MotorSpeedPid_Reset(&can1_gm6020_id2.speed_pid);
                can1_gm6020_id2.output_hold = 0U;   /* 失去控制权即解除锁存 */
                pitch_hold_active = 0U;
                pitch_hold_until_ms = 0U;
                pitch_profile_speed = 0.0f;
                pitch_profile_acceleration = 0.0f;
            }
            if (yaw_control_permitted == 0U)
            {
                yaw_target_initialized = 0U;
                yaw_encoder_initialized = 0U;
                yaw_profile_speed = 0.0f;
                yaw_profile_acceleration = 0.0f;
                YawHold_Reset(&yaw_hold_state);
                YawStartup_Reset(&yaw_startup_state);
                reset_position_pid(&yaw_angle_pid);
                MotorSpeedPid_Reset(&can2_dm4310_id1.speed_pid);
            }

            if ((pitch_control_permitted != 0U) &&
                (pitch_target_initialized == 0U) &&
                (pitch_encoder_initialized != 0U))
            {
                /* 本机物理 Pitch 在控制链路中沿用历史变量名 imu_roll；
                 * 当前配置已把该量映射到 BMI088 raw X。上电用该角建立编码器
                 * 零偏；后续位置环和重力前馈使用连续编码器角。 */
                pitch_encoder_offset = pitch_encoder_filtered - imu_roll;
                pitch_angle_actual = imu_roll;
                pitch_home = 0.0f;
                pitch_target = pitch_home + pending_pitch_delta;
                pending_pitch_delta = 0.0f;
                /* 轨迹从当前位置起步，避免初始大误差造成的阶跃冲击 */
                pitch_profile_target = pitch_angle_actual;
                pitch_profile_speed = 0.0f;
                pitch_profile_acceleration = 0.0f;
                pitch_target_initialized = 1U;
            }
            else if (pitch_target_initialized != 0U)
            {
                pitch_angle_actual = pitch_encoder_filtered -
                                     pitch_encoder_offset;
            }

            if ((yaw_control_permitted != 0U) &&
                (yaw_target_initialized == 0U) &&
                (yaw_encoder_initialized != 0U) &&
                (YawStartup_Update(
                     &yaw_startup_state, &yaw_startup_config,
                     now_ms, can2_dm4310_id1.last_update_ms,
                     yaw_angle_filtered,
                     YAW_ENCODER_SIGN * can2_dm4310_id1.speed_rpm /
                     YAW_ENCODER_TO_OUTPUT_RATIO) != 0U))
            {
                /* 编码器是闭环测量值。若要让输出轴回到 IMU 相对 Yaw 零点，需将 IMU
                 * 误差转换到编码器参考系。两个符号已通过正负电流测试确认。 */
                if (YAW_HOME_TO_IMU_ZERO_ON_AUTHORIZE != 0U)
                    yaw_home = yaw_angle_filtered - yaw_imu_filtered;
                else
                    yaw_home = yaw_angle_filtered;
                yaw_target = yaw_home + pending_yaw_delta;
                yaw_profile_target = yaw_angle_filtered;
                yaw_profile_speed = 0.0f;
                yaw_profile_acceleration = 0.0f;
                pending_yaw_delta = 0.0f;
                YawHold_Reset(&yaw_hold_state);
                yaw_target_initialized = 1U;
            }

            if ((yaw_target_initialized != 0U) &&
                (YAW_SOFT_LIMIT_DEG > 0.0f))
            {
                if (yaw_target > yaw_home + YAW_SOFT_LIMIT_RAD)
                    yaw_target = yaw_home + YAW_SOFT_LIMIT_RAD;
                if (yaw_target < yaw_home - YAW_SOFT_LIMIT_RAD)
                    yaw_target = yaw_home - YAW_SOFT_LIMIT_RAD;
            }
            /* 软限位仅在 DEG>0 时生效；DEG=0 表示不限，避免把目标夹死在 home±0 */
            if ((pitch_target_initialized != 0U) &&
                (PITCH_SOFT_LIMIT_DEG > 0.0f))
            {
                if (pitch_target > pitch_home + PITCH_SOFT_LIMIT_RAD)
                    pitch_target = pitch_home + PITCH_SOFT_LIMIT_RAD;
                if (pitch_target < pitch_home - PITCH_SOFT_LIMIT_RAD)
                    pitch_target = pitch_home - PITCH_SOFT_LIMIT_RAD;
            }

#if (YAW_SYSID_MODE != 0U)
            if (can2_dm4310_id1.online != 0U)
            {
                /* ===== 辨识固件(YAW_SYSID_MODE=1)：pitch 无输出、yaw 不使用 PID =====
                 * 空闲时 DM4310 直通 0 电流(自由)；identify_on 后直通正弦扫频并打印。
                 * 宏=0 时本分支不编译，走下面的常规控制。 */
                float sysid_command;
                sysid_command = Gimbal_YawSysid_Update(now_ms, control_dt_s);
                /* Pitch：前馈 0、速度 0，位置/速度 PID 每周期复位，无输出 */
                reset_position_pid(&pitch_angle_pid);
                MotorSpeedPid_Reset(&can1_gm6020_id2.speed_pid);
                GM6020_SetVoltageFeedforward(&can1_gm6020_id2, 0.0f);
                GM6020_SetSpeed(&can1_gm6020_id2, 0.0f);
                /* Yaw：不使用速度 PID；DM4310 驱动层直通指令电流 */
                reset_position_pid(&yaw_angle_pid);
                MotorSpeedPid_Reset(&can2_dm4310_id1.speed_pid);
                can2_dm4310_id1.current_quantization_error = 0.0f;
                can2_dm4310_id1.direct_current_en = 1U;
                can2_dm4310_id1.direct_current = sysid_command;
                DM4310_SetCurrentFeedforward(&can2_dm4310_id1, 0);  /* 辨识：禁止电流前馈 */
                YawHold_Reset(&yaw_hold_state);
                yaw_profile_speed = 0.0f;
                yaw_profile_acceleration = 0.0f;
                motor_status = CanMotorBus_UpdateSelected(control_dt_s, 0U, 1U);
                if (motor_status != HAL_OK)
                    Gimbal_YawSysid_Abort();
                gimbal_control_state.sysid_running =
                    (uint8_t)(Gimbal_YawSysid_IsRunning() != 0U);
                gimbal_control_state.sysid_time_s =
                    Gimbal_YawSysid_ElapsedSeconds();
                gimbal_control_state.sysid_command = sysid_command;
                gimbal_control_state.sysid_speed_rpm =
                    can2_dm4310_id1.speed_rpm;   /* 原始反馈速度，不滤波 */
                gimbal_control_state.active =
                    gimbal_control_state.sysid_running;
                gimbal_control_state.gravity_feedforward = 0.0f;
            }
            else
            {
                /* DM4310 离线：中止辨识；pitch/yaw 断电，其余电机按常规停机 */
                Gimbal_YawSysid_Abort();
                reset_position_pid(&pitch_angle_pid);
                MotorSpeedPid_Reset(&can1_gm6020_id2.speed_pid);
                reset_position_pid(&yaw_angle_pid);
                MotorSpeedPid_Reset(&can2_dm4310_id1.speed_pid);
                can2_dm4310_id1.direct_current_en = 0U;
                can2_dm4310_id1.direct_current = 0.0f;
                GM6020_SetVoltageFeedforward(&can1_gm6020_id2, 0.0f);
                GM6020_SetSpeed(&can1_gm6020_id2, 0.0f);
                DM4310_SetSpeed(&can2_dm4310_id1, 0.0f);
                DM4310_SetCurrentFeedforward(&can2_dm4310_id1, 0);
                motor_status = CanMotorBus_StopGimbal(control_dt_s);
                gimbal_control_state.active = 0U;
                gimbal_control_state.gravity_feedforward = 0.0f;
                gimbal_control_state.sysid_running = 0U;
                gimbal_control_state.sysid_time_s = 0.0f;
                gimbal_control_state.sysid_command = 0.0f;
                gimbal_control_state.sysid_speed_rpm = 0.0f;
            }
#else
            /* ===== 常规电机输出选择：yaw 测试 / 闭环控制 / 停机 ===== */
            if ((yaw_test_active != 0U) &&
                (can2_dm4310_id1.online != 0U) && (can_healthy != 0U) &&
                (YAW_CLOSED_LOOP_ENABLE == 0U))
            {
                YawHold_Reset(&yaw_hold_state);
                reset_position_pid(&yaw_angle_pid);
                MotorSpeedPid_Reset(&can2_dm4310_id1.speed_pid);
                motor_status = CanMotorBus_SendYawTestCurrent(yaw_test_current);
            }
            else if ((pitch_target_initialized != 0U) ||
                     (yaw_target_initialized != 0U))
            {
                float gravity_feedforward = 0.0f;
                uint8_t was_holding;
                uint8_t yaw_holding;
                if (yaw_target_initialized != 0U)
                {
                    float yaw_feedback;
                    yaw_trajectory_step(&yaw_profile_target, &yaw_profile_speed,
                                        &yaw_profile_acceleration, yaw_target,
                                        YAW_TRAJECTORY_MAX_SPEED_RAD_S,
                                        YAW_TRAJECTORY_MAX_ACCEL_RAD_S2,
                                        control_dt_s);
                    yaw_velocity_feedforward = YAW_VELOCITY_FF_GAIN *
                        yaw_profile_speed;
                    yaw_acceleration_feedforward = clampf(
                        YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2 *
                        yaw_profile_acceleration,
                        YAW_ACCELERATION_FF_CURRENT_LIMIT);
                    was_holding = yaw_hold_state.active;
                    yaw_holding = YawHold_Update(
                        &yaw_hold_state, &yaw_hold_config,
                        yaw_target, yaw_profile_target, yaw_profile_speed,
                        yaw_angle_filtered,
                        YAW_ENCODER_SIGN *
                        can2_dm4310_id1.filtered_speed_rpm /
                        YAW_ENCODER_TO_OUTPUT_RATIO);
                    if (yaw_holding != 0U)
                    {
                        /* 进入保持状态时只重置一次。每毫秒重复重置会造成不连续的
                         * 刹车/重启极限环。 */
                        if (was_holding == 0U)
                        {
                            reset_position_pid(&yaw_angle_pid);
                            MotorSpeedPid_Reset(&can2_dm4310_id1.speed_pid);
                            can2_dm4310_id1.current_quantization_error = 0.0f;
                        }
                        yaw_feedback = 0.0f;
                        yaw_velocity_feedforward = 0.0f;
                        yaw_acceleration_feedforward = 0.0f;
                    }
                    else
                    {
                        if (was_holding != 0U)
                            reset_position_pid(&yaw_angle_pid);
                        yaw_feedback = position_pid(&yaw_angle_pid,
                                                     yaw_profile_target,
                                                     yaw_angle_filtered,
                                                     control_dt_s);
                    }
                    yaw_speed_target_rad_s = clampf(yaw_feedback +
                        yaw_velocity_feedforward, YAW_MAX_SPEED_RAD_S);
                    DM4310_SetSpeed(&can2_dm4310_id1,
                        YAW_MOTOR_COMMAND_SIGN * yaw_speed_target_rad_s *
                        RAD_S_TO_RPM * YAW_ENCODER_TO_OUTPUT_RATIO);
                    DM4310_SetCurrentFeedforward(&can2_dm4310_id1,
                        YAW_MOTOR_COMMAND_SIGN *
                        yaw_acceleration_feedforward);
                }
                if (pitch_target_initialized != 0U)
                {
                    /* 锁存触发：只响应“收到过 yaw 遥控指令”（pitch_hold_until_ms≠0）。
                     * 不再用“yaw 是否在动”推断——轨迹容差/转速噪声会误判成一直在动，
                     * 导致正常打 pitch 被锁死。解锁：yaw 基本到位或超时兜底。 */
                    uint8_t yaw_settled = 0U;
                    float yaw_meas_speed_rad_s =
                        YAW_ENCODER_SIGN *
                        can2_dm4310_id1.filtered_speed_rpm * RPM_TO_RAD_S;
                    if ((yaw_target_initialized != 0U) &&
                        (fabsf(yaw_target - yaw_profile_target) <=
                         (PITCH_LATCH_YAW_SETTLED_DEG * TASK_DEG_TO_RAD)) &&
                        (fabsf(yaw_profile_speed) <=
                         YAW_PROFILE_SETTLED_SPEED_RAD_S) &&
                        (fabsf(yaw_meas_speed_rad_s) <= 0.10f))
                        yaw_settled = 1U;
                    if ((pitch_hold_until_ms != 0U) &&
                        ((yaw_settled != 0U) ||
                         ((int32_t)(now_ms - pitch_hold_until_ms) >= 0)))
                        pitch_hold_until_ms = 0U;   /* 到位或超时 → 允许解锁 */

                    if (pitch_hold_until_ms != 0U)
                    {
                        /* yaw/pitch 不会同时被遥控指令。yaw 转动会漏进 Roll(物理
                         * pitch)通道，使 pitch 速度环误动；指令运动期间锁存 pitch：
                         * 沿用进入时的电机输出，PID/滤波状态冻结，到位后解锁。 */
                        if (pitch_hold_active == 0U)
                        {
                            pitch_hold_active = 1U;
                            can1_gm6020_id2.output_hold = 1U;
                            can1_gm6020_id2.held_output =
                                (int16_t)can1_gm6020_id2.last_output;
                            pitch_held_gravity_ff =
                                gimbal_control_state.gravity_feedforward;
                        }
                        gravity_feedforward = pitch_held_gravity_ff;
                    }
                    else
                    {
                        if (pitch_hold_active != 0U)
                        {
                            pitch_hold_active = 0U;
                            can1_gm6020_id2.output_hold = 0U;
                            /* 解锁：重置速度环，避免保持期与恢复期的速度/滤波
                             * 状态衔接时出现积分或微分跳变。 */
                            MotorSpeedPid_Reset(
                                &can1_gm6020_id2.speed_pid);
                            can1_gm6020_id2.speed_filter_initialized = 0U;
                        }
                        if (PITCH_GRAVITY_ONLY_ENABLE != 0U)
                        {
                            reset_position_pid(&pitch_angle_pid);
                            MotorSpeedPid_Reset(&can1_gm6020_id2.speed_pid);
                            pitch_speed_target_rpm = 0.0f;
                            pitch_profile_speed = 0.0f;
                            pitch_profile_acceleration = 0.0f;
                        }
                        else
                        {
                            /* 有限加速度轨迹 + 速度前馈(仿 yaw)：位置环跟踪轨迹而非
                             * 直接阶跃，保证 30° 阶跃 ≤200ms 且超调 ≤0.2°。 */
                            yaw_trajectory_step(&pitch_profile_target,
                                                &pitch_profile_speed,
                                                &pitch_profile_acceleration,
                                                pitch_target,
                                                fminf(
                                                    pitch_trajectory_max_speed,
                                                    pitch_max_speed_deg_s *
                                                    TASK_DEG_TO_RAD),
                                                pitch_trajectory_max_accel,
                                                control_dt_s);
                            pitch_speed_target_rpm =
                                position_pid(&pitch_angle_pid,
                                             pitch_profile_target * RAD_TO_DEG,
                                             pitch_angle_actual * RAD_TO_DEG,
                                             control_dt_s) +
                                pitch_velocity_ff_gain *
                                pitch_profile_speed * RAD_TO_DEG;
                            pitch_speed_target_rpm = clampf(
                                pitch_speed_target_rpm,
                                pitch_max_speed_deg_s);
                        }
                        /* 模板的合成是 PID - ff；本驱动固定做 PID + 前馈，
                         * 故这里传入 -ff。使用编码器 Pitch 角可避免 IMU 融合角和
                         * 额外低通在阶跃中滞后，把重力补偿打到错误相位。 */
                        gravity_feedforward = pitch_gravity_ff_voltage *
                            pitch_gravity_feedforward_scale(
                                pitch_angle_actual);
                        pitch_acceleration_ff = pitch_accel_ff_gain *
                            pitch_profile_acceleration;
                        pitch_motor_feedforward =
                            -PITCH_MOTOR_SIGN * gravity_feedforward +
                            PITCH_CONTROL_TO_MOTOR_SIGN *
                            pitch_acceleration_ff;
                        GM6020_SetVoltageFeedforward(&can1_gm6020_id2,
                            pitch_motor_feedforward);
                        GM6020_SetSpeed(&can1_gm6020_id2,
                                       PITCH_CONTROL_TO_MOTOR_SIGN *
                                       pitch_speed_target_rpm);
                        /* 模板使用物理轴的 IMU 角速度闭环。本机 Pitch 对应 Roll，
                         * 因而位置仍由编码器给出、速度由 Roll 陀螺给出。 */
                        can1_gm6020_id2.external_speed_rpm =
                            PITCH_ROLL_RATE_TO_SPEED_SIGN *
                            bmi_roll_rate_rad_s * RAD_TO_DEG;
                        can1_gm6020_id2.use_external_speed_feedback = 1U;
                    }
                }
                motor_status = CanMotorBus_UpdateSelected(
                    control_dt_s, pitch_target_initialized,
                    yaw_target_initialized);
                if (motor_status == HAL_OK)
                {
                    gimbal_control_state.active = 1U;
                    gimbal_control_state.gravity_feedforward =
                        gravity_feedforward;
                }
                else
                {
                    pitch_target_initialized = 0U;
                    yaw_target_initialized = 0U;
                    pitch_encoder_initialized = 0U;
                    yaw_encoder_initialized = 0U;
                    YawStartup_Reset(&yaw_startup_state);
                    can1_gm6020_id2.output_hold = 0U;   /* 故障下退出锁存 */
                    pitch_hold_active = 0U;
                    pitch_hold_until_ms = 0U;
                    pitch_profile_speed = 0.0f;
                    pitch_profile_acceleration = 0.0f;
                    gimbal_control_state.active = 0U;
                    gimbal_control_state.gravity_feedforward = 0.0f;
                }
            }
            else
            {
                GM6020_SetVoltageFeedforward(&can1_gm6020_id2, 0.0f);
                GM6020_SetSpeed(&can1_gm6020_id2, 0.0f);
                DM4310_SetSpeed(&can2_dm4310_id1, 0.0f);
                DM4310_SetCurrentFeedforward(&can2_dm4310_id1, 0);
                if ((remote_fresh == 0U) || (can_healthy == 0U) ||
                    (overrun_pending != 0U))
                    motor_status = CanMotorBus_StopAll();
                else
                    motor_status = CanMotorBus_StopGimbal(control_dt_s);
                if ((overrun_pending != 0U) &&
                    (motor_status == HAL_OK) &&
                    (CanMotorBus_TxHealthy() != 0U))
                    overrun_pending = 0U;
                gimbal_control_state.active = 0U;
                gimbal_control_state.gravity_feedforward = 0.0f;
            }
#endif /* YAW_SYSID_MODE */

            CanMotorBus_GetStatus(&bus_status);
            if (((pitch_control_permitted != 0U) &&
                 (pitch_target_initialized == 0U)) ||
                ((yaw_control_permitted != 0U) &&
                 (yaw_target_initialized == 0U)))
                control_inhibit_flags |= GIMBAL_INHIBIT_UNINITIALIZED;
            gimbal_control_state.feedback_healthy = feedback_healthy;
            gimbal_control_state.imu_fresh = imu_fresh;
            gimbal_control_state.can_tx_fault =
                (uint8_t)(CanMotorBus_TxHealthy() == 0U);
            gimbal_control_state.can_tx_failure_count =
                bus_status.total_tx_failures;
            gimbal_control_state.can_bus_error_count =
                bus_status.bus_error_count;
            gimbal_control_state.can1_busoff_count =
                bus_status.can1_busoff_count;
            gimbal_control_state.can2_busoff_count =
                bus_status.can2_busoff_count;
            gimbal_control_state.can1_recovery_count =
                bus_status.can1_recovery_count;
            gimbal_control_state.can2_recovery_count =
                bus_status.can2_recovery_count;
            gimbal_control_state.can1_last_error =
                bus_status.can1_last_error;
            gimbal_control_state.can2_last_error =
                bus_status.can2_last_error;
            gimbal_control_state.dm4310_feedback_count =
                bus_status.dm4310_feedback_count;
            gimbal_control_state.can2_tx_complete_count =
                bus_status.can2_tx_complete_count;
            gimbal_control_state.can2_tx_busy_count =
                bus_status.can2_tx_busy_count;
            gimbal_control_state.can1_rx_count =
                bus_status.can1_rx_count;
            gimbal_control_state.can2_rx_count =
                bus_status.can2_rx_count;
            gimbal_control_state.can2_last_rx_std_id =
                bus_status.last_can2_rx_std_id;
            gimbal_control_state.can_last_send_failure_mask =
                bus_status.last_send_failure_mask;
            gimbal_control_state.can1_tx_free_level =
                bus_status.can1_tx_free_level;
            gimbal_control_state.can2_tx_free_level =
                bus_status.can2_tx_free_level;
            gimbal_control_state.can1_m3508_id2_online =
                can1_m3508_id2.feedback.online;
            gimbal_control_state.can1_m3508_id3_online =
                can1_m3508_id3.feedback.online;
            gimbal_control_state.can1_gm6020_id2_online =
                can1_gm6020_id2.feedback.online;
            gimbal_control_state.can2_m2006_id5_online =
                can2_m2006_id5.feedback.online;
            gimbal_control_state.can2_dm4310_id1_online =
                can2_dm4310_id1.online;
            gimbal_control_state.control_overrun_count =
                control_overrun_count;
            gimbal_control_state.control_inhibit_flags =
                control_inhibit_flags;
            gimbal_control_state.pitch_can_command =
                bus_status.last_gm6020_id2_command;
            gimbal_control_state.yaw_can_command =
                bus_status.last_dm4310_id1_command;
            gimbal_control_state.yaw_torque_current_ma =
                can2_dm4310_id1.torque_current_ma;
            gimbal_control_state.yaw_encoder_count = can2_dm4310_id1.encoder;
            gimbal_control_state.yaw_motor_speed_rpm =
                can2_dm4310_id1.speed_rpm;
            gimbal_control_state.m3508_id2_speed_rpm =
                (float)can1_m3508_id2.feedback.speed_rpm;
            gimbal_control_state.m3508_id3_speed_rpm =
                (float)can1_m3508_id3.feedback.speed_rpm;
            gimbal_control_state.yaw_test_request_current = yaw_test_current;
            gimbal_control_state.yaw_test_active = yaw_test_active;
            gimbal_control_state.yaw_hold_active = yaw_hold_state.active;
            gimbal_control_state.remote_fresh = remote_fresh;
            gimbal_control_state.targets_initialized = (uint8_t)(
                (pitch_target_initialized != 0U) &&
                (yaw_target_initialized != 0U));
            gimbal_control_state.yaw_trajectory_speed_rad_s =
                yaw_profile_speed;
            gimbal_control_state.yaw_profile_target_rad =
                (yaw_target_initialized != 0U) ? yaw_profile_target :
                yaw_angle_filtered;
            gimbal_control_state.yaw_speed_target_rad_s =
                yaw_speed_target_rad_s;
            gimbal_control_state.yaw_velocity_feedforward_rad_s =
                (yaw_target_initialized != 0U) ?
                yaw_velocity_feedforward : 0.0f;
            gimbal_control_state.yaw_acceleration_feedforward_current =
                (yaw_target_initialized != 0U) ?
                yaw_acceleration_feedforward : 0.0f;
            gimbal_control_state.pitch_speed_target_rpm =
                pitch_speed_target_rpm;
            gimbal_control_state.pitch_profile_target_rad =
                pitch_profile_target;
            gimbal_control_state.pitch_speed_actual_deg_s =
                bmi_roll_rate_rad_s * RAD_TO_DEG;
            gimbal_control_state.pitch_feedback_output =
                can1_gm6020_id2.last_feedback_output;
            gimbal_control_state.pitch_motor_feedforward =
                can1_gm6020_id2.voltage_feedforward;
        }

        gimbal_control_state.pitch_gravity_ff_setting =
            pitch_gravity_ff_voltage;
        gimbal_control_state.pitch_target_rad = pitch_target;
        gimbal_control_state.yaw_target_rad = yaw_target;
        gimbal_control_state.pitch_encoder_rad = pitch_angle_actual;
        gimbal_control_state.yaw_encoder_rad = yaw_angle_filtered;
        gimbal_control_state.yaw_imu_actual_rad = yaw_imu_filtered;
        gimbal_control_state.pitch_speed_rpm = PITCH_MOTOR_SIGN *
                                               can1_gm6020_id2.filtered_speed_rpm;
        gimbal_control_state.pitch_motor_speed_rpm =
            (float)can1_gm6020_id2.feedback.speed_rpm;
        gimbal_control_state.pitch_imu_speed_rpm =
            PITCH_ROLL_RATE_TO_SPEED_SIGN *
            bmi_roll_rate_rad_s * RAD_TO_DEG;
        gimbal_control_state.yaw_speed_rad_s =
            (can2_dm4310_id1.online != 0U) ?
            YAW_ENCODER_SIGN * can2_dm4310_id1.filtered_speed_rpm *
            RPM_TO_RAD_S : 0.0f;
        ++gimbal_control_state.pid_heartbeat;
        wake_tick += CONTROL_PERIOD_TICKS;
        if (osDelayUntil(wake_tick) != osOK)
        {
            overrun_pending = 1U;
            ++control_overrun_count;
            wake_tick = osKernelGetTickCount();
        }
    }
}
