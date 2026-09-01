#include "gimbal_control.h"
#include "cmsis_os2.h"
#include "can_motor_bus.h"
#include "gm6020.h"
#include "dm4310.h"
#include "config.h"
#include <math.h>
#include <string.h>

extern osMessageQueueId_t Target_AngleHandle;
extern osMessageQueueId_t Update_PID_paraHandle;

#define DJI_ENCODER_COUNTS                8192
#define TWO_PI_F                          6.2831853071795864769f
#define PITCH_SOFT_LIMIT_RAD              (PITCH_SOFT_LIMIT_DEG * TASK_DEG_TO_RAD)
#define YAW_SOFT_LIMIT_RAD                (YAW_SOFT_LIMIT_DEG * TASK_DEG_TO_RAD)

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float output_limit;
    float previous_measurement;
    uint8_t initialized;
} PositionPid_t;

volatile GimbalControlState_t gimbal_control_state;

static float clampf(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float position_pid(PositionPid_t *pid, float target, float measurement,
                          float dt)
{
    float derivative = 0.0f;
    float error = target - measurement;
    if (pid->initialized != 0U)
        derivative = -(measurement - pid->previous_measurement) / dt;
    else
        pid->initialized = 1U;
    pid->previous_measurement = measurement;
    pid->integral = clampf(pid->integral + error * pid->ki * dt,
                           pid->integral_limit);
    return clampf(error * pid->kp + pid->integral + derivative * pid->kd,
                  pid->output_limit);
}

static float slew(float current, float target, float max_step)
{
    float difference = target - current;
    if (difference > max_step) difference = max_step;
    if (difference < -max_step) difference = -max_step;
    return current + difference;
}

void PID_calc(void *argument)
{
    TargetAngleMessage_t message;
    PidParameterUpdate_t parameter_update;
    PositionPid_t pitch_angle_pid = {
        PITCH_ANGLE_KP_RPM_PER_RAD, PITCH_ANGLE_KI_RPM_PER_RAD_S,
        PITCH_ANGLE_KD_RPM_S_PER_RAD, 0.0f, PITCH_ANGLE_INTEGRAL_LIMIT_RPM,
        PITCH_MAX_SPEED_RPM, 0.0f, 0U
    };
    PositionPid_t yaw_angle_pid = {
        YAW_ANGLE_KP_RAD_S_PER_RAD, YAW_ANGLE_KI_RAD_S_PER_RAD_S,
        YAW_ANGLE_KD_RAD_S2_PER_RAD, 0.0f, YAW_ANGLE_INTEGRAL_LIMIT_RAD_S,
        YAW_MAX_SPEED_RAD_S, 0.0f, 0U
    };
    int32_t pitch_total_count = 0;
    uint16_t pitch_previous_count = 0U;
    float pitch_encoder_filtered = 0.0f;
    float pitch_angle_actual = 0.0f;
    float pitch_encoder_offset = 0.0f;
    float yaw_angle_filtered = 0.0f;
    float yaw_speed_filtered = 0.0f;
    float pitch_target = 0.0f;
    float yaw_target = 0.0f;
    float yaw_profile_target = 0.0f;
    float pitch_home = 0.0f;
    float yaw_home = 0.0f;
    float imu_pitch = 0.0f;
    float pending_pitch_delta = 0.0f;
    float pending_yaw_delta = 0.0f;
    uint32_t last_remote_ms = 0U;
    uint32_t wake_tick;
    uint8_t encoder_initialized = 0U;
    uint8_t targets_initialized = 0U;
    uint8_t dm_enabled = 0U;
    uint8_t yaw_encoder_initialized = 0U;
    uint8_t remote_seen = 0U;
    uint8_t imu_initialized = 0U;
    (void)argument;
    memset((void *)&gimbal_control_state, 0, sizeof(gimbal_control_state));
    MotorSpeedPid_Init(&can1_gm6020_id2.speed_pid, PITCH_SPEED_KP,
                       PITCH_SPEED_KI, PITCH_SPEED_INTEGRAL_LIMIT,
                       PITCH_SPEED_OUTPUT_LIMIT);
    MotorSpeedPid_SetGains(&can1_gm6020_id2.speed_pid, PITCH_SPEED_KP,
                           PITCH_SPEED_KI, PITCH_SPEED_KD);
    GM6020_SetSpeedFilterAlpha(&can1_gm6020_id2, PITCH_SPEED_LPF_ALPHA);
    wake_tick = osKernelGetTickCount();

    for (;;)
    {
        while (osMessageQueueGet(Update_PID_paraHandle, &parameter_update,
                                 0, 0U) == osOK)
        {
            switch (parameter_update.id)
            {
                case PID_PARAM_KP_POS:
                    pitch_angle_pid.kp = parameter_update.value;
                    yaw_angle_pid.kp = parameter_update.value;
                    break;
                case PID_PARAM_KI_POS:
                    pitch_angle_pid.ki = parameter_update.value;
                    yaw_angle_pid.ki = parameter_update.value;
                    break;
                case PID_PARAM_KD_POS:
                    pitch_angle_pid.kd = parameter_update.value;
                    yaw_angle_pid.kd = parameter_update.value;
                    break;
                case PID_PARAM_KP_SPD:
                    can1_gm6020_id2.speed_pid.kp = parameter_update.value;
                    break;
                case PID_PARAM_KI_SPD:
                    can1_gm6020_id2.speed_pid.ki = parameter_update.value;
                    break;
                case PID_PARAM_KD_SPD:
                    can1_gm6020_id2.speed_pid.kd = parameter_update.value;
                    break;
                default:
                    break;
            }
        }
        while (osMessageQueueGet(Target_AngleHandle, &message, 0, 0U) == osOK)
        {
            if ((message.flags & GIMBAL_MSG_ATTITUDE) != 0U)
            {
                imu_pitch = message.pitch_rad;
                imu_initialized = 1U;
            }
            if ((message.flags & GIMBAL_MSG_REMOTE_OK) != 0U)
            {
                last_remote_ms = message.timestamp_ms;
                remote_seen = 1U;
            }
            if ((message.flags & GIMBAL_MSG_PITCH_DELTA) != 0U)
            {
                if (targets_initialized != 0U)
                    pitch_target += message.pitch_delta_rad;
                else
                    pending_pitch_delta += message.pitch_delta_rad;
            }
            if ((message.flags & GIMBAL_MSG_YAW_DELTA) != 0U)
            {
                if (targets_initialized != 0U)
                    yaw_target += message.yaw_delta_rad;
                else
                    pending_yaw_delta += message.yaw_delta_rad;
            }
        }

        if (can1_gm6020_id2.feedback.online != 0U)
        {
            uint16_t count = can1_gm6020_id2.feedback.encoder;
            if (encoder_initialized == 0U)
            {
                pitch_previous_count = count;
                pitch_total_count = (int32_t)count;
                pitch_encoder_filtered = PITCH_MOTOR_SIGN * (float)count *
                                         TWO_PI_F / (float)DJI_ENCODER_COUNTS;
                encoder_initialized = 1U;
            }
            else
            {
                int32_t delta = (int32_t)count - (int32_t)pitch_previous_count;
                if (delta > (DJI_ENCODER_COUNTS / 2)) delta -= DJI_ENCODER_COUNTS;
                if (delta < -(DJI_ENCODER_COUNTS / 2)) delta += DJI_ENCODER_COUNTS;
                pitch_total_count += delta;
                pitch_previous_count = count;
                pitch_encoder_filtered += PITCH_ENCODER_LPF_ALPHA *
                    (PITCH_MOTOR_SIGN * (float)pitch_total_count * TWO_PI_F /
                     (float)DJI_ENCODER_COUNTS - pitch_encoder_filtered);
            }
        }
        if (can2_dm4310_id1.online != 0U)
        {
            if (yaw_encoder_initialized == 0U)
            {
                yaw_angle_filtered = YAW_MOTOR_SIGN *
                                     can2_dm4310_id1.position_rad;
                yaw_speed_filtered = YAW_MOTOR_SIGN *
                                     can2_dm4310_id1.velocity_rad_s;
                yaw_encoder_initialized = 1U;
            }
            else
            {
                yaw_angle_filtered += YAW_ENCODER_LPF_ALPHA *
                    (YAW_MOTOR_SIGN * can2_dm4310_id1.position_rad -
                     yaw_angle_filtered);
                yaw_speed_filtered += YAW_SPEED_LPF_ALPHA *
                    (YAW_MOTOR_SIGN * can2_dm4310_id1.velocity_rad_s -
                     yaw_speed_filtered);
            }
        }

        if ((targets_initialized == 0U) && (encoder_initialized != 0U) &&
            (yaw_encoder_initialized != 0U) && (imu_initialized != 0U))
        {
            pitch_encoder_offset = pitch_encoder_filtered - imu_pitch;
            pitch_angle_actual = imu_pitch;
            pitch_home = 0.0f;
            yaw_home = yaw_angle_filtered;
            pitch_target = pending_pitch_delta;
            yaw_target = yaw_home + pending_yaw_delta;
            yaw_profile_target = yaw_target;
            targets_initialized = 1U;
        }
        else if (targets_initialized != 0U)
        {
            pitch_angle_actual = pitch_encoder_filtered - pitch_encoder_offset;
        }

        if (dm_enabled == 0U)
        {
            if (DM4310_Enable(&can2_dm4310_id1) == HAL_OK) dm_enabled = 1U;
        }

        if (targets_initialized != 0U)
        {
            if (pitch_target > pitch_home + PITCH_SOFT_LIMIT_RAD)
                pitch_target = pitch_home + PITCH_SOFT_LIMIT_RAD;
            if (pitch_target < pitch_home - PITCH_SOFT_LIMIT_RAD)
                pitch_target = pitch_home - PITCH_SOFT_LIMIT_RAD;
            if (yaw_target > yaw_home + YAW_SOFT_LIMIT_RAD)
                yaw_target = yaw_home + YAW_SOFT_LIMIT_RAD;
            if (yaw_target < yaw_home - YAW_SOFT_LIMIT_RAD)
                yaw_target = yaw_home - YAW_SOFT_LIMIT_RAD;
        }

        if ((targets_initialized != 0U) && (remote_seen != 0U) &&
            ((HAL_GetTick() - last_remote_ms) <= REMOTE_COMMAND_TIMEOUT_MS))
        {
            float pitch_speed_target;
            float yaw_feedback;
            float yaw_feedforward;
            float gravity_feedforward;
            float old_profile = yaw_profile_target;
            yaw_profile_target = slew(yaw_profile_target, yaw_target,
                YAW_TARGET_SLEW_RAD_S * CONTROL_PERIOD_S);
            yaw_feedforward = (yaw_profile_target - old_profile) /
                              CONTROL_PERIOD_S;
            yaw_feedback = position_pid(&yaw_angle_pid, yaw_profile_target,
                                        yaw_angle_filtered, CONTROL_PERIOD_S);
            pitch_speed_target = position_pid(&pitch_angle_pid, pitch_target,
                                               pitch_angle_actual,
                                               CONTROL_PERIOD_S);
            gravity_feedforward = PITCH_GRAVITY_SIGN *
                PITCH_GRAVITY_FF_MAX_VOLTAGE *
                cosf(imu_pitch - PITCH_GRAVITY_ZERO_RAD);
            GM6020_SetVoltageFeedforward(&can1_gm6020_id2,
                PITCH_MOTOR_SIGN * gravity_feedforward);
            GM6020_SetSpeed(&can1_gm6020_id2,
                           PITCH_MOTOR_SIGN * pitch_speed_target);
            can2_dm4310_id1.target_velocity_rad_s =
                YAW_MOTOR_SIGN * clampf(yaw_feedback + yaw_feedforward,
                                        YAW_MAX_SPEED_RAD_S);
            (void)CanMotorBus_Update(CONTROL_PERIOD_S);
            gimbal_control_state.active = 1U;
            gimbal_control_state.gravity_feedforward = gravity_feedforward;
        }
        else
        {
            GM6020_SetVoltageFeedforward(&can1_gm6020_id2, 0.0f);
            GM6020_SetSpeed(&can1_gm6020_id2, 0.0f);
            can2_dm4310_id1.target_velocity_rad_s = 0.0f;
            (void)CanMotorBus_StopAll();
            gimbal_control_state.active = 0U;
        }

        gimbal_control_state.pitch_target_rad = pitch_target;
        gimbal_control_state.yaw_target_rad = yaw_target;
        gimbal_control_state.pitch_encoder_rad = pitch_angle_actual;
        gimbal_control_state.yaw_encoder_rad = yaw_angle_filtered;
        gimbal_control_state.pitch_speed_rpm = PITCH_MOTOR_SIGN *
                                               can1_gm6020_id2.filtered_speed_rpm;
        gimbal_control_state.yaw_speed_rad_s = yaw_speed_filtered;
        wake_tick += CONTROL_PERIOD_TICKS;
        (void)osDelayUntil(wake_tick);
    }
}
