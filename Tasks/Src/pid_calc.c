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
#define RAD_S_TO_RPM                      (60.0f / TWO_PI_F)
#define RPM_TO_RAD_S                      (TWO_PI_F / 60.0f)
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
    float proportional_derivative;
    float candidate_integral;
    uint8_t integrate;
    if (pid->initialized != 0U)
        derivative = -(measurement - pid->previous_measurement) / dt;
    else
        pid->initialized = 1U;
    pid->previous_measurement = measurement;
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
    pid->previous_measurement = 0.0f;
    pid->initialized = 0U;
}

static float signf(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

/* A finite-acceleration trajectory exposes both velocity and acceleration
 * references without differentiating a discontinuous position step. */
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
        PITCH_ANGLE_INTEGRAL_SEPARATION_RAD, PITCH_MAX_SPEED_RPM, 0.0f, 0U
    };
    PositionPid_t yaw_angle_pid = {
        YAW_ANGLE_KP_RAD_S_PER_RAD, YAW_ANGLE_KI_RAD_S_PER_RAD_S,
        YAW_ANGLE_KD_RAD_S2_PER_RAD, 0.0f, YAW_ANGLE_INTEGRAL_LIMIT_RAD_S,
        YAW_ANGLE_INTEGRAL_SEPARATION_RAD, YAW_MAX_SPEED_RAD_S, 0.0f, 0U
    };
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
    float yaw_home = 0.0f;
    float imu_pitch = 0.0f;
    float pending_pitch_delta = 0.0f;
    float pending_yaw_delta = 0.0f;
    uint32_t last_remote_ms = 0U;
    uint32_t last_imu_ms = 0U;
    uint32_t previous_control_ms;
    uint32_t now_ms;
    uint32_t wake_tick;
    uint32_t control_overrun_count = 0U;
    float control_dt_s = CONTROL_PERIOD_S;
    uint8_t encoder_initialized = 0U;
    uint8_t targets_initialized = 0U;
    uint8_t yaw_encoder_initialized = 0U;
    uint8_t remote_valid = 0U;
    uint8_t imu_initialized = 0U;
    uint8_t overrun_pending = 0U;
    uint8_t ever_armed = 0U;
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
                last_imu_ms = now_ms;
                gimbal_control_state.imu_roll_rad = message.roll_rad;
                gimbal_control_state.imu_pitch_rad = message.pitch_rad;
                gimbal_control_state.imu_yaw_rad = message.yaw_rad;
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

        CanMotorBus_CheckOffline(now_ms);

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
            uint16_t count = can2_dm4310_id1.encoder;
            if (yaw_encoder_initialized == 0U)
            {
                yaw_previous_count = count;
                yaw_total_count = (int32_t)count;
                yaw_angle_filtered = YAW_MOTOR_SIGN * (float)count *
                                     TWO_PI_F /
                                     (float)DM4310_ENCODER_COUNTS;
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
                    (YAW_MOTOR_SIGN * (float)yaw_total_count * TWO_PI_F /
                     (float)DM4310_ENCODER_COUNTS -
                     yaw_angle_filtered);
            }
        }

        {
            uint8_t imu_fresh = (uint8_t)((imu_initialized != 0U) &&
                ((now_ms - last_imu_ms) <= IMU_DATA_TIMEOUT_MS));
            uint8_t feedback_healthy = (uint8_t)(
                (can1_gm6020_id2.feedback.online != 0U) &&
                (can2_dm4310_id1.online != 0U) && (imu_fresh != 0U));
            uint8_t remote_fresh = (uint8_t)((remote_valid != 0U) &&
                ((now_ms - last_remote_ms) <= REMOTE_COMMAND_TIMEOUT_MS));
            uint8_t can_healthy = CanMotorBus_TxHealthy();
            uint8_t control_permitted = (uint8_t)(
                (feedback_healthy != 0U) && (remote_fresh != 0U) &&
                (can_healthy != 0U) && (overrun_pending == 0U));
            HAL_StatusTypeDef motor_status = HAL_ERROR;
            CanMotorBusStatus_t bus_status;
            float yaw_velocity_feedforward = 0.0f;
            float yaw_acceleration_feedforward = 0.0f;

            if (control_permitted == 0U)
            {
                targets_initialized = 0U;
                encoder_initialized = 0U;
                yaw_encoder_initialized = 0U;
                pending_pitch_delta = 0.0f;
                pending_yaw_delta = 0.0f;
                yaw_profile_speed = 0.0f;
                yaw_profile_acceleration = 0.0f;
                reset_position_pid(&pitch_angle_pid);
                reset_position_pid(&yaw_angle_pid);
                MotorSpeedPid_Reset(&can1_gm6020_id2.speed_pid);
                MotorSpeedPid_Reset(&can2_dm4310_id1.speed_pid);
            }
            else if ((targets_initialized == 0U) &&
                     (encoder_initialized != 0U) &&
                     (yaw_encoder_initialized != 0U))
            {
                pitch_encoder_offset = pitch_encoder_filtered - imu_pitch;
                pitch_angle_actual = imu_pitch;
                pitch_home = 0.0f;
                yaw_home = yaw_angle_filtered;
                pitch_target = (ever_armed == 0U) ? pitch_home :
                               pitch_angle_actual;
                yaw_target = yaw_home;
                yaw_profile_target = yaw_home;
                yaw_profile_speed = 0.0f;
                yaw_profile_acceleration = 0.0f;
                pending_pitch_delta = 0.0f;
                pending_yaw_delta = 0.0f;
                targets_initialized = 1U;
                ever_armed = 1U;
            }
            else if (targets_initialized != 0U)
            {
                pitch_angle_actual = pitch_encoder_filtered -
                                     pitch_encoder_offset;
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

            if ((control_permitted != 0U) &&
                (targets_initialized != 0U))
            {
                float pitch_speed_target;
                float yaw_feedback;
                float yaw_speed_target_rad_s;
                float gravity_feedforward;
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
                yaw_feedback = position_pid(&yaw_angle_pid,
                                             yaw_profile_target,
                                             yaw_angle_filtered,
                                             control_dt_s);
                pitch_speed_target = position_pid(&pitch_angle_pid,
                                                   pitch_target,
                                                   pitch_angle_actual,
                                                   control_dt_s);
                gravity_feedforward = PITCH_GRAVITY_SIGN *
                    PITCH_GRAVITY_FF_MAX_VOLTAGE *
                    cosf(imu_pitch - PITCH_GRAVITY_ZERO_RAD);
                GM6020_SetVoltageFeedforward(&can1_gm6020_id2,
                    PITCH_MOTOR_SIGN * gravity_feedforward);
                GM6020_SetSpeed(&can1_gm6020_id2,
                               PITCH_MOTOR_SIGN * pitch_speed_target);
                yaw_speed_target_rad_s = clampf(yaw_feedback +
                    yaw_velocity_feedforward, YAW_MAX_SPEED_RAD_S);
                DM4310_SetSpeed(&can2_dm4310_id1,
                                YAW_MOTOR_SIGN * yaw_speed_target_rad_s *
                                RAD_S_TO_RPM);
                DM4310_SetCurrentFeedforward(&can2_dm4310_id1,
                    (int16_t)(YAW_MOTOR_SIGN *
                              yaw_acceleration_feedforward));
                motor_status = CanMotorBus_Update(control_dt_s);
                if (motor_status == HAL_OK)
                {
                    gimbal_control_state.active = 1U;
                    gimbal_control_state.gravity_feedforward =
                        gravity_feedforward;
                }
                else
                {
                    targets_initialized = 0U;
                    encoder_initialized = 0U;
                    yaw_encoder_initialized = 0U;
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

            CanMotorBus_GetStatus(&bus_status);
            gimbal_control_state.feedback_healthy = feedback_healthy;
            gimbal_control_state.imu_fresh = imu_fresh;
            gimbal_control_state.can_tx_fault =
                (uint8_t)(CanMotorBus_TxHealthy() == 0U);
            gimbal_control_state.can_tx_failure_count =
                bus_status.total_tx_failures;
            gimbal_control_state.control_overrun_count =
                control_overrun_count;
            gimbal_control_state.yaw_trajectory_speed_rad_s =
                yaw_profile_speed;
            gimbal_control_state.yaw_velocity_feedforward_rad_s =
                (targets_initialized != 0U) ? yaw_velocity_feedforward : 0.0f;
            gimbal_control_state.yaw_acceleration_feedforward_current =
                (targets_initialized != 0U) ? yaw_acceleration_feedforward : 0.0f;
        }

        gimbal_control_state.pitch_target_rad = pitch_target;
        gimbal_control_state.yaw_target_rad = yaw_target;
        gimbal_control_state.pitch_encoder_rad = pitch_angle_actual;
        gimbal_control_state.yaw_encoder_rad = yaw_angle_filtered;
        gimbal_control_state.pitch_speed_rpm = PITCH_MOTOR_SIGN *
                                               can1_gm6020_id2.filtered_speed_rpm;
        gimbal_control_state.yaw_speed_rad_s =
            (can2_dm4310_id1.online != 0U) ?
            YAW_MOTOR_SIGN * can2_dm4310_id1.filtered_speed_rpm *
            RPM_TO_RAD_S : 0.0f;
        wake_tick += CONTROL_PERIOD_TICKS;
        if (osDelayUntil(wake_tick) != osOK)
        {
            overrun_pending = 1U;
            ++control_overrun_count;
            wake_tick = osKernelGetTickCount();
        }
    }
}
