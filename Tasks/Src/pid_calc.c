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
    float yaw_home = 0.0f;
    float imu_pitch = 0.0f;
    float imu_yaw = 0.0f;
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
                default:
                    break;
            }
        }
        while (osMessageQueueGet(Target_AngleHandle, &message, 0, 0U) == osOK)
        {
            if ((message.flags & GIMBAL_MSG_ATTITUDE) != 0U)
            {
                imu_pitch = message.pitch_rad;
                imu_yaw = message.yaw_rad;
                if (yaw_imu_filter_initialized == 0U)
                {
                    yaw_imu_filtered = imu_yaw;
                    yaw_imu_filter_initialized = 1U;
                }
                else
                {
                    /* Filter the shortest angular difference so an IMU yaw
                     * wrap at +/-pi cannot look like a 360-degree jump. */
                    yaw_imu_filtered = normalize_yaw_rad(yaw_imu_filtered +
                        YAW_IMU_POSITION_LPF_ALPHA *
                        yaw_angle_difference(imu_yaw, yaw_imu_filtered));
                }
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
                if (pitch_target_initialized != 0U)
                    pitch_target += message.pitch_delta_rad;
                else
                    pending_pitch_delta += message.pitch_delta_rad;
            }
            if ((message.flags & GIMBAL_MSG_YAW_DELTA) != 0U)
            {
                if (yaw_target_initialized != 0U)
                    yaw_target += message.yaw_delta_rad;
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
                pitch_encoder_filtered = PITCH_MOTOR_SIGN * (float)count *
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
                (imu_fresh != 0U) && (remote_fresh != 0U) &&
                (can_healthy != 0U) && (overrun_pending == 0U));
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
                pitch_encoder_offset = pitch_encoder_filtered - imu_pitch;
                pitch_angle_actual = imu_pitch;
                pitch_home = 0.0f;
                pitch_target = pitch_home + pending_pitch_delta;
                pending_pitch_delta = 0.0f;
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
                /* Encoder is the closed-loop measurement.  To return the
                 * output axis to the IMU's relative yaw zero, transform that
                 * IMU error into the encoder reference frame.  The two signs
                 * were validated by the +/− current tests. */
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

            if (pitch_target_initialized != 0U)
            {
                if (pitch_target > pitch_home + PITCH_SOFT_LIMIT_RAD)
                    pitch_target = pitch_home + PITCH_SOFT_LIMIT_RAD;
                if (pitch_target < pitch_home - PITCH_SOFT_LIMIT_RAD)
                    pitch_target = pitch_home - PITCH_SOFT_LIMIT_RAD;
            }
            if (yaw_target_initialized != 0U)
            {
                if (yaw_target > yaw_home + YAW_SOFT_LIMIT_RAD)
                    yaw_target = yaw_home + YAW_SOFT_LIMIT_RAD;
                if (yaw_target < yaw_home - YAW_SOFT_LIMIT_RAD)
                    yaw_target = yaw_home - YAW_SOFT_LIMIT_RAD;
            }

            if ((yaw_test_active != 0U) && (remote_fresh != 0U) &&
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
                    float yaw_speed_target_rad_s;
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
                        /* Reset once on entry.  Repeating this every
                         * millisecond creates a discontinuous brake/restart
                         * limit cycle. */
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
                    float pitch_speed_target = position_pid(
                        &pitch_angle_pid, pitch_target, pitch_angle_actual,
                        control_dt_s);
                    gravity_feedforward = PITCH_GRAVITY_SIGN *
                        PITCH_GRAVITY_FF_MAX_VOLTAGE *
                        cosf(imu_pitch - PITCH_GRAVITY_ZERO_RAD);
                    GM6020_SetVoltageFeedforward(&can1_gm6020_id2,
                        PITCH_MOTOR_SIGN * gravity_feedforward);
                    GM6020_SetSpeed(&can1_gm6020_id2,
                                   PITCH_MOTOR_SIGN * pitch_speed_target);
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
            gimbal_control_state.dm4310_feedback_count =
                bus_status.dm4310_feedback_count;
            gimbal_control_state.can2_tx_complete_count =
                bus_status.can2_tx_complete_count;
            gimbal_control_state.can2_tx_busy_count =
                bus_status.can2_tx_busy_count;
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
            gimbal_control_state.yaw_test_request_current = yaw_test_current;
            gimbal_control_state.yaw_test_active = yaw_test_active;
            gimbal_control_state.yaw_hold_active = yaw_hold_state.active;
            gimbal_control_state.remote_fresh = remote_fresh;
            gimbal_control_state.targets_initialized = (uint8_t)(
                (pitch_target_initialized != 0U) &&
                (yaw_target_initialized != 0U));
            gimbal_control_state.yaw_trajectory_speed_rad_s =
                yaw_profile_speed;
            gimbal_control_state.yaw_velocity_feedforward_rad_s =
                (yaw_target_initialized != 0U) ?
                yaw_velocity_feedforward : 0.0f;
            gimbal_control_state.yaw_acceleration_feedforward_current =
                (yaw_target_initialized != 0U) ?
                yaw_acceleration_feedforward : 0.0f;
        }

        gimbal_control_state.pitch_target_rad = pitch_target;
        gimbal_control_state.yaw_target_rad = yaw_target;
        gimbal_control_state.pitch_encoder_rad = pitch_angle_actual;
        gimbal_control_state.yaw_encoder_rad = yaw_angle_filtered;
        gimbal_control_state.yaw_imu_actual_rad = yaw_imu_filtered;
        gimbal_control_state.pitch_speed_rpm = PITCH_MOTOR_SIGN *
                                               can1_gm6020_id2.filtered_speed_rpm;
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
