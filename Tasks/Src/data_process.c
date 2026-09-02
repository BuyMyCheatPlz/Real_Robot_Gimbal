#include "gimbal_control.h"
#include "cmsis_os2.h"
#include "hardware_drivers.h"
#include "can_motor_bus.h"
#include "sbus.h"
#include "vofa.h"
#include "config.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

extern osMessageQueueId_t Target_AngleHandle;
extern osMessageQueueId_t Update_PID_paraHandle;
extern osMessageQueueId_t Update_launch_paraHandle;
extern osSemaphoreId_t wake_launchHandle;
extern osSemaphoreId_t wake_launch_motorHandle;

#define COMMAND_STEP_RAD             (GIMBAL_COMMAND_STEP_DEG * TASK_DEG_TO_RAD)

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    uint32_t last_sample_ms;
    uint32_t sample_count;
    uint8_t initialized;
} AttitudeEstimator_t;

static uint8_t channel_changed(float current, float previous)
{
    return (uint8_t)(fabsf(current - previous) >= REMOTE_EDGE_THRESHOLD);
}

static void queue_latest(const TargetAngleMessage_t *message)
{
    TargetAngleMessage_t discarded;
    if (osMessageQueuePut(Target_AngleHandle, message, 0U, 0U) != osOK)
    {
        (void)osMessageQueueGet(Target_AngleHandle, &discarded, 0, 0U);
        (void)osMessageQueuePut(Target_AngleHandle, message, 0U, 0U);
    }
}

static uint8_t parse_pid_command(const char *command,
                                 PidParameterUpdate_t *update)
{
    const char *equals;
    char *end;
    size_t key_length;
    if ((command == 0) || (update == 0)) return 0U;
    equals = strchr(command, '=');
    if (equals == 0) return 0U;
    key_length = (size_t)(equals - command);
    if ((key_length == 6U) && (strncmp(command, "KP_POS", 6U) == 0))
        update->id = PID_PARAM_KP_POS;
    else if ((key_length == 6U) && (strncmp(command, "KI_POS", 6U) == 0))
        update->id = PID_PARAM_KI_POS;
    else if ((key_length == 6U) && (strncmp(command, "KD_POS", 6U) == 0))
        update->id = PID_PARAM_KD_POS;
    else if ((key_length == 6U) && (strncmp(command, "KP_SPD", 6U) == 0))
        update->id = PID_PARAM_KP_SPD;
    else if ((key_length == 6U) && (strncmp(command, "KI_SPD", 6U) == 0))
        update->id = PID_PARAM_KI_SPD;
    else if ((key_length == 6U) && (strncmp(command, "KD_SPD", 6U) == 0))
        update->id = PID_PARAM_KD_SPD;
    else
        return 0U;

    update->value = strtof(equals + 1, &end);
    if (end == (equals + 1)) return 0U;
    while ((*end == ' ') || (*end == '\t')) ++end;
    if ((*end != '\0') || (isfinite(update->value) == 0) ||
        (update->value < 0.0f) || (update->value > ONLINE_PID_VALUE_MAX))
        return 0U;
    return 1U;
}

static void process_vofa_commands(void)
{
    char command[VOFA_COMMAND_MAX_LENGTH];
    PidParameterUpdate_t update;
    PidParameterUpdate_t discarded;
    while (VOFA_GetCommand(command) != 0U)
    {
        if (parse_pid_command(command, &update) == 0U) continue;
        if (osMessageQueuePut(Update_PID_paraHandle, &update, 0U, 0U) != osOK)
        {
            (void)osMessageQueueGet(Update_PID_paraHandle, &discarded, 0, 0U);
            (void)osMessageQueuePut(Update_PID_paraHandle, &update, 0U, 0U);
        }
    }
}

static float map_sbus_speed(uint16_t channel, float maximum_rpm)
{
    if (channel <= SBUS_CONTROL_MIN) return 0.0f;
    if (channel >= SBUS_CONTROL_MAX) return maximum_rpm;
    return ((float)(channel - SBUS_CONTROL_MIN) /
            (float)(SBUS_CONTROL_MAX - SBUS_CONTROL_MIN)) * maximum_rpm;
}

static void publish_launch_parameters(const SBusData_t *remote,
                                      uint8_t remote_ok)
{
    LaunchParameterUpdate_t update;
    LaunchParameterUpdate_t discarded;
    memset(&update, 0, sizeof(update));
    update.timestamp_ms = HAL_GetTick();
    if (remote_ok != 0U)
    {
        update.flywheel_speed_rpm =
            map_sbus_speed(remote->channel[REMOTE_CH_FLYWHEEL_INDEX],
                           LAUNCH_M3508_TARGET_MAX_SPEED_RPM);
        update.feeder_speed_rpm =
            map_sbus_speed(remote->channel[REMOTE_CH_FEEDER_INDEX],
                           LAUNCH_M2006_ID5_MAX_SPEED_RPM);
        if ((can1_m3508_id2.feedback.online != 0U) &&
            (can1_m3508_id3.feedback.online != 0U))
        {
            update.flags |= LAUNCH_FLYWHEEL_READY;
            (void)osSemaphoreRelease(wake_launchHandle);
        }
        if (can2_m2006_id5.feedback.online != 0U)
        {
            update.flags |= LAUNCH_FEEDER_READY;
            (void)osSemaphoreRelease(wake_launch_motorHandle);
        }
    }
    if (osMessageQueuePut(Update_launch_paraHandle, &update, 0U, 0U) != osOK)
    {
        (void)osMessageQueueGet(Update_launch_paraHandle, &discarded, 0, 0U);
        (void)osMessageQueuePut(Update_launch_paraHandle, &update, 0U, 0U);
    }
}

static uint8_t update_attitude(AttitudeEstimator_t *estimator,
                               TargetAngleMessage_t *message)
{
    float ax;
    float ay;
    float az;
    float roll_acc;
    float pitch_acc;
    float gx;
    float gy;
    float gz;
    float dt;
    uint32_t now;
    uint32_t sample_count;
    uint32_t primask;

    if (BMI088_DMADataReady(&bmi088) == 0U)
        return 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    ax = IMU_ACCEL_X_SIGN * bmi088.acceleration_m_s2[IMU_ACCEL_X_AXIS];
    ay = IMU_ACCEL_Y_SIGN * bmi088.acceleration_m_s2[IMU_ACCEL_Y_AXIS];
    az = IMU_ACCEL_Z_SIGN * bmi088.acceleration_m_s2[IMU_ACCEL_Z_AXIS];
    gx = IMU_GYRO_ROLL_SIGN *
         bmi088.angular_rate_rad_s[IMU_GYRO_ROLL_AXIS];
    gy = IMU_GYRO_PITCH_SIGN *
         bmi088.angular_rate_rad_s[IMU_GYRO_PITCH_AXIS];
    gz = IMU_GYRO_YAW_SIGN *
         bmi088.angular_rate_rad_s[IMU_GYRO_YAW_AXIS];
    now = bmi088.last_update_ms;
    sample_count = bmi088.sample_count;
    if (primask == 0U) __enable_irq();
    if (sample_count == estimator->sample_count) return 0U;
    roll_acc = atan2f(ay, az);
    pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

    if (estimator->initialized == 0U)
    {
        estimator->roll = roll_acc;
        estimator->pitch = pitch_acc;
        estimator->yaw = 0.0f;
        estimator->initialized = 1U;
    }
    else
    {
        dt = (float)(now - estimator->last_sample_ms) * 0.001f;
        if ((dt <= 0.0f) || (dt > ATTITUDE_MAX_DT_S)) dt = CONTROL_PERIOD_S;
        estimator->roll += gx * dt;
        estimator->pitch += gy * dt;
        estimator->yaw += gz * dt;
        estimator->roll = estimator->roll * (1.0f - ATTITUDE_ACCEL_WEIGHT) +
                          roll_acc * ATTITUDE_ACCEL_WEIGHT;
        estimator->pitch = estimator->pitch * (1.0f - ATTITUDE_ACCEL_WEIGHT) +
                           pitch_acc * ATTITUDE_ACCEL_WEIGHT;
    }

    estimator->last_sample_ms = now;
    estimator->sample_count = sample_count;
    message->roll_rad = estimator->roll;
    message->pitch_rad = estimator->pitch;
    message->yaw_rad = estimator->yaw;
    message->timestamp_ms = now;
    message->flags |= GIMBAL_MSG_ATTITUDE;
    return 1U;
}

void Data_Process(void *argument)
{
    AttitudeEstimator_t estimator;
    SBusData_t remote;
    TargetAngleMessage_t message;
    float previous[4] = {0.0f};
    uint32_t previous_remote_update = 0U;
    uint8_t previous_valid = 0U;
    uint8_t publish;
    (void)argument;
    memset(&estimator, 0, sizeof(estimator));

    for (;;)
    {
        process_vofa_commands();
        memset(&message, 0, sizeof(message));
        (void)BMI088_ServiceDMA(&bmi088, HAL_GetTick());
        publish = update_attitude(&estimator, &message);
        SBus_CheckOffline(HAL_GetTick(), SBUS_TIMEOUT_MS);
        CanMotorBus_CheckOffline(HAL_GetTick());

        if ((SBus_GetData(&remote) != 0U) &&
            (remote.last_update_ms != previous_remote_update))
        {
            float current[4];
            uint8_t remote_ok = (uint8_t)((remote.failsafe == 0U) &&
                                          (remote.frame_lost == 0U));
            publish_launch_parameters(&remote, remote_ok);
            current[0] = SBus_ChannelNormalized(REMOTE_CH_PITCH_POS_INDEX);
            current[1] = SBus_ChannelNormalized(REMOTE_CH_PITCH_NEG_INDEX);
            current[2] = SBus_ChannelNormalized(REMOTE_CH_YAW_POS_INDEX);
            current[3] = SBus_ChannelNormalized(REMOTE_CH_YAW_NEG_INDEX);
            if (remote_ok != 0U)
                message.flags |= GIMBAL_MSG_REMOTE_OK;
            else
                message.flags |= GIMBAL_MSG_REMOTE_BAD;

            if ((previous_valid != 0U) && (remote_ok != 0U))
            {
                if (channel_changed(current[0], previous[0]) != 0U)
                {
                    message.pitch_delta_rad += COMMAND_STEP_RAD;
                    message.flags |= GIMBAL_MSG_PITCH_DELTA;
                }
                if (channel_changed(current[1], previous[1]) != 0U)
                {
                    message.pitch_delta_rad -= COMMAND_STEP_RAD;
                    message.flags |= GIMBAL_MSG_PITCH_DELTA;
                }
                if (channel_changed(current[2], previous[2]) != 0U)
                {
                    message.yaw_delta_rad += COMMAND_STEP_RAD;
                    message.flags |= GIMBAL_MSG_YAW_DELTA;
                }
                if (channel_changed(current[3], previous[3]) != 0U)
                {
                    message.yaw_delta_rad -= COMMAND_STEP_RAD;
                    message.flags |= GIMBAL_MSG_YAW_DELTA;
                }
            }
            memcpy(previous, current, sizeof(previous));
            previous_valid = remote_ok;
            previous_remote_update = remote.last_update_ms;
            message.timestamp_ms = HAL_GetTick();
            publish = 1U;
        }

        if (publish != 0U) queue_latest(&message);
        osDelay(DATA_PROCESS_PERIOD_MS);
    }
}
