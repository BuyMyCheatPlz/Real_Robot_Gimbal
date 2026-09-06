#include "gimbal_control.h"
#include "cmsis_os2.h"
#include "hardware_drivers.h"
#include "can_motor_bus.h"
#include "dbus.h"
#include "vofa.h"
#include "config.h"
#include "pid_parameter.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

extern osMessageQueueId_t Target_AngleHandle;
extern osMessageQueueId_t Update_PID_paraHandle;
extern osMessageQueueId_t Update_launch_paraHandle;
extern osSemaphoreId_t wake_launchHandle;
extern osSemaphoreId_t wake_launch_motorHandle;

#define COMMAND_STEP_RAD             (GIMBAL_COMMAND_STEP_DEG * TASK_DEG_TO_RAD)
#define PI_F                         3.14159265358979323846f
#define TWO_PI_F                     (2.0f * PI_F)

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    uint32_t last_sample_ms;
    uint32_t sample_count;
    uint8_t initialized;
} AttitudeEstimator_t;

/* IMU Yaw 是相对航向角。为显示和回零计算保持其有界；电机编码器位置在 PID_calc
 * 中仍保持展开状态。 */
static float normalize_yaw_rad(float angle)
{
    while (angle >= PI_F) angle -= TWO_PI_F;
    while (angle < -PI_F) angle += TWO_PI_F;
    return angle;
}

static int8_t dbus_channel_sign(uint16_t channel)
{
    int32_t value = (int32_t)channel;
    int32_t center = (int32_t)DBUS_CENTER_CHANNEL;
    int32_t deadzone = (int32_t)DBUS_DEADZONE;

    if ((value >= (center - deadzone)) && (value <= (center + deadzone)))
        return 0;
    if (value > (center + deadzone)) return 1;
    return -1;
}

static float map_dbus_switch_speed(uint16_t switch_value,
                                   float high_speed_rpm,
                                   float low_speed_rpm)
{
    if (switch_value == 3U) return high_speed_rpm;
    if (switch_value == 2U) return low_speed_rpm;
    return 0.0f;
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

static uint8_t parse_yaw_test_command(const char *command, int16_t *current)
{
    const char prefix[] = "YAWTEST=";
    char *end;
    long value;
    if ((command == 0) || (current == 0) ||
        (strncmp(command, prefix, sizeof(prefix) - 1U) != 0))
        return 0U;
    value = strtol(command + sizeof(prefix) - 1U, &end, 10);
    if ((end == command + sizeof(prefix) - 1U) || (*end != '\0') ||
        (value < -YAW_DIRECTION_TEST_MAX_CURRENT) ||
        (value > YAW_DIRECTION_TEST_MAX_CURRENT))
        return 0U;
    *current = (int16_t)value;
    return 1U;
}

static void process_vofa_commands(void)
{
    char command[VOFA_COMMAND_MAX_LENGTH];
    PidParameterUpdate_t update;
    PidParameterUpdate_t discarded;
    int16_t yaw_test_current;
    while (VOFA_GetCommand(command) != 0U)
    {
#if (YAW_SYSID_MODE != 0U)
        if (strcmp(command, "identify_on") == 0)
        {
            /* 开始一次 yaw 系统辨识(正弦扫频直通电流)，可重复触发 */
            Gimbal_YawSysid_Start();
            continue;
        }
#endif
        if (parse_yaw_test_command(command, &yaw_test_current) != 0U)
        {
            Gimbal_YawTest_Request(yaw_test_current);
            continue;
        }
        if (PidParameter_Parse(command, &update) == 0U)
        {
            /* 解析失败：计数不加，I6 通道不变 */
            continue;
        }
        if (osMessageQueuePut(Update_PID_paraHandle, &update, 0U, 0U) != osOK)
        {
            (void)osMessageQueueGet(Update_PID_paraHandle, &discarded, 0, 0U);
            (void)osMessageQueuePut(Update_PID_paraHandle, &update, 0U, 0U);
        }
        /* 回显计数：成功解析一次 +1，VOFA I6 通道可观测 */
        ++gimbal_control_state.param_parse_count;
    }
}

static void publish_launch_parameters(const DBusData_t *remote,
                                      uint8_t remote_ok)
{
    LaunchParameterUpdate_t update;
    LaunchParameterUpdate_t discarded;
    memset(&update, 0, sizeof(update));
    update.timestamp_ms = HAL_GetTick();
    if (remote_ok != 0U)
    {
        update.flywheel_speed_rpm =
            map_dbus_switch_speed(remote->channel[REMOTE_CH_S2_INDEX],
                                  LAUNCH_M3508_TARGET_MAX_SPEED_RPM,
                                  100.0f);
        update.feeder_speed_rpm =
            map_dbus_switch_speed(remote->channel[REMOTE_CH_S1_INDEX],
                                  LAUNCH_M2006_ID5_MAX_SPEED_RPM,
                                  50.0f);
        update.feeder_switch = remote->channel[REMOTE_CH_S1_INDEX];
        if ((can1_m3508_id2.feedback.online != 0U) ||
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
        estimator->yaw = normalize_yaw_rad(estimator->yaw + gz * dt);
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
    DBusData_t remote;
    TargetAngleMessage_t message;
    int8_t previous_pitch_sign = 0;
    int8_t previous_yaw_sign = 0;
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
        DBus_CheckOffline(HAL_GetTick(), DBUS_TIMEOUT_MS);
        CanMotorBus_CheckOffline(HAL_GetTick());

        if ((DBus_GetData(&remote) != 0U) &&
            (remote.last_update_ms != previous_remote_update))
        {
            int8_t current_pitch_sign;
            int8_t current_yaw_sign;
            uint8_t remote_ok = (uint8_t)((remote.failsafe == 0U) &&
                                          (remote.frame_lost == 0U));
            publish_launch_parameters(&remote, remote_ok);
            current_pitch_sign = dbus_channel_sign(remote.channel[REMOTE_CH_PITCH_INDEX]);
            current_yaw_sign = dbus_channel_sign(remote.channel[REMOTE_CH_YAW_INDEX]);
            if (remote_ok != 0U)
                message.flags |= GIMBAL_MSG_REMOTE_OK;
            else
                message.flags |= GIMBAL_MSG_REMOTE_BAD;

            if ((previous_valid != 0U) && (remote_ok != 0U))
            {
                if (current_pitch_sign != previous_pitch_sign)
                {
                    if (current_pitch_sign > 0)
                        message.pitch_delta_rad += COMMAND_STEP_RAD;
                    else if (current_pitch_sign < 0)
                        message.pitch_delta_rad -= COMMAND_STEP_RAD;
                    if (current_pitch_sign != 0)
                        message.flags |= GIMBAL_MSG_PITCH_DELTA;
                }
                if (current_yaw_sign != previous_yaw_sign)
                {
                    if (current_yaw_sign < 0)
                        message.yaw_delta_rad += COMMAND_STEP_RAD;
                    else if (current_yaw_sign > 0)
                        message.yaw_delta_rad -= COMMAND_STEP_RAD;
                    if (current_yaw_sign != 0)
                        message.flags |= GIMBAL_MSG_YAW_DELTA;
                }
            }
            previous_pitch_sign = current_pitch_sign;
            previous_yaw_sign = current_yaw_sign;
            previous_valid = remote_ok;
            previous_remote_update = remote.last_update_ms;
            message.timestamp_ms = HAL_GetTick();
            publish = 1U;
        }

        if (publish != 0U) queue_latest(&message);
        osDelay(DATA_PROCESS_PERIOD_MS);
    }
}
