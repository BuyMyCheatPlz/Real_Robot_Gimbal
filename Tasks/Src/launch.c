#include "gimbal_control.h"
#include "cmsis_os2.h"
#include "can_motor_bus.h"
#include "m3508.h"
#include "m2006.h"
#include "config.h"

extern osMessageQueueId_t Update_launch_paraHandle;
extern osSemaphoreId_t wake_launchHandle;
extern osSemaphoreId_t wake_launch_motorHandle;

void Launch_Task(void *argument)
{
    LaunchParameterUpdate_t update;
    uint8_t have_update = 0U;
    uint8_t flywheel_authorized = 0U;
    uint8_t feeder_authorized = 0U;
    (void)argument;
    MotorSpeedPid_Init(&can1_m3508_id2.speed_pid,
                       LAUNCH_M3508_ID2_SPEED_KP,
                       LAUNCH_M3508_ID2_SPEED_KI,
                       LAUNCH_M3508_ID2_INTEGRAL_LIMIT,
                       LAUNCH_M3508_ID2_OUTPUT_LIMIT);
    MotorSpeedPid_SetGains(&can1_m3508_id2.speed_pid,
                           LAUNCH_M3508_ID2_SPEED_KP,
                           LAUNCH_M3508_ID2_SPEED_KI,
                           LAUNCH_M3508_ID2_SPEED_KD);
    M3508_SetSpeedFilterAlpha(&can1_m3508_id2,
                              LAUNCH_M3508_ID2_SPEED_LPF_ALPHA);
    MotorSpeedPid_Init(&can1_m3508_id3.speed_pid,
                       LAUNCH_M3508_ID3_SPEED_KP,
                       LAUNCH_M3508_ID3_SPEED_KI,
                       LAUNCH_M3508_ID3_INTEGRAL_LIMIT,
                       LAUNCH_M3508_ID3_OUTPUT_LIMIT);
    MotorSpeedPid_SetGains(&can1_m3508_id3.speed_pid,
                           LAUNCH_M3508_ID3_SPEED_KP,
                           LAUNCH_M3508_ID3_SPEED_KI,
                           LAUNCH_M3508_ID3_SPEED_KD);
    M3508_SetSpeedFilterAlpha(&can1_m3508_id3,
                              LAUNCH_M3508_ID3_SPEED_LPF_ALPHA);
    MotorSpeedPid_Init(&can2_m2006_id5.speed_pid,
                       LAUNCH_M2006_ID5_SPEED_KP,
                       LAUNCH_M2006_ID5_SPEED_KI,
                       LAUNCH_M2006_ID5_INTEGRAL_LIMIT,
                       LAUNCH_M2006_ID5_OUTPUT_LIMIT);
    MotorSpeedPid_SetGains(&can2_m2006_id5.speed_pid,
                           LAUNCH_M2006_ID5_SPEED_KP,
                           LAUNCH_M2006_ID5_SPEED_KI,
                           LAUNCH_M2006_ID5_SPEED_KD);
    M2006_SetSpeedFilterAlpha(&can2_m2006_id5,
                              LAUNCH_M2006_ID5_SPEED_LPF_ALPHA);

    for (;;)
    {
        if (osMessageQueueGet(Update_launch_paraHandle, &update, 0,
                              LAUNCH_TASK_WAIT_MS) == osOK)
        {
            while (osMessageQueueGet(Update_launch_paraHandle, &update,
                                     0, 0U) == osOK) {}
            have_update = 1U;
            flywheel_authorized = (uint8_t)(
                ((update.flags & LAUNCH_FLYWHEEL_READY) != 0U) &&
                (osSemaphoreAcquire(wake_launchHandle, 0U) == osOK));
            feeder_authorized = (uint8_t)(
                ((update.flags & LAUNCH_FEEDER_READY) != 0U) &&
                (osSemaphoreAcquire(wake_launch_motorHandle, 0U) == osOK));
            if (flywheel_authorized == 0U)
                (void)osSemaphoreAcquire(wake_launchHandle, 0U);
            if (feeder_authorized == 0U)
                (void)osSemaphoreAcquire(wake_launch_motorHandle, 0U);
        }

        CanMotorBus_CheckOffline(HAL_GetTick());
        if ((have_update != 0U) && (flywheel_authorized != 0U) &&
            ((HAL_GetTick() - update.timestamp_ms) <= LAUNCH_REMOTE_TIMEOUT_MS) &&
            (can1_m3508_id2.feedback.online != 0U) &&
            (can1_m3508_id3.feedback.online != 0U))
        {
            M3508_SetSpeed(&can1_m3508_id2,
                LAUNCH_M3508_ID2_DIRECTION * update.flywheel_speed_rpm);
            M3508_SetSpeed(&can1_m3508_id3,
                LAUNCH_M3508_ID3_DIRECTION * update.flywheel_speed_rpm);
        }
        else
        {
            M3508_SetSpeed(&can1_m3508_id2, 0.0f);
            M3508_SetSpeed(&can1_m3508_id3, 0.0f);
        }

        if ((have_update != 0U) && (feeder_authorized != 0U) &&
            ((HAL_GetTick() - update.timestamp_ms) <= LAUNCH_REMOTE_TIMEOUT_MS) &&
            (can2_m2006_id5.feedback.online != 0U))
        {
            M2006_SetSpeed(&can2_m2006_id5,
                LAUNCH_M2006_ID5_DIRECTION * update.feeder_speed_rpm);
        }
        else
        {
            M2006_SetSpeed(&can2_m2006_id5, 0.0f);
        }
    }
}
