#include "gimbal_control.h"
#include "cmsis_os2.h"
#include "can_motor_bus.h"
#include "m3508.h"
#include "m2006.h"
#include "config.h"

extern osMessageQueueId_t Update_launch_paraHandle;
extern osSemaphoreId_t wake_launchHandle;
extern osSemaphoreId_t wake_launch_motorHandle;

/* M2006 角度-速度双环状态。编码器在电机轴(单圈 8192 计数)，拨盘在 P36 输出轴，
 * 累计差分展开后 ÷36 得到输出轴(拨盘)角度(°)。 */
static int32_t m2006_accum_encoder = 0;
static int32_t m2006_last_encoder = 0;
static float m2006_target_output_deg = 0.0f;
static uint16_t m2006_prev_switch = 0U;
static uint8_t m2006_angle_initialized = 0U;

/* 积分编码器差分(处理 8192 回绕)，返回输出轴(拨盘)角度(°)。 */
static float m2006_read_output_deg(void)
{
    int32_t raw = (int32_t)can2_m2006_id5.feedback.encoder;
    int32_t delta = raw - m2006_last_encoder;
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    m2006_accum_encoder += delta;
    m2006_last_encoder = raw;
    return (float)m2006_accum_encoder *
           (360.0f / (M2006_ENCODER_COUNTS_PER_REV * M2006_OUTPUT_GEAR_RATIO));
}

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
    MotorSpeedPid_SetIntegralSeparation(&can1_m3508_id2.speed_pid,
        LAUNCH_M3508_ID2_INTEGRAL_SEPARATION_RPM);
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
    MotorSpeedPid_SetIntegralSeparation(&can1_m3508_id3.speed_pid,
        LAUNCH_M3508_ID3_INTEGRAL_SEPARATION_RPM);
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
    MotorSpeedPid_SetIntegralSeparation(&can2_m2006_id5.speed_pid,
        LAUNCH_M2006_ID5_INTEGRAL_SEPARATION_RPM);
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
                (update.flags & LAUNCH_FLYWHEEL_READY) != 0U);
            feeder_authorized = (uint8_t)(
                (update.flags & LAUNCH_FEEDER_READY) != 0U);
            (void)osSemaphoreAcquire(wake_launchHandle, 0U);
            (void)osSemaphoreAcquire(wake_launch_motorHandle, 0U);
        }

        CanMotorBus_CheckOffline(HAL_GetTick());
        if ((have_update != 0U) && (flywheel_authorized != 0U) &&
            ((HAL_GetTick() - update.timestamp_ms) <= LAUNCH_REMOTE_TIMEOUT_MS))
        {
            if (can1_m3508_id2.feedback.online != 0U)
            {
                M3508_SetSpeed(&can1_m3508_id2,
                    LAUNCH_M3508_ID2_DIRECTION * update.flywheel_speed_rpm);
            }
            else
            {
                M3508_SetSpeed(&can1_m3508_id2, 0.0f);
            }

            if (can1_m3508_id3.feedback.online != 0U)
            {
                M3508_SetSpeed(&can1_m3508_id3,
                    LAUNCH_M3508_ID3_DIRECTION * update.flywheel_speed_rpm);
            }
            else
            {
                M3508_SetSpeed(&can1_m3508_id3, 0.0f);
            }
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
            float output_deg;
            float error_deg;
            float target_rpm;
            uint16_t sw = update.feeder_switch;

            if (m2006_angle_initialized == 0U)
            {
                m2006_last_encoder = (int32_t)can2_m2006_id5.feedback.encoder;
                m2006_accum_encoder = 0;
                m2006_target_output_deg = 0.0f;
                m2006_prev_switch = sw;
                m2006_angle_initialized = 1U;
            }

            output_deg = m2006_read_output_deg();

            /* 模式切换：目标同步到当前位置。S1 语义：
             * 1=保持(角度环)  2=连发(纯速度环 4800rpm=20Hz)
             * 3=单动(角度环)：每次从 1 拨到 3 触发一步 +40°(输出)。 */
            if (sw != m2006_prev_switch)
            {
                uint16_t old_sw = m2006_prev_switch;
                m2006_target_output_deg = output_deg;
                m2006_prev_switch = sw;
                MotorSpeedPid_Reset(&can2_m2006_id5.speed_pid);
                if ((sw == 3U) && (old_sw == 1U))
                    m2006_target_output_deg += LAUNCH_M2006_ID5_STEP_DEG;
            }

            if (sw == 2U)
            {
                /* S1=2 连发：纯速度环。内环积分用于扛住 4800rpm 的持续负载。 */
                MotorSpeedPid_SetGains(&can2_m2006_id5.speed_pid,
                                       LAUNCH_M2006_ID5_SPEED_KP,
                                       LAUNCH_M2006_ID5_SPEED_KI, 0.0f);
                m2006_target_output_deg = output_deg;
                M2006_SetSpeed(&can2_m2006_id5,
                    LAUNCH_M2006_ID5_DIRECTION *
                    LAUNCH_M2006_ID5_CONTINUOUS_SPEED_RPM);
            }
            else
            {
                /* S1=1 保持 / S1=3 单动(触发后走 40°并保持)：角度环。
                 * 内环 KI=0 防过冲；进入到位死区直接给 0 转速 → 内环断电，
                 * 靠摩擦停住，避免目标点附近持续 ~5Hz 振荡。 */
                MotorSpeedPid_SetGains(&can2_m2006_id5.speed_pid,
                                       LAUNCH_M2006_ID5_SPEED_KP,
                                       0.0f, 0.0f);
                error_deg = m2006_target_output_deg - output_deg;
                if ((error_deg < LAUNCH_M2006_ID5_ANGLE_DEADBAND_DEG) &&
                    (error_deg > -LAUNCH_M2006_ID5_ANGLE_DEADBAND_DEG))
                {
                    target_rpm = 0.0f;
                }
                else
                {
                    target_rpm = LAUNCH_M2006_ID5_ANGLE_KP_RPM_PER_DEG *
                                 error_deg;
                    if (target_rpm > LAUNCH_M2006_ID5_ANGLE_MAX_SPEED_RPM)
                        target_rpm = LAUNCH_M2006_ID5_ANGLE_MAX_SPEED_RPM;
                    else if (target_rpm < -LAUNCH_M2006_ID5_ANGLE_MAX_SPEED_RPM)
                        target_rpm = -LAUNCH_M2006_ID5_ANGLE_MAX_SPEED_RPM;
                }
                M2006_SetSpeed(&can2_m2006_id5,
                               LAUNCH_M2006_ID5_DIRECTION * target_rpm);
            }

            gimbal_control_state.m2006_target_deg = m2006_target_output_deg;
            gimbal_control_state.m2006_actual_deg = output_deg;
        }
        else
        {
            M2006_SetSpeed(&can2_m2006_id5, 0.0f);
            MotorSpeedPid_Reset(&can2_m2006_id5.speed_pid);
            m2006_angle_initialized = 0U;
            gimbal_control_state.m2006_target_deg = 0.0f;
            gimbal_control_state.m2006_actual_deg = 0.0f;
        }
    }
}
