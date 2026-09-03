#include "yaw_startup.h"
#include <math.h>
#include <string.h>

void YawStartup_Reset(YawStartupState_t *state)
{
    if (state != 0) memset(state, 0, sizeof(*state));
}

uint8_t YawStartup_Update(YawStartupState_t *state,
                          const YawStartupConfig_t *config,
                          uint32_t now_ms,
                          uint32_t feedback_timestamp_ms,
                          float position_rad,
                          float speed_rpm)
{
    if ((state == 0) || (config == 0)) return 0U;
    if (state->ready != 0U) return 1U;

    /* 仅等待时间绝不能授权电机。只有收到新的新鲜 CAN 反馈帧时，才推进稳定窗口。 */
    if ((now_ms - feedback_timestamp_ms) > config->max_feedback_age_ms)
    {
        state->have_feedback = 0U;
        state->settling = 0U;
        return 0U;
    }
    if ((state->have_feedback != 0U) &&
        (feedback_timestamp_ms == state->last_feedback_ms))
        return 0U;

    state->have_feedback = 1U;
    state->last_feedback_ms = feedback_timestamp_ms;
    if (fabsf(speed_rpm) > config->max_speed_rpm)
    {
        state->settling = 0U;
        return 0U;
    }

    if (state->settling == 0U)
    {
        state->anchor_position_rad = position_rad;
        state->stable_since_ms = feedback_timestamp_ms;
        state->settling = 1U;
    }
    else if (fabsf(position_rad - state->anchor_position_rad) >
             config->max_position_drift_rad)
    {
        state->anchor_position_rad = position_rad;
        state->stable_since_ms = feedback_timestamp_ms;
    }
    else if ((feedback_timestamp_ms - state->stable_since_ms) >=
             config->settle_time_ms)
    {
        state->ready = 1U;
    }
    return state->ready;
}
