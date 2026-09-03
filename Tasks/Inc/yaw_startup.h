#ifndef YAW_STARTUP_H
#define YAW_STARTUP_H

#include <stdint.h>

typedef struct
{
    uint32_t settle_time_ms;
    uint32_t max_feedback_age_ms;
    float max_speed_rpm;
    float max_position_drift_rad;
} YawStartupConfig_t;

typedef struct
{
    uint32_t stable_since_ms;
    uint32_t last_feedback_ms;
    float anchor_position_rad;
    uint8_t have_feedback;
    uint8_t settling;
    uint8_t ready;
} YawStartupState_t;

void YawStartup_Reset(YawStartupState_t *state);
uint8_t YawStartup_Update(YawStartupState_t *state,
                          const YawStartupConfig_t *config,
                          uint32_t now_ms,
                          uint32_t feedback_timestamp_ms,
                          float position_rad,
                          float speed_rpm);

#endif
