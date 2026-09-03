#ifndef YAW_HOLD_H
#define YAW_HOLD_H

#include <stdint.h>

typedef struct
{
    float enter_error_rad;
    float exit_error_rad;
    float enter_speed_rpm;
    float profile_position_tolerance_rad;
    float profile_speed_tolerance_rad_s;
} YawHoldConfig_t;

typedef struct
{
    uint8_t active;
} YawHoldState_t;

void YawHold_Reset(YawHoldState_t *state);
uint8_t YawHold_Update(YawHoldState_t *state,
                       const YawHoldConfig_t *config,
                       float final_target_rad,
                       float profile_target_rad,
                       float profile_speed_rad_s,
                       float actual_position_rad,
                       float actual_speed_rpm);

#endif
