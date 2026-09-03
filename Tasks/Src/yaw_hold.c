#include "yaw_hold.h"
#include <math.h>

void YawHold_Reset(YawHoldState_t *state)
{
    if (state != 0) state->active = 0U;
}

uint8_t YawHold_Update(YawHoldState_t *state,
                       const YawHoldConfig_t *config,
                       float final_target_rad,
                       float profile_target_rad,
                       float profile_speed_rad_s,
                       float actual_position_rad,
                       float actual_speed_rpm)
{
    float final_error;
    uint8_t profile_settled;
    if ((state == 0) || (config == 0)) return 0U;

    final_error = fabsf(final_target_rad - actual_position_rad);
    profile_settled = (uint8_t)(
        (fabsf(final_target_rad - profile_target_rad) <=
         config->profile_position_tolerance_rad) &&
        (fabsf(profile_speed_rad_s) <=
         config->profile_speed_tolerance_rad_s));

    if (state->active != 0U)
    {
        if ((profile_settled == 0U) ||
            (final_error > config->exit_error_rad))
            state->active = 0U;
    }
    else if ((profile_settled != 0U) &&
             (final_error <= config->enter_error_rad) &&
             (fabsf(actual_speed_rpm) <= config->enter_speed_rpm))
    {
        state->active = 1U;
    }
    return state->active;
}
