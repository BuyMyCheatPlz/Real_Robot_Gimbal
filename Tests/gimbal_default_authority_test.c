#include <assert.h>
#include <math.h>
#include <stdlib.h>

#include "../Tasks/Inc/config.h"
#include "../Hardware_Drivers/Can/Inc/motor_common.h"
#include "../Hardware_Drivers/Gm6020/Inc/gm6020.h"
#include "../Hardware_Drivers/Dm4310/Inc/dm4310.h"

int main(void)
{
    GM6020_t pitch;
    DM4310_t yaw;
    float pitch_error_rad = 10.0f * TASK_DEG_TO_RAD;
    float pitch_speed_target = PITCH_ANGLE_KP_RPM_PER_RAD *
                               pitch_error_rad;
    int16_t pitch_command;
    int16_t yaw_command;

    GM6020_Init(&pitch, 2U, PITCH_SPEED_KP, PITCH_SPEED_KI);
    MotorSpeedPid_SetGains(&pitch.speed_pid, PITCH_SPEED_KP,
                           PITCH_SPEED_KI, PITCH_SPEED_KD);
    pitch.feedback.online = 1U;
    pitch.feedback.speed_rpm = 0;
    GM6020_SetSpeed(&pitch, pitch_speed_target);
    pitch_command = GM6020_Update(&pitch, CONTROL_PERIOD_S);

    /* A normal 10 degree startup error must produce useful authority.  The
     * commissioning values generated only about 35/30000 command counts and
     * could neither lift nor hold the Pitch assembly. */
    assert(abs(pitch_command) >= 1000);

    DM4310_Init(&yaw, 1U, YAW_SPEED_KP_CURRENT_PER_RPM,
                YAW_SPEED_KI_CURRENT_PER_RPM_S);
    MotorSpeedPid_SetGains(&yaw.speed_pid,
                           YAW_SPEED_KP_CURRENT_PER_RPM,
                           YAW_SPEED_KI_CURRENT_PER_RPM_S,
                           YAW_SPEED_KD_CURRENT_S_PER_RPM);
    yaw.speed_pid.output_limit = YAW_CURRENT_OUTPUT_LIMIT;
    DM4310_SetSpeed(&yaw, 100.0f);
    yaw_command = DM4310_Update(&yaw, CONTROL_PERIOD_S);

    /* Real captured data shows that command 20 excites a violent oscillation;
     * the complete command path, including direction-test mode, is capped at
     * the empirically safe commissioning value. */
    assert(abs(yaw_command) <= 3);
    assert(YAW_DIRECTION_TEST_MAX_CURRENT <= 3);
    assert(DM4310_CURRENT_COMMAND_LIMIT <= 3.0f);
    assert((YAW_VELOCITY_FF_GAIN > 0.0f) &&
           (YAW_VELOCITY_FF_GAIN <= 1.0f));
    assert(YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2 > 0.0f);
    assert(YAW_ACCELERATION_FF_CURRENT_LIMIT <=
           YAW_CURRENT_OUTPUT_LIMIT);
    assert(YAW_SPEED_INTEGRAL_LIMIT_CURRENT <=
           YAW_CURRENT_OUTPUT_LIMIT);

    return 0;
}
