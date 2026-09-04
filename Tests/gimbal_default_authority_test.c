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

    /* A 10 degree Pitch error must overcome the loaded gimbal's static
     * friction; gravity itself is handled by the separate feedforward below. */
    assert(abs(pitch_command) >= 800);
    assert(abs(pitch_command) <= 1000);
    assert(PITCH_GRAVITY_FF_MAX_VOLTAGE >= 1000.0f);

    DM4310_Init(&yaw, 1U, YAW_SPEED_KP_CURRENT_PER_RPM,
                YAW_SPEED_KI_CURRENT_PER_RPM_S);
    MotorSpeedPid_SetGains(&yaw.speed_pid,
                           YAW_SPEED_KP_CURRENT_PER_RPM,
                           YAW_SPEED_KI_CURRENT_PER_RPM_S,
                           YAW_SPEED_KD_CURRENT_S_PER_RPM);
    yaw.speed_pid.output_limit = YAW_CURRENT_OUTPUT_LIMIT;
    DM4310_SetSpeed(&yaw, 100.0f);
    yaw_command = DM4310_Update(&yaw, CONTROL_PERIOD_S);

    /* After correcting the DM4310 command slots to little-endian, 20 is sent
     * as 14 00 and no longer becomes the byte-swapped value 5120. */
    assert(abs(yaw_command) <= 1000);
    /* A large yaw error must request enough speed/current to overcome the
     * measured static friction; the former 0.08 rad/s trajectory and 1.0
     * current/rpm path produced only about 5 current counts. */
    assert(YAW_MAX_SPEED_RAD_S >= 0.5f);
    assert(YAW_TRAJECTORY_MAX_SPEED_RAD_S >= 0.4f);
    assert(YAW_SPEED_KP_CURRENT_PER_RPM >= 20.0f);
       assert(YAW_DIRECTION_TEST_MAX_CURRENT <= 1000);
       assert(DM4310_CURRENT_COMMAND_LIMIT <= 1000.0f);
    assert((YAW_VELOCITY_FF_GAIN > 0.0f) &&
           (YAW_VELOCITY_FF_GAIN <= 1.0f));
    assert(YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2 > 0.0f);
    assert(YAW_ACCELERATION_FF_CURRENT_LIMIT <=
           YAW_CURRENT_OUTPUT_LIMIT);
    assert(YAW_SPEED_INTEGRAL_LIMIT_CURRENT <=
           YAW_CURRENT_OUTPUT_LIMIT);

    return 0;
}
