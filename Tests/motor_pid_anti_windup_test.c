#include "motor_common.h"
#include <assert.h>

int main(void)
{
    MotorSpeedPid_t pid;
    float output;

    MotorSpeedPid_Init(&pid, 0.0f, 5.0f, 10.0f, 50.0f);
    MotorSpeedPid_SetIntegralSeparation(&pid, 2.0f);
    assert(MotorSpeedPid_Calculate(&pid, 3.0f, 0.0f, 1.0f) == 0);
    assert(pid.integral == 0.0f);
    assert(MotorSpeedPid_Calculate(&pid, 1.0f, 0.0f, 1.0f) == 5);
    assert(MotorSpeedPid_Calculate(&pid, 1.0f, 0.0f, 1.0f) == 10);
    assert(MotorSpeedPid_Calculate(&pid, 1.0f, 0.0f, 1.0f) == 10);
    assert(pid.integral == 10.0f);

    MotorSpeedPid_Init(&pid, 55.0f, 10.0f, 100.0f, 50.0f);
    MotorSpeedPid_SetIntegralSeparation(&pid, 10.0f);
    assert(MotorSpeedPid_Calculate(&pid, 1.0f, 0.0f, 1.0f) == 50);
    assert(pid.integral == 0.0f);
    pid.integral = 20.0f;
    assert(MotorSpeedPid_Calculate(&pid, -1.0f, 0.0f, 1.0f) == -45);
    assert(pid.integral == 10.0f);

    /* A positive feedforward can consume all positive actuator headroom.
     * The feedback PID must use the remaining asymmetric limits; otherwise
     * its integrator winds up even though the final motor command is already
     * saturated after feedforward is added. */
    MotorSpeedPid_Init(&pid, 0.0f, 10.0f, 100.0f, 50.0f);
    output = MotorSpeedPid_CalculateFloatBounded(
        &pid, 1.0f, 0.0f, 1.0f, -50.0f, 0.0f);
    assert(output == 0.0f);
    assert(pid.integral == 0.0f);

    /* The opposite error must still be allowed to generate braking torque. */
    output = MotorSpeedPid_CalculateFloatBounded(
        &pid, -1.0f, 0.0f, 1.0f, -50.0f, 0.0f);
    assert(output == -10.0f);
    assert(pid.integral == -10.0f);
    return 0;
}
