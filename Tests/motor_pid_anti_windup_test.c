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

    /* 正向前馈可能耗尽执行器正向余量。反馈 PID 必须使用剩余的非对称限幅；
     * 否则最终电机命令在叠加前馈后已经饱和，积分器仍会继续饱和累积。 */
    MotorSpeedPid_Init(&pid, 0.0f, 10.0f, 100.0f, 50.0f);
    output = MotorSpeedPid_CalculateFloatBounded(
        &pid, 1.0f, 0.0f, 1.0f, -50.0f, 0.0f);
    assert(output == 0.0f);
    assert(pid.integral == 0.0f);

    /* 反向误差仍必须允许产生制动力矩。 */
    output = MotorSpeedPid_CalculateFloatBounded(
        &pid, -1.0f, 0.0f, 1.0f, -50.0f, 0.0f);
    assert(output == -10.0f);
    assert(pid.integral == -10.0f);
    return 0;
}
