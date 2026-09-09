#include <assert.h>
#include <math.h>

#include "../Tasks/Inc/attitude_math.h"

int main(void)
{
    float roll;
    float pitch;

    /* 绕 X 轴平滑转过 ay=0：旧 atan2(-ax, ay) 在 ax 的微小噪声下会跳变，
     * 完整重力投影应保持连续。 */
    AttitudeMath_AccelToRollPitch(0.001f, 0.01f, 9.81f, &roll, &pitch);
    assert(fabsf(roll) < 0.01f);
    assert(fabsf(pitch) < 0.01f);
    AttitudeMath_AccelToRollPitch(-0.001f, -0.01f, 9.81f, &roll, &pitch);
    assert(fabsf(roll) < 0.01f);
    assert(fabsf(pitch) < 0.01f);

    /* +pi/-pi 相邻时，互补滤波校正必须是小角度，而非约 2*pi。 */
    assert(fabsf(AttitudeMath_AngleDifference(-3.13f, 3.13f)) < 0.03f);
    return 0;
}
