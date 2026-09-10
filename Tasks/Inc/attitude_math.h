#ifndef ATTITUDE_MATH_H
#define ATTITUDE_MATH_H

#include <math.h>

#define ATTITUDE_MATH_PI_F      3.14159265358979323846f
#define ATTITUDE_MATH_TWO_PI_F  (2.0f * ATTITUDE_MATH_PI_F)

/* 重力向量投影到标准 Roll/Pitch 欧拉角。使用完整的 Y/Z 模长，避免 ay 接近
 * 0 时把加速度计噪声放大为姿态角跳变。 */
static inline void AttitudeMath_AccelToRollPitch(float ax, float ay, float az,
                                                 float *roll_rad,
                                                 float *pitch_rad)
{
    *roll_rad = atan2f(ay, az);
    *pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));
}

/* 返回目标角相对当前角的最短有符号角差，范围 [-pi, pi)。 */
static inline float AttitudeMath_AngleDifference(float target, float current)
{
    float difference = target - current;
    while (difference >= ATTITUDE_MATH_PI_F) difference -= ATTITUDE_MATH_TWO_PI_F;
    while (difference < -ATTITUDE_MATH_PI_F) difference += ATTITUDE_MATH_TWO_PI_F;
    return difference;
}

#endif
