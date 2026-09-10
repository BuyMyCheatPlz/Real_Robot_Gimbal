#include "yaw_hold.h"
#include <assert.h>

int main(void)
{
    const YawHoldConfig_t config = {
        0.002f, 0.004f, 0.50f, 0.001f, 0.01f
    };
    YawHoldState_t state = {0U};

    /* 被控对象虽然紧跟轨迹参考，但轨迹参考仍远离最终 30° 目标。
     * 此时进入保持会产生实测到的驱动/刹车极限环。 */
    assert(YawHold_Update(&state, &config,
                          0.5235988f, 0.1000f, 0.15f,
                          0.0995f, 1.40f) == 0U);

    /* 只有轨迹和电机都稳定后才允许进入保持。 */
    assert(YawHold_Update(&state, &config,
                          0.5235988f, 0.5235988f, 0.0f,
                          0.5225988f, 0.10f) != 0U);

    /* 滞回用于防止在进入/退出阈值之间来回抖动。 */
    assert(YawHold_Update(&state, &config,
                          0.5235988f, 0.5235988f, 0.0f,
                          0.5210988f, 0.10f) != 0U);
    assert(YawHold_Update(&state, &config,
                          0.5235988f, 0.5235988f, 0.0f,
                          0.5185988f, 0.10f) == 0U);

    /* 高速运动时即使位置瞬间重合，也不能视为稳定。 */
    assert(YawHold_Update(&state, &config,
                          0.5235988f, 0.5235988f, 0.0f,
                          0.5235988f, 2.0f) == 0U);

    YawHold_Reset(&state);
    assert(state.active == 0U);
    return 0;
}
