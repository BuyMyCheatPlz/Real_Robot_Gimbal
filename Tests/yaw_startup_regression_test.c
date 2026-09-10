#include "yaw_startup.h"
#include "config.h"
#include <assert.h>

int main(void)
{
    const YawStartupConfig_t config = {
        100U, 10U, 0.50f, 0.0035f
    };
    YawStartupState_t state = {0U};

    /* 单帧看似静止的反馈不足以给 yaw 闭环上电。 */
    assert(YawStartup_Update(&state, &config,
                             1000U, 1000U, 3.4000f, 0.10f) == 0U);
    assert(YawStartup_Update(&state, &config,
                             1050U, 1000U, 3.4000f, 0.10f) == 0U);

    /* 新鲜的高速反馈帧会重新开始零电流稳定窗口。 */
    assert(YawStartup_Update(&state, &config,
                             1052U, 1052U, 3.4100f, 8.00f) == 0U);
    assert(YawStartup_Update(&state, &config,
                             1100U, 1100U, 3.4105f, 0.10f) == 0U);

    /* 位置漂移过大也会重启稳定窗口，即使反馈速度恰好较小。 */
    assert(YawStartup_Update(&state, &config,
                             1160U, 1160U, 3.4145f, 0.10f) == 0U);

    /* 只有由新鲜电机反馈组成的完整稳定区间结束后，才释放闭环。
     * 调用方随后捕获当前位置。 */
    assert(YawStartup_Update(&state, &config,
                             1210U, 1210U, 3.4148f, 0.10f) == 0U);
    assert(YawStartup_Update(&state, &config,
                             1260U, 1260U, 3.4150f, 0.10f) != 0U);
    assert(state.ready != 0U);

    YawStartup_Reset(&state);
    assert(state.ready == 0U);
    assert(state.have_feedback == 0U);

    /* DM4310 反馈流可能慢于 1 ms 控制任务。有效 40 ms 反馈帧之间的
     * 20 ms 控制迭代，不应重启真正稳定的启动窗口。 */
    {
        const YawStartupConfig_t sparse_feedback_config = {
            200U, YAW_STARTUP_MAX_FEEDBACK_AGE_MS, 0.50f, 0.0035f
        };
        YawStartup_Reset(&state);
        assert(YawStartup_Update(&state, &sparse_feedback_config,
                                 2000U, 2000U, 1.0000f, 0.10f) == 0U);
        assert(YawStartup_Update(&state, &sparse_feedback_config,
                                 2020U, 2000U, 1.0000f, 0.10f) == 0U);
        assert(YawStartup_Update(&state, &sparse_feedback_config,
                                 2040U, 2040U, 1.0001f, 0.10f) == 0U);
        assert(YawStartup_Update(&state, &sparse_feedback_config,
                                 2060U, 2040U, 1.0001f, 0.10f) == 0U);
        assert(YawStartup_Update(&state, &sparse_feedback_config,
                                 2080U, 2080U, 1.0002f, 0.10f) == 0U);
        assert(YawStartup_Update(&state, &sparse_feedback_config,
                                 2100U, 2080U, 1.0002f, 0.10f) == 0U);
        assert(YawStartup_Update(&state, &sparse_feedback_config,
                                 2120U, 2120U, 1.0003f, 0.10f) == 0U);
        assert(YawStartup_Update(&state, &sparse_feedback_config,
                                 2140U, 2120U, 1.0003f, 0.10f) == 0U);
        assert(YawStartup_Update(&state, &sparse_feedback_config,
                                 2160U, 2160U, 1.0004f, 0.10f) == 0U);
        assert(YawStartup_Update(&state, &sparse_feedback_config,
                                 2180U, 2160U, 1.0004f, 0.10f) == 0U);
        assert(YawStartup_Update(&state, &sparse_feedback_config,
                                 2200U, 2200U, 1.0005f, 0.10f) != 0U);
    }
    return 0;
}
