#include "yaw_startup.h"
#include "config.h"
#include <assert.h>

int main(void)
{
    const YawStartupConfig_t config = {
        100U, 10U, 0.50f, 0.0035f
    };
    YawStartupState_t state = {0U};

    /* One apparently quiet frame is not enough to energize the yaw loop. */
    assert(YawStartup_Update(&state, &config,
                             1000U, 1000U, 3.4000f, 0.10f) == 0U);
    assert(YawStartup_Update(&state, &config,
                             1050U, 1000U, 3.4000f, 0.10f) == 0U);

    /* A fresh high-speed frame restarts the zero-current settling window. */
    assert(YawStartup_Update(&state, &config,
                             1052U, 1052U, 3.4100f, 8.00f) == 0U);
    assert(YawStartup_Update(&state, &config,
                             1100U, 1100U, 3.4105f, 0.10f) == 0U);

    /* Excess position drift also restarts the window, even when reported
     * speed happens to be small. */
    assert(YawStartup_Update(&state, &config,
                             1160U, 1160U, 3.4145f, 0.10f) == 0U);

    /* Closed loop is released only after a full stable interval made from
     * fresh motor frames.  The caller then captures this current position. */
    assert(YawStartup_Update(&state, &config,
                             1210U, 1210U, 3.4148f, 0.10f) == 0U);
    assert(YawStartup_Update(&state, &config,
                             1260U, 1260U, 3.4150f, 0.10f) != 0U);
    assert(state.ready != 0U);

    YawStartup_Reset(&state);
    assert(state.ready == 0U);
    assert(state.have_feedback == 0U);

    /* The DM4310 feedback stream can be slower than the 1 ms control task.
     * A 20 ms control iteration between valid 40 ms feedback frames must not
     * restart a genuinely stable startup window. */
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
