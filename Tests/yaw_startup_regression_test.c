#include "yaw_startup.h"
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
    return 0;
}
