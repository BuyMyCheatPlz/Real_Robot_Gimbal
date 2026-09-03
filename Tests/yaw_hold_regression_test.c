#include "yaw_hold.h"
#include <assert.h>

int main(void)
{
    const YawHoldConfig_t config = {
        0.002f, 0.004f, 0.50f, 0.001f, 0.01f
    };
    YawHoldState_t state = {0U};

    /* The plant is closely following the reference, but the reference is
     * still far from the final 30-degree target.  Entering hold here creates
     * the observed drive/brake limit cycle. */
    assert(YawHold_Update(&state, &config,
                          0.5235988f, 0.1000f, 0.15f,
                          0.0995f, 1.40f) == 0U);

    /* Hold may be entered only after both trajectory and motor have settled. */
    assert(YawHold_Update(&state, &config,
                          0.5235988f, 0.5235988f, 0.0f,
                          0.5225988f, 0.10f) != 0U);

    /* Hysteresis prevents chatter between the enter and exit thresholds. */
    assert(YawHold_Update(&state, &config,
                          0.5235988f, 0.5235988f, 0.0f,
                          0.5210988f, 0.10f) != 0U);
    assert(YawHold_Update(&state, &config,
                          0.5235988f, 0.5235988f, 0.0f,
                          0.5185988f, 0.10f) == 0U);

    /* A stationary-position coincidence at high speed is not settled. */
    assert(YawHold_Update(&state, &config,
                          0.5235988f, 0.5235988f, 0.0f,
                          0.5235988f, 2.0f) == 0U);

    YawHold_Reset(&state);
    assert(state.active == 0U);
    return 0;
}
