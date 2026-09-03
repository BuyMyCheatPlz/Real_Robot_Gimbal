#ifndef TEST_TASK_CONFIG_H
#define TEST_TASK_CONFIG_H

/* Exercise the normal dual-CAN path in the host safety test.  The target's
 * config.h keeps commissioning isolation enabled independently. */
#define YAW_COMMISSIONING_MODE 0U
#define LAUNCH_MOTOR_OUTPUT_ENABLE 0U
#define ONLINE_PID_VALUE_MAX 100000.0f

#endif
