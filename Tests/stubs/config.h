#ifndef TEST_TASK_CONFIG_H
#define TEST_TASK_CONFIG_H

/* Exercise the normal dual-CAN path in the host safety test.  The target's
 * config.h keeps commissioning isolation enabled independently. */
#define YAW_COMMISSIONING_MODE 0U
#define LAUNCH_MOTOR_OUTPUT_ENABLE 0U
#define ONLINE_PID_VALUE_MAX 100000.0f
#define CAN_COMMAND_PERIOD_MS 10U
#define M3508_CURRENT_LIMIT 16384.0f
#define M2006_CURRENT_LIMIT 10000.0f
#define DM4310_CURRENT_COMMAND_LIMIT 16384.0f
#define PITCH_GM6020_CAN_ID 2U
#define PITCH_STARTUP_SPEED_THRESHOLD_RPM 1.0f
#define PITCH_STARTUP_MIN_VOLTAGE 8000.0f
#define YAW_STARTUP_SPEED_THRESHOLD_RPM 0.50f
#define YAW_STARTUP_MIN_CURRENT 500.0f

#endif
