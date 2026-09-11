#include <assert.h>
#include <math.h>
#include <stdlib.h>

#include "../Tasks/Inc/config.h"
#include "../Hardware_Drivers/Can/Inc/motor_common.h"
#include "../Hardware_Drivers/Gm6020/Inc/gm6020.h"
#include "../Hardware_Drivers/Dm4310/Inc/dm4310.h"

int main(void)
{
    GM6020_t pitch;
    GM6020_t pitch_with_ff;
    DM4310_t yaw;
    float pitch_error_deg = 10.0f;
    float pitch_speed_target = PITCH_ANGLE_KP_RPM_PER_RAD *
                               pitch_error_deg;
    int expected_pitch_command;
    int16_t pitch_command;
    int16_t yaw_command;

    GM6020_Init(&pitch, 2U, PITCH_SPEED_KP, PITCH_SPEED_KI);
    MotorSpeedPid_SetGains(&pitch.speed_pid, PITCH_SPEED_KP,
                           PITCH_SPEED_KI, PITCH_SPEED_KD);
    MotorSpeedPid_SetIntegralSeparation(&pitch.speed_pid,
                                        PITCH_SPEED_INTEGRAL_SEPARATION_RPM);
    pitch.feedback.online = 1U;
    pitch.feedback.speed_rpm = 0;
    GM6020_SetSpeed(&pitch, pitch_speed_target);
    pitch_command = GM6020_Update(&pitch, CONTROL_PERIOD_S);

    /* 该单元测试把位置环结果直接送入速度环，因此有意绕过 PITCH_MAX_SPEED_RPM。
     * 首个采样点 PI 积分为 0，命令必须等于 Kp * 目标值，只受 GM6020 输出限幅约束。 */
    expected_pitch_command = (int)(PITCH_SPEED_KP * pitch_speed_target);
    if (expected_pitch_command > (int)GM6020_VOLTAGE_LIMIT)
        expected_pitch_command = (int)GM6020_VOLTAGE_LIMIT;
    if ((fabsf(pitch_speed_target) >= PITCH_STARTUP_SPEED_THRESHOLD_RPM) &&
        (abs(expected_pitch_command) < (int)PITCH_STARTUP_MIN_VOLTAGE))
        expected_pitch_command = (pitch_speed_target > 0.0f) ?
            (int)PITCH_STARTUP_MIN_VOLTAGE :
            -(int)PITCH_STARTUP_MIN_VOLTAGE;
    /* 主机编译器与目标 FPU 的浮点评估顺序略有差异，转成 int16 时可能相差 1 个 LSB。 */
    assert(abs((int)pitch_command - expected_pitch_command) <= 1);
    assert(PITCH_GRAVITY_ONLY_ENABLE == 0U);
    assert((PITCH_GRAVITY_ROLL_LPF_ALPHA > 0.0f) &&
           (PITCH_GRAVITY_ROLL_LPF_ALPHA <= 1.0f));
    assert(PITCH_SPEED_LPF_ALPHA == 1.0f);
    assert((PITCH_ROLL_RATE_TO_SPEED_SIGN == 1.0f) ||
           (PITCH_ROLL_RATE_TO_SPEED_SIGN == -1.0f));
    assert(PITCH_ROLL_RATE_TO_SPEED_SIGN ==
           (PITCH_MOTOR_SIGN * PITCH_ENCODER_TO_IMU_SIGN));
    assert(PITCH_GRAVITY_FIT_MIN_DEG == -50.0f);
    assert(PITCH_GRAVITY_FIT_MAX_DEG == 26.0f);
    assert(PITCH_GRAVITY_POLY_C0 == 1678.105241f);
    assert(PITCH_GRAVITY_C1_GR == 161.466898f);
    assert(PITCH_GRAVITY_POLY_C2 == -0.590091f);
    assert(PITCH_GRAVITY_POLY_C3 == 0.006451f);
    assert(PITCH_GRAVITY_POLY_C4 == 0.0f);
    assert(IMU_GYRO_ROLL_AXIS == 0U);
    assert(IMU_GYRO_ROLL_SIGN == 1.0f);
    assert(IMU_GYRO_PITCH_AXIS == 2U);
    assert(IMU_GYRO_YAW_AXIS == 1U);
    assert(PITCH_ANGLE_KP_RPM_PER_RAD == 43.6f);
    assert(PITCH_ANGLE_KI_RPM_PER_RAD_S == 39.64f);
    assert(PITCH_ANGLE_KD_RPM_S_PER_RAD == 0.0f);
    assert(PITCH_POSITION_D_NOTCH_ENABLE == 1U);
    assert(PITCH_POSITION_D_NOTCH_CENTER_HZ == 32.0f);
    assert(PITCH_POSITION_D_NOTCH_Q == 1.5f);
    assert(PITCH_POSITION_D_NOTCH2_ENABLE == 1U);
    assert(PITCH_POSITION_D_NOTCH2_CENTER_HZ == 23.7f);
    assert(PITCH_POSITION_D_NOTCH2_Q == 1.5f);
    assert(PITCH_POSITION_D_LOW_ANGLE_SCALE_ENABLE == 1U);
    assert(PITCH_POSITION_D_LOW_ANGLE_START_DEG == -45.0f);
    assert(PITCH_POSITION_D_LOW_ANGLE_FULL_DEG == -55.0f);
    assert(PITCH_POSITION_D_LOW_ANGLE_SCALE == 0.45f);
    assert(PITCH_POSITION_D_STEP_FADE_ENABLE == 1U);
    assert(PITCH_POSITION_D_STEP_DISABLE_ERROR_DEG == 5.0f);
    assert(PITCH_POSITION_D_STEP_RESTORE_ERROR_DEG == 2.0f);
    assert(PITCH_POSITION_OUTPUT_DIRECTION_GUARD_ENABLE == 1U);
    assert(PITCH_POSITION_OUTPUT_DIRECTION_GUARD_ERROR_DEG == 0.6f);
    assert(PITCH_SETTLE_HOLD_ENABLE == 0U);
    assert(PITCH_SETTLE_HOLD_ENTER_ERROR_DEG == 0.18f);
    assert(PITCH_SETTLE_HOLD_EXIT_ERROR_DEG == 0.35f);
    assert(PITCH_SETTLE_HOLD_ENTER_SPEED_DEG_S == 18.0f);
    assert(PITCH_NEAR_TARGET_SPEED_CLAMP_ENABLE == 1U);
    assert(PITCH_NEAR_TARGET_SPEED_CLAMP_ERROR_DEG == 0.6f);
    assert(PITCH_NEAR_TARGET_SPEED_LIMIT_DEG_S == 12.0f);
    assert(PITCH_STATIC_ERROR_COMP_ENABLE == 1U);
    assert(PITCH_STATIC_ERROR_COMP_DEADBAND_DEG == 0.10f);
    assert(PITCH_STATIC_ERROR_COMP_MAX_ERROR_DEG == 1.0f);
    assert(PITCH_STATIC_ERROR_COMP_FULL_SPEED_DEG_S == 1.0f);
    assert(PITCH_STATIC_ERROR_COMP_FADE_SPEED_DEG_S == 8.0f);
    assert(PITCH_STATIC_ERROR_COMP_VOLT_PER_DEG == 20000.0f);
    assert(PITCH_STATIC_ERROR_COMP_LIMIT == 3200.0f);
    assert(PITCH_POSITION_STEP_RESET_RAD == 5.0f * TASK_DEG_TO_RAD);
    assert(PITCH_ANGLE_INTEGRAL_SEPARATION_RAD == 1.5f * TASK_DEG_TO_RAD);
    assert(PITCH_APPROACH_SPEED_LIMIT_ENABLE == 1U);
    assert(PITCH_APPROACH_BRAKE_ACCEL_DEG_S2 == 2500.0f);
    assert(PITCH_APPROACH_MIN_SPEED_DEG_S == 25.0f);
    assert(PITCH_APPROACH_MIN_SPEED_FADE_ENABLE == 1U);
    assert(PITCH_APPROACH_MIN_SPEED_FADE_ERROR_DEG == 0.6f);
    assert(PITCH_APPROACH_MIN_SPEED_NEAR_DEG_S == 3.0f);
    assert(PITCH_APPROACH_LOW_ANGLE_SCALE_ENABLE == 1U);
    assert(PITCH_APPROACH_LOW_ANGLE_TARGET_MAX_DEG == -25.0f);
    assert(PITCH_APPROACH_LOW_ANGLE_SCALE == 0.60f);
    assert(PITCH_APPROACH_LOW_TARGET_DOWN_MAX_DEG == -45.0f);
    assert(PITCH_APPROACH_DOWN_BRAKE_DELAY_S == 0.020f);
    assert(PITCH_APPROACH_LOW_TARGET_DOWN_BRAKE_DELAY_S == 0.030f);
    assert(PITCH_APPROACH_LOW_ANGLE_UP_BRAKE_DELAY_S == 0.010f);
    assert(PITCH_APPROACH_HIGH_ANGLE_UP_ACCEL_SCALE == 2.00f);
    assert(PITCH_APPROACH_HIGH_ANGLE_UP_BRAKE_DELAY_S == 0.000f);
    assert(PITCH_APPROACH_BRAKE_FF_ENABLE == 1U);
    assert(PITCH_APPROACH_LOW_ANGLE_BRAKE_START_ERROR_DEG == 6.0f);
    assert(PITCH_APPROACH_LOW_ANGLE_BRAKE_FULL_ERROR_DEG == 4.0f);
    assert(PITCH_APPROACH_BRAKE_FF_STOP_SPEED_DEG_S == 3.0f);
    assert(PITCH_APPROACH_DOWN_BRAKE_FF_GAIN == 120.0f);
    assert(PITCH_APPROACH_DOWN_BRAKE_FF_LIMIT == 8000.0f);
    assert(PITCH_APPROACH_LOW_ANGLE_UP_BRAKE_FF_GAIN == 180.0f);
    assert(PITCH_APPROACH_LOW_ANGLE_UP_BRAKE_FF_LIMIT == 8000.0f);
    assert(PITCH_APPROACH_HIGH_ANGLE_UP_BRAKE_FF_GAIN == 90.0f);
    assert(PITCH_APPROACH_HIGH_ANGLE_UP_BRAKE_FF_LIMIT == 4500.0f);
    assert(PITCH_APPROACH_BRAKE_FF_SLEW_VOLT_PER_S == 400000.0f);
    assert(PITCH_MAX_SPEED_RPM == 70.0f);
    assert(PITCH_SPEED_KP == 172.6f);
    assert(PITCH_SPEED_KI == 12.0f);
    assert(PITCH_SPEED_KD == 0.0f);
    assert(PITCH_STARTUP_SPEED_THRESHOLD_RPM == 1.0f);
    assert(PITCH_STARTUP_MIN_VOLTAGE >= 0.0f);
    assert((PITCH_SPEED_LIMIT_ENABLE == 0U) ||
           (PITCH_TRAJECTORY_MAX_SPEED_RAD_S <=
            PITCH_MAX_SPEED_RPM * TASK_DEG_TO_RAD));
    assert((PITCH_SPEED_LIMIT_ENABLE == 0U) ||
           (PITCH_TRAJECTORY_MAX_ACCEL_RAD_S2 <= 10.0f));

    /* GM6020 的最终限幅包含前馈。前馈已经占满正向余量时，速度积分器
     * 不得继续正向累积，但必须保留反向制动能力。 */
    GM6020_Init(&pitch_with_ff, 2U, 0.0f, 10.0f);
    pitch_with_ff.feedback.online = 1U;
    pitch_with_ff.speed_pid.output_limit = 25000.0f;
    GM6020_SetVoltageFeedforward(&pitch_with_ff, GM6020_VOLTAGE_LIMIT);
    GM6020_SetSpeed(&pitch_with_ff, 1.0f);
    assert(GM6020_Update(&pitch_with_ff, 1.0f) ==
           (int16_t)GM6020_VOLTAGE_LIMIT);
    assert(pitch_with_ff.speed_pid.integral == 0.0f);
    GM6020_SetSpeed(&pitch_with_ff, -1.0f);
    assert(GM6020_Update(&pitch_with_ff, 1.0f) <
           (int16_t)GM6020_VOLTAGE_LIMIT);
    assert(pitch_with_ff.speed_pid.integral < 0.0f);

    /* Pitch 内环允许用映射后的物理 Pitch 角速度作为外部速度反馈；反馈滤波关闭时
     * 首帧必须原样进入 PID，而不能残留编码器 rpm。 */
    MotorSpeedPid_Reset(&pitch.speed_pid);
    pitch.speed_filter_initialized = 0U;
    pitch.use_external_speed_feedback = 1U;
    pitch.external_speed_rpm = 42.0f;
    GM6020_SetSpeed(&pitch, 0.0f);
    (void)GM6020_Update(&pitch, CONTROL_PERIOD_S);
    assert(fabsf(pitch.filtered_speed_rpm - 42.0f) < 0.001f);

    DM4310_Init(&yaw, 1U, YAW_SPEED_KP_CURRENT_PER_RPM,
                YAW_SPEED_KI_CURRENT_PER_RPM_S);
    MotorSpeedPid_SetGains(&yaw.speed_pid,
                           YAW_SPEED_KP_CURRENT_PER_RPM,
                           YAW_SPEED_KI_CURRENT_PER_RPM_S,
                           YAW_SPEED_KD_CURRENT_S_PER_RPM);
    yaw.speed_pid.output_limit = YAW_CURRENT_OUTPUT_LIMIT;
    DM4310_SetSpeed(&yaw, 100.0f);
    yaw_command = DM4310_Update(&yaw, CONTROL_PERIOD_S);

    /* DM4310 命令槽修正为小端后，20 会发送为 14 00，
     * 不再变成字节序错误的 5120。 */
    assert(abs(yaw_command) <= 1000);
    /* 大 yaw 误差必须请求足够的速度/电流以克服实测静摩擦；旧的 0.08 rad/s
     * 轨迹和 1.0 电流/rpm 路径只能产生约 5 个电流计数。 */
    assert(YAW_MAX_SPEED_RAD_S >= 0.5f);
    assert(YAW_TRAJECTORY_MAX_SPEED_RAD_S >= 0.4f);
    assert(YAW_SPEED_KP_CURRENT_PER_RPM >= 20.0f);
    assert(YAW_DIRECTION_TEST_MAX_CURRENT <= DM4310_CURRENT_COMMAND_LIMIT);
    /* DM4310 电流命令是有符号 16 位协议值；这里检查配置的协议安全上限，
     * 不是上面故意设小的方向测试电流。 */
    assert(DM4310_CURRENT_COMMAND_LIMIT <= 16384.0f);
    assert((YAW_VELOCITY_FF_GAIN > 0.0f) &&
           (YAW_VELOCITY_FF_GAIN <= 1.0f));
    assert(YAW_ACCELERATION_FF_CURRENT_PER_RAD_S2 > 0.0f);
    assert(YAW_ACCELERATION_FF_CURRENT_LIMIT <=
           YAW_CURRENT_OUTPUT_LIMIT);
    assert(YAW_SPEED_INTEGRAL_LIMIT_CURRENT <=
           YAW_CURRENT_OUTPUT_LIMIT);

    return 0;
}
