#include "dm4310.h"
#include <string.h>

#define TWO_PI_F 6.2831853071795864769f

static uint16_t big_endian_u16(const uint8_t data[2])
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t clamp_current(int16_t current)
{
    if (current > (int16_t)DM4310_CURRENT_COMMAND_LIMIT)
        return (int16_t)DM4310_CURRENT_COMMAND_LIMIT;
    if (current < (int16_t)-DM4310_CURRENT_COMMAND_LIMIT)
        return (int16_t)-DM4310_CURRENT_COMMAND_LIMIT;
    return current;
}

void DM4310_Init(DM4310_t *motor, uint8_t id, float kp, float ki)
{
    if (motor == 0) return;
    memset(motor, 0, sizeof(*motor));
    if ((id >= DM4310_MOTOR_ID_MIN) && (id <= DM4310_MOTOR_ID_MAX))
        motor->id = id;
    motor->speed_filter_alpha = 1.0f;
    MotorSpeedPid_Init(&motor->speed_pid, kp, ki,
                       DM4310_CURRENT_COMMAND_LIMIT,
                       DM4310_CURRENT_COMMAND_LIMIT);
}

void DM4310_SetSpeed(DM4310_t *motor, float speed_rpm)
{
    if (motor != 0) motor->target_speed_rpm = speed_rpm;
}

void DM4310_SetCurrentFeedforward(DM4310_t *motor, float current)
{
    if (motor != 0) motor->current_feedforward = current;
}

void DM4310_SetSpeedFilterAlpha(DM4310_t *motor, float alpha)
{
    if (motor == 0) return;
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    motor->speed_filter_alpha = alpha;
}

void DM4310_Decode(DM4310_t *motor, const uint8_t data[8], uint32_t now_ms)
{
    int16_t speed_scaled;
    if ((motor == 0) || (data == 0) ||
        (motor->id < DM4310_MOTOR_ID_MIN) ||
        (motor->id > DM4310_MOTOR_ID_MAX))
        return;

    motor->encoder = big_endian_u16(&data[0]) &
                     (uint16_t)(DM4310_ENCODER_COUNTS - 1U);
    motor->single_turn_position_rad = (float)motor->encoder * TWO_PI_F /
                                      (float)DM4310_ENCODER_COUNTS;
    speed_scaled = (int16_t)big_endian_u16(&data[2]);
    motor->speed_rpm = (float)speed_scaled / DM4310_SPEED_FEEDBACK_SCALE;
    motor->torque_current_ma = (int16_t)big_endian_u16(&data[4]);
    motor->winding_temperature = data[6];
    motor->pcb_temperature = data[7];
    motor->last_update_ms = now_ms;
    motor->online = 1U;
}

int16_t DM4310_Update(DM4310_t *motor, float dt_s)
{
    float output;
    float output_limit;
    float quantized_input;
    int16_t command;
    if ((motor == 0) || (dt_s <= 0.0f)) return 0;
    if ((motor->online == 0U) ||
        (motor->id < DM4310_MOTOR_ID_MIN) ||
        (motor->id > DM4310_MOTOR_ID_MAX))
    {
        MotorSpeedPid_Reset(&motor->speed_pid);
        motor->speed_filter_initialized = 0U;
        motor->current_quantization_error = 0.0f;
        return 0;
    }
    if (motor->speed_filter_initialized == 0U)
    {
        motor->filtered_speed_rpm = motor->speed_rpm;
        motor->speed_filter_initialized = 1U;
    }
    else
    {
        motor->filtered_speed_rpm += motor->speed_filter_alpha *
            (motor->speed_rpm - motor->filtered_speed_rpm);
    }
    output = MotorSpeedPid_CalculateFloat(&motor->speed_pid,
                                           motor->target_speed_rpm,
                                           motor->filtered_speed_rpm,
                                           dt_s) +
             motor->current_feedforward;
    output_limit = motor->speed_pid.output_limit;
    if (output_limit > DM4310_CURRENT_COMMAND_LIMIT)
        output_limit = DM4310_CURRENT_COMMAND_LIMIT;
    if (output_limit < 0.0f) output_limit = 0.0f;
    if (output > output_limit) output = output_limit;
    if (output < -output_limit) output = -output_limit;

    /* Error diffusion preserves sub-count PID/feedforward effort across
     * frames.  This avoids a large dead zone while every individual current
     * command remains inside the hard safety limit. */
    quantized_input = output + motor->current_quantization_error;
    if (quantized_input > output_limit) quantized_input = output_limit;
    if (quantized_input < -output_limit) quantized_input = -output_limit;
    command = (quantized_input >= 0.0f) ?
        (int16_t)(quantized_input + 0.5f) :
        (int16_t)(quantized_input - 0.5f);
    motor->current_quantization_error = quantized_input - (float)command;
    return command;
}

uint8_t DM4310_PackCurrentCommand(const DM4310_t *motor, int16_t current,
                                  uint16_t *std_id, uint8_t data[8])
{
    uint8_t slot;
    uint16_t encoded;
    if ((motor == 0) || (std_id == 0) || (data == 0) ||
        (motor->id < DM4310_MOTOR_ID_MIN) ||
        (motor->id > DM4310_MOTOR_ID_MAX))
        return 0U;

    if (motor->id <= 4U)
    {
        *std_id = DM4310_CURRENT_CONTROL_ID_1_TO_4;
        slot = (uint8_t)(motor->id - 1U);
    }
    else
    {
        *std_id = DM4310_CURRENT_CONTROL_ID_5_TO_8;
        slot = (uint8_t)(motor->id - 5U);
    }
    encoded = (uint16_t)clamp_current(current);
    data[slot * 2U] = (uint8_t)(encoded >> 8);
    data[slot * 2U + 1U] = (uint8_t)encoded;
    return 1U;
}
