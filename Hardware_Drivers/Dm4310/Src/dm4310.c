#include "dm4310.h"
#include <string.h>

#define DM4310_POSITION_MIN (-12.5f)
#define DM4310_POSITION_MAX (12.5f)
#define DM4310_VELOCITY_MIN (-30.0f)
#define DM4310_VELOCITY_MAX (30.0f)
#define DM4310_TORQUE_MIN   (-10.0f)
#define DM4310_TORQUE_MAX   (10.0f)

static float uint_to_float(uint16_t value, float min, float max, uint8_t bits)
{
    const uint32_t full_scale = (1UL << bits) - 1UL;
    return ((float)value * (max - min) / (float)full_scale) + min;
}

static HAL_StatusTypeDef send_frame(DM4310_t *motor, const uint8_t *data,
                                    uint8_t length)
{
    CAN_TxHeaderTypeDef header;
    uint32_t mailbox;
    if ((motor == 0) || (motor->hcan == 0) || (data == 0)) return HAL_ERROR;
    header.StdId = DM4310_SPEED_MODE_BASE_ID + motor->motor_id;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = length;
    header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(motor->hcan, &header, (uint8_t *)data, &mailbox);
}

static HAL_StatusTypeDef send_special(DM4310_t *motor, uint8_t command)
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                       0xFFU, 0xFFU, 0xFFU, command};
    return send_frame(motor, data, 8U);
}

void DM4310_Init(DM4310_t *motor, CAN_HandleTypeDef *hcan,
                 uint16_t motor_id, uint16_t master_id)
{
    if (motor == 0) return;
    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->master_id = master_id;
}

HAL_StatusTypeDef DM4310_Enable(DM4310_t *motor)
{
    HAL_StatusTypeDef status = send_special(motor, 0xFCU);
    if ((motor != 0) && (status == HAL_OK)) motor->enabled = 1U;
    return status;
}

HAL_StatusTypeDef DM4310_Disable(DM4310_t *motor)
{
    if (motor != 0) motor->target_velocity_rad_s = 0.0f;
    if (motor != 0) motor->enabled = 0U;
    return send_special(motor, 0xFDU);
}

HAL_StatusTypeDef DM4310_ClearError(DM4310_t *motor)
{
    return send_special(motor, 0xFBU);
}

HAL_StatusTypeDef DM4310_SetVelocity(DM4310_t *motor, float velocity_rad_s)
{
    uint8_t data[4];
    if (motor == 0) return HAL_ERROR;
    if (velocity_rad_s > DM4310_VELOCITY_MAX) velocity_rad_s = DM4310_VELOCITY_MAX;
    if (velocity_rad_s < DM4310_VELOCITY_MIN) velocity_rad_s = DM4310_VELOCITY_MIN;
    motor->target_velocity_rad_s = velocity_rad_s;
    memcpy(data, &velocity_rad_s, sizeof(data));
    return send_frame(motor, data, 4U);
}

void DM4310_Decode(DM4310_t *motor, const uint8_t data[8], uint32_t now_ms)
{
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    if ((motor == 0) || (data == 0)) return;
    if ((data[0] & 0x0FU) != (motor->motor_id & 0x0FU)) return;

    motor->state = data[0] >> 4;
    position = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    velocity = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    torque = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);
    motor->position_rad = uint_to_float(position, DM4310_POSITION_MIN,
                                        DM4310_POSITION_MAX, 16U);
    motor->velocity_rad_s = uint_to_float(velocity, DM4310_VELOCITY_MIN,
                                          DM4310_VELOCITY_MAX, 12U);
    motor->torque_nm = uint_to_float(torque, DM4310_TORQUE_MIN,
                                     DM4310_TORQUE_MAX, 12U);
    motor->mos_temperature = data[6];
    motor->rotor_temperature = data[7];
    motor->last_update_ms = now_ms;
    motor->online = 1U;
}
