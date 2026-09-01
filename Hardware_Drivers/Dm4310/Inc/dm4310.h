#ifndef DM4310_H
#define DM4310_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define DM4310_SPEED_MODE_BASE_ID 0x200U

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint16_t motor_id;
    uint16_t master_id;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float target_velocity_rad_s;
    uint8_t state;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    uint8_t enabled;
    uint8_t online;
    uint32_t last_update_ms;
} DM4310_t;

void DM4310_Init(DM4310_t *motor, CAN_HandleTypeDef *hcan,
                 uint16_t motor_id, uint16_t master_id);
HAL_StatusTypeDef DM4310_Enable(DM4310_t *motor);
HAL_StatusTypeDef DM4310_Disable(DM4310_t *motor);
HAL_StatusTypeDef DM4310_ClearError(DM4310_t *motor);
HAL_StatusTypeDef DM4310_SetVelocity(DM4310_t *motor, float velocity_rad_s);
void DM4310_Decode(DM4310_t *motor, const uint8_t data[8], uint32_t now_ms);

#endif
