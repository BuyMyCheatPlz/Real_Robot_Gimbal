#ifndef CAN_MOTOR_BUS_H
#define CAN_MOTOR_BUS_H

#include "stm32f4xx_hal.h"
#include "m3508.h"
#include "m2006.h"
#include "gm6020.h"
#include "dm4310.h"

#define CAN_MOTOR_OFFLINE_TIMEOUT_MS 100U

extern M3508_t can1_m3508_id2;
extern M3508_t can1_m3508_id3;
extern GM6020_t can1_gm6020_id2;
extern M2006_t can2_m2006_id5;
extern DM4310_t can2_dm4310_id1;

HAL_StatusTypeDef CanMotorBus_Init(CAN_HandleTypeDef *can1,
                                   CAN_HandleTypeDef *can2);
HAL_StatusTypeDef CanMotorBus_Update(float dt_s);
HAL_StatusTypeDef CanMotorBus_StopAll(void);
void CanMotorBus_CheckOffline(uint32_t now_ms);

#endif
