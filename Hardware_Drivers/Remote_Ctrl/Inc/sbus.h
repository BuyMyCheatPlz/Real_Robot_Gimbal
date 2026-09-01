#ifndef SBUS_H
#define SBUS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define SBUS_CHANNEL_COUNT 18U
#define SBUS_FRAME_LENGTH  25U

typedef struct
{
    uint16_t channel[SBUS_CHANNEL_COUNT];
    uint8_t frame_lost;
    uint8_t failsafe;
    uint8_t online;
    uint32_t last_update_ms;
} SBusData_t;

HAL_StatusTypeDef SBus_Init(UART_HandleTypeDef *huart);
uint8_t SBus_GetData(SBusData_t *data);
float SBus_ChannelNormalized(uint8_t channel);
void SBus_CheckOffline(uint32_t now_ms, uint32_t timeout_ms);

#endif
