#ifndef VOFA_H
#define VOFA_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define VOFA_COMMAND_MAX_LENGTH 48U
#define VOFA_CONTROL_CHANNEL_COUNT 18U

HAL_StatusTypeDef VOFA_Init(UART_HandleTypeDef *huart);
uint8_t VOFA_GetCommand(char command[VOFA_COMMAND_MAX_LENGTH]);
HAL_StatusTypeDef VOFA_SendControlFrame(
    const float channels[VOFA_CONTROL_CHANNEL_COUNT]);
void VOFA_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void VOFA_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
