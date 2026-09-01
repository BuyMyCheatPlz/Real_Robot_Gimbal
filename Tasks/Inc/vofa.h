#ifndef VOFA_H
#define VOFA_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define VOFA_COMMAND_MAX_LENGTH 48U

HAL_StatusTypeDef VOFA_Init(UART_HandleTypeDef *huart);
uint8_t VOFA_GetCommand(char command[VOFA_COMMAND_MAX_LENGTH]);
HAL_StatusTypeDef VOFA_SendAngles(float pitch_target_deg,
                                  float pitch_actual_deg,
                                  float yaw_target_deg,
                                  float yaw_actual_deg);
void VOFA_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void VOFA_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
