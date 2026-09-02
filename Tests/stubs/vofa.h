#ifndef TEST_VOFA_H
#define TEST_VOFA_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void VOFA_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void VOFA_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
