#include "sbus.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

static uint8_t *receive_buffer;
static uint16_t receive_capacity;
static uint32_t fake_tick;

uint32_t __get_PRIMASK(void) { return 0U; }
void __disable_irq(void) {}
void __enable_irq(void) {}
uint32_t HAL_GetTick(void) { return fake_tick; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                               uint8_t *data,
                                               uint16_t length)
{
    (void)huart;
    receive_buffer = data;
    receive_capacity = length;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart)
{
    (void)huart;
    return HAL_OK;
}
void VOFA_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)huart;
    (void)size;
}
void VOFA_UART_ErrorCallback(UART_HandleTypeDef *huart) { (void)huart; }
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

static void deliver(UART_HandleTypeDef *uart, const uint8_t *data,
                    uint16_t length)
{
    assert(length <= receive_capacity);
    memcpy(receive_buffer, data, length);
    HAL_UARTEx_RxEventCallback(uart, length);
}

int main(void)
{
    UART_HandleTypeDef uart;
    DMA_HandleTypeDef dma;
    SBusData_t decoded;
    uint8_t valid[SBUS_FRAME_LENGTH] = {0};
    uint8_t noisy[30];

    uart.hdmarx = &dma;
    valid[0] = 0x0FU;
    valid[23] = 0x08U;
    valid[24] = 0x00U;
    assert(SBus_Init(&uart) == HAL_OK);

    fake_tick = 10U;
    deliver(&uart, valid, 10U);
    assert(SBus_GetData(&decoded) == 0U);
    deliver(&uart, &valid[10], 15U);
    assert(SBus_GetData(&decoded) != 0U);
    assert((decoded.failsafe != 0U) && (decoded.last_update_ms == 10U));

    assert(SBus_Init(&uart) == HAL_OK);
    memset(noisy, 0x55, sizeof(noisy));
    memcpy(&noisy[5], valid, sizeof(valid));
    fake_tick = 20U;
    deliver(&uart, noisy, sizeof(noisy));
    assert(SBus_GetData(&decoded) != 0U);
    assert((decoded.failsafe != 0U) && (decoded.last_update_ms == 20U));
    return 0;
}
