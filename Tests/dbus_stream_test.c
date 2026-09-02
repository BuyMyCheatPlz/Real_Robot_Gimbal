#include "dbus.h"
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
    DBusData_t decoded;
    uint8_t valid[DBUS_FRAME_LENGTH] = {0};

    uart.hdmarx = &dma;
    valid[0] = 0x00U;
    valid[1] = 0x04U;
    valid[2] = 0x20U;
    valid[3] = 0x00U;
    valid[4] = 0x00U;
    valid[5] = (uint8_t)((2U << 4) | (3U << 6));
    assert(DBus_Init(&uart) == HAL_OK);

    fake_tick = 5U;
    deliver(&uart, valid, DBUS_FRAME_LENGTH);
    assert(DBus_GetData(&decoded) != 0U);
    assert((decoded.channel[4] == 2U) && (decoded.channel[5] == 3U));
    assert(decoded.last_update_ms == 5U);

    assert(DBus_Init(&uart) == HAL_OK);
    fake_tick = 10U;
    deliver(&uart, valid, 8U);
    assert(DBus_GetData(&decoded) == 0U);
    deliver(&uart, &valid[8], DBUS_FRAME_LENGTH - 8U);
    assert(DBus_GetData(&decoded) != 0U);
    assert((decoded.channel[4] == 2U) && (decoded.channel[5] == 3U));
    assert(decoded.last_update_ms == 10U);
    return 0;
}
