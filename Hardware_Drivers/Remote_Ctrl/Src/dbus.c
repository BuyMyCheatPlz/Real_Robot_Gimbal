#include "dbus.h"
#include "vofa.h"
#include <string.h>

#define DBUS_DMA_BUFFER_LENGTH 50U
#define DBUS_CENTER            1024.0f
#define DBUS_HALF_RANGE        660.0f

static UART_HandleTypeDef *dbus_uart;
static uint8_t dbus_dma_buffer[DBUS_DMA_BUFFER_LENGTH];
static uint8_t dbus_frame_buffer[DBUS_FRAME_LENGTH];
static uint8_t dbus_frame_length;
static volatile DBusData_t dbus_data;

static uint8_t decode_frame(const uint8_t frame[DBUS_FRAME_LENGTH])
{
    DBusData_t decoded;

    memset(&decoded, 0, sizeof(decoded));
    decoded.channel[0] = (uint16_t)((frame[0] | frame[1] << 8) & 0x07FFU);
    decoded.channel[1] = (uint16_t)((frame[1] >> 3 | frame[2] << 5) & 0x07FFU);
    decoded.channel[2] = (uint16_t)((frame[2] >> 6 | frame[3] << 2 |
                                     frame[4] << 10) & 0x07FFU);
    decoded.channel[3] = (uint16_t)((frame[4] >> 1 | frame[5] << 7) & 0x07FFU);
    decoded.channel[4] = (uint16_t)((frame[5] >> 4) & 0x03U);
    decoded.channel[5] = (uint16_t)((frame[5] >> 6) & 0x03U);
    decoded.online = 1U;
    decoded.last_update_ms = HAL_GetTick();
    dbus_data = decoded;
    return 1U;
}

static void consume_byte(uint8_t value)
{
    dbus_frame_buffer[dbus_frame_length++] = value;
    if (dbus_frame_length < DBUS_FRAME_LENGTH) return;
    (void)decode_frame(dbus_frame_buffer);
    dbus_frame_length = 0U;
}

static HAL_StatusTypeDef start_receive(void)
{
    HAL_StatusTypeDef result;
    if (dbus_uart == 0) return HAL_ERROR;
    result = HAL_UARTEx_ReceiveToIdle_DMA(dbus_uart, dbus_dma_buffer,
                                         DBUS_DMA_BUFFER_LENGTH);
    if ((result == HAL_OK) && (dbus_uart->hdmarx != 0))
        __HAL_DMA_DISABLE_IT(dbus_uart->hdmarx, DMA_IT_HT);
    return result;
}

HAL_StatusTypeDef DBus_Init(UART_HandleTypeDef *huart)
{
    if (huart == 0) return HAL_ERROR;
    dbus_uart = huart;
    memset((void *)&dbus_data, 0, sizeof(dbus_data));
    dbus_frame_length = 0U;
    return start_receive();
}

uint8_t DBus_GetData(DBusData_t *data)
{
    uint32_t primask;
    if (data == 0) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(data, (const void *)&dbus_data, sizeof(*data));
    if (primask == 0U) __enable_irq();
    return data->online;
}

float DBus_ChannelNormalized(uint8_t channel)
{
    DBusData_t snapshot;
    float value;
    if ((channel >= 16U) || (DBus_GetData(&snapshot) == 0U)) return 0.0f;
    value = ((float)snapshot.channel[channel] - DBUS_CENTER) / DBUS_HALF_RANGE;
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;
    return value;
}

void DBus_CheckOffline(uint32_t now_ms, uint32_t timeout_ms)
{
    if ((dbus_data.online != 0U) &&
        ((now_ms - dbus_data.last_update_ms) > timeout_ms))
        dbus_data.online = 0U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t index;
    if (huart != dbus_uart)
    {
        VOFA_UART_RxEventCallback(huart, size);
        return;
    }
    for (index = 0U; index < size; ++index)
        consume_byte(dbus_dma_buffer[index]);
    (void)start_receive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != dbus_uart)
    {
        VOFA_UART_ErrorCallback(huart);
        return;
    }
    dbus_frame_length = 0U;
    (void)HAL_UART_AbortReceive(huart);
    (void)start_receive();
}
