#include "sbus.h"
#include "vofa.h"
#include <string.h>

#define SBUS_DMA_BUFFER_LENGTH 50U
#define SBUS_CENTER            992.0f
#define SBUS_HALF_RANGE        820.0f

static UART_HandleTypeDef *sbus_uart;
static uint8_t sbus_dma_buffer[SBUS_DMA_BUFFER_LENGTH];
static volatile SBusData_t sbus_data;

static uint8_t frame_end_valid(uint8_t value)
{
    return (uint8_t)((value == 0x00U) || ((value & 0x0FU) == 0x04U));
}

static void decode_frame(const uint8_t frame[SBUS_FRAME_LENGTH])
{
    SBusData_t decoded;
    if ((frame[0] != 0x0FU) || (frame_end_valid(frame[24]) == 0U)) return;

    memset(&decoded, 0, sizeof(decoded));
    decoded.channel[0]  = (uint16_t)((frame[1]       | frame[2]  << 8) & 0x07FFU);
    decoded.channel[1]  = (uint16_t)((frame[2] >> 3  | frame[3]  << 5) & 0x07FFU);
    decoded.channel[2]  = (uint16_t)((frame[3] >> 6  | frame[4]  << 2 | frame[5] << 10) & 0x07FFU);
    decoded.channel[3]  = (uint16_t)((frame[5] >> 1  | frame[6]  << 7) & 0x07FFU);
    decoded.channel[4]  = (uint16_t)((frame[6] >> 4  | frame[7]  << 4) & 0x07FFU);
    decoded.channel[5]  = (uint16_t)((frame[7] >> 7  | frame[8]  << 1 | frame[9] << 9) & 0x07FFU);
    decoded.channel[6]  = (uint16_t)((frame[9] >> 2  | frame[10] << 6) & 0x07FFU);
    decoded.channel[7]  = (uint16_t)((frame[10] >> 5 | frame[11] << 3) & 0x07FFU);
    decoded.channel[8]  = (uint16_t)((frame[12]      | frame[13] << 8) & 0x07FFU);
    decoded.channel[9]  = (uint16_t)((frame[13] >> 3 | frame[14] << 5) & 0x07FFU);
    decoded.channel[10] = (uint16_t)((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FFU);
    decoded.channel[11] = (uint16_t)((frame[16] >> 1 | frame[17] << 7) & 0x07FFU);
    decoded.channel[12] = (uint16_t)((frame[17] >> 4 | frame[18] << 4) & 0x07FFU);
    decoded.channel[13] = (uint16_t)((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FFU);
    decoded.channel[14] = (uint16_t)((frame[20] >> 2 | frame[21] << 6) & 0x07FFU);
    decoded.channel[15] = (uint16_t)((frame[21] >> 5 | frame[22] << 3) & 0x07FFU);
    decoded.channel[16] = (frame[23] & 0x01U) ? 2047U : 0U;
    decoded.channel[17] = (frame[23] & 0x02U) ? 2047U : 0U;
    decoded.frame_lost = (frame[23] & 0x04U) ? 1U : 0U;
    decoded.failsafe = (frame[23] & 0x08U) ? 1U : 0U;
    decoded.online = 1U;
    decoded.last_update_ms = HAL_GetTick();
    sbus_data = decoded;
}

static HAL_StatusTypeDef start_receive(void)
{
    HAL_StatusTypeDef result;
    if (sbus_uart == 0) return HAL_ERROR;
    result = HAL_UARTEx_ReceiveToIdle_DMA(sbus_uart, sbus_dma_buffer,
                                         SBUS_DMA_BUFFER_LENGTH);
    if ((result == HAL_OK) && (sbus_uart->hdmarx != 0))
        __HAL_DMA_DISABLE_IT(sbus_uart->hdmarx, DMA_IT_HT);
    return result;
}

HAL_StatusTypeDef SBus_Init(UART_HandleTypeDef *huart)
{
    if (huart == 0) return HAL_ERROR;
    sbus_uart = huart;
    memset((void *)&sbus_data, 0, sizeof(sbus_data));
    return start_receive();
}

uint8_t SBus_GetData(SBusData_t *data)
{
    uint32_t primask;
    if (data == 0) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(data, (const void *)&sbus_data, sizeof(*data));
    if (primask == 0U) __enable_irq();
    return data->online;
}

float SBus_ChannelNormalized(uint8_t channel)
{
    SBusData_t snapshot;
    float value;
    if ((channel >= 16U) || (SBus_GetData(&snapshot) == 0U)) return 0.0f;
    value = ((float)snapshot.channel[channel] - SBUS_CENTER) / SBUS_HALF_RANGE;
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;
    return value;
}

void SBus_CheckOffline(uint32_t now_ms, uint32_t timeout_ms)
{
    if ((sbus_data.online != 0U) &&
        ((now_ms - sbus_data.last_update_ms) > timeout_ms))
        sbus_data.online = 0U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t index;
    if (huart != sbus_uart)
    {
        VOFA_UART_RxEventCallback(huart, size);
        return;
    }
    for (index = 0U; (index + SBUS_FRAME_LENGTH) <= size; ++index)
    {
        if (sbus_dma_buffer[index] == 0x0FU)
        {
            decode_frame(&sbus_dma_buffer[index]);
            index += SBUS_FRAME_LENGTH - 1U;
        }
    }
    (void)start_receive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != sbus_uart)
    {
        VOFA_UART_ErrorCallback(huart);
        return;
    }
    (void)HAL_UART_AbortReceive(huart);
    (void)start_receive();
}
