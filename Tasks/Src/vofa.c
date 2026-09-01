#include "vofa.h"
#include "gimbal_control.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "config.h"
#include <math.h>
#include <string.h>

#define VOFA_RX_DMA_LENGTH 64U
#define VOFA_CHANNEL_COUNT 4U
#define VOFA_TX_LENGTH     (VOFA_CHANNEL_COUNT * sizeof(float) + 4U)
#define VOFA_COMMAND_QUEUE_DEPTH 4U
#define RAD_TO_DEG         57.295779513082320876f

static UART_HandleTypeDef *vofa_uart;
static uint8_t rx_dma_buffer[VOFA_RX_DMA_LENGTH];
static char assembling_command[VOFA_COMMAND_MAX_LENGTH];
static volatile uint16_t assembling_length;
static char completed_command[VOFA_COMMAND_QUEUE_DEPTH][VOFA_COMMAND_MAX_LENGTH];
static volatile uint8_t command_read_index;
static volatile uint8_t command_write_index;
static volatile uint8_t command_count;
static uint8_t tx_buffer[VOFA_TX_LENGTH];
static volatile uint8_t tx_busy;

static HAL_StatusTypeDef start_rx_dma(void)
{
    HAL_StatusTypeDef status;
    if (vofa_uart == 0) return HAL_ERROR;
    status = HAL_UARTEx_ReceiveToIdle_DMA(vofa_uart, rx_dma_buffer,
                                         VOFA_RX_DMA_LENGTH);
    if ((status == HAL_OK) && (vofa_uart->hdmarx != 0))
        __HAL_DMA_DISABLE_IT(vofa_uart->hdmarx, DMA_IT_HT);
    return status;
}

static float normalize_degrees(float radians)
{
    float degrees = fmodf(radians * RAD_TO_DEG, 360.0f);
    if (degrees < 0.0f) degrees += 360.0f;
    return degrees;
}

HAL_StatusTypeDef VOFA_Init(UART_HandleTypeDef *huart)
{
    if (huart == 0) return HAL_ERROR;
    vofa_uart = huart;
    assembling_length = 0U;
    command_read_index = 0U;
    command_write_index = 0U;
    command_count = 0U;
    tx_busy = 0U;
    return start_rx_dma();
}

uint8_t VOFA_GetCommand(char command[VOFA_COMMAND_MAX_LENGTH])
{
    uint32_t primask;
    if ((command == 0) || (command_count == 0U)) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(command, completed_command[command_read_index],
           VOFA_COMMAND_MAX_LENGTH);
    command_read_index = (uint8_t)((command_read_index + 1U) %
                                   VOFA_COMMAND_QUEUE_DEPTH);
    --command_count;
    if (primask == 0U) __enable_irq();
    return 1U;
}

HAL_StatusTypeDef VOFA_SendAngles(float pitch_target_deg,
                                  float pitch_actual_deg,
                                  float yaw_target_deg,
                                  float yaw_actual_deg)
{
    float channels[VOFA_CHANNEL_COUNT];
    if ((vofa_uart == 0) || (tx_busy != 0U)) return HAL_BUSY;
    channels[0] = pitch_target_deg;
    channels[1] = pitch_actual_deg;
    channels[2] = yaw_target_deg;
    channels[3] = yaw_actual_deg;
    memcpy(tx_buffer, channels, sizeof(channels));
    tx_buffer[16] = 0x00U;
    tx_buffer[17] = 0x00U;
    tx_buffer[18] = 0x80U;
    tx_buffer[19] = 0x7FU;
    tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(vofa_uart, tx_buffer, VOFA_TX_LENGTH) != HAL_OK)
    {
        tx_busy = 0U;
        return HAL_ERROR;
    }
    return HAL_OK;
}

void VOFA_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t index;
    if (huart != vofa_uart) return;
    for (index = 0U; index < size; ++index)
    {
        char ch = (char)rx_dma_buffer[index];
        if ((ch == '\r') || (ch == '\n'))
        {
            if (assembling_length != 0U)
            {
                assembling_command[assembling_length] = '\0';
                if (command_count >= VOFA_COMMAND_QUEUE_DEPTH)
                {
                    command_read_index = (uint8_t)((command_read_index + 1U) %
                                                   VOFA_COMMAND_QUEUE_DEPTH);
                    --command_count;
                }
                memcpy(completed_command[command_write_index], assembling_command,
                       VOFA_COMMAND_MAX_LENGTH);
                command_write_index = (uint8_t)((command_write_index + 1U) %
                                                VOFA_COMMAND_QUEUE_DEPTH);
                ++command_count;
                assembling_length = 0U;
            }
        }
        else if (assembling_length < (VOFA_COMMAND_MAX_LENGTH - 1U))
        {
            assembling_command[assembling_length++] = ch;
        }
        else
        {
            assembling_length = 0U;
        }
    }
    (void)start_rx_dma();
}

void VOFA_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != vofa_uart) return;
    assembling_length = 0U;
    (void)HAL_UART_AbortReceive(huart);
    (void)start_rx_dma();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == vofa_uart) tx_busy = 0U;
}

void VOFA_print(void *argument)
{
    GimbalControlState_t snapshot;
    uint32_t primask;
    (void)argument;
    (void)VOFA_Init(&huart4);

    for (;;)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        memcpy(&snapshot, (const void *)&gimbal_control_state,
               sizeof(snapshot));
        if (primask == 0U) __enable_irq();
        (void)VOFA_SendAngles(
            normalize_degrees(snapshot.pitch_target_rad),
            normalize_degrees(snapshot.pitch_encoder_rad),
            normalize_degrees(snapshot.yaw_target_rad),
            normalize_degrees(snapshot.yaw_encoder_rad));
        osDelay(VOFA_PERIOD_MS);
    }
}
