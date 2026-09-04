#include "vofa.h"
#include "gimbal_control.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "config.h"
#include <string.h>

#define VOFA_RX_DMA_LENGTH 64U
#define VOFA_CHANNEL_COUNT VOFA_CONTROL_CHANNEL_COUNT
#define VOFA_PAYLOAD_LENGTH (VOFA_CHANNEL_COUNT * sizeof(float))
#define VOFA_TX_LENGTH      (VOFA_PAYLOAD_LENGTH + 4U)
#define VOFA_COMMAND_QUEUE_DEPTH 4U

static UART_HandleTypeDef *vofa_uart;
static uint8_t rx_dma_buffer[VOFA_RX_DMA_LENGTH];
static char assembling_command[VOFA_COMMAND_MAX_LENGTH];
//
static volatile uint16_t assembling_length;
static char completed_command[VOFA_COMMAND_QUEUE_DEPTH][VOFA_COMMAND_MAX_LENGTH];
static volatile uint8_t command_read_index;
static volatile uint8_t command_write_index;
static volatile uint8_t command_count;
static uint8_t tx_buffer[VOFA_TX_LENGTH];
static volatile uint8_t tx_busy;
static volatile uint32_t vofa_heartbeat;
static volatile uint32_t vofa_tx_ok_count;

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

HAL_StatusTypeDef VOFA_SendControlFrame(
    const float channels[VOFA_CONTROL_CHANNEL_COUNT])
{
    if ((vofa_uart == 0) || (channels == 0) || (tx_busy != 0U))
        return HAL_BUSY;
    /* 数组形参在此处实际是指针。禁止使用 sizeof(channels)，否则在当前目标上
     * 只会复制一个 float。 */
    memcpy(tx_buffer, channels, VOFA_PAYLOAD_LENGTH);
    tx_buffer[VOFA_PAYLOAD_LENGTH] = 0x00U;
    tx_buffer[VOFA_PAYLOAD_LENGTH + 1U] = 0x00U;
    tx_buffer[VOFA_PAYLOAD_LENGTH + 2U] = 0x80U;
    tx_buffer[VOFA_PAYLOAD_LENGTH + 3U] = 0x7FU;
    tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(vofa_uart, tx_buffer, VOFA_TX_LENGTH) != HAL_OK)
    {
        tx_busy = 0U;
        return HAL_ERROR;
    }
    ++vofa_tx_ok_count;
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
    float channels[VOFA_CONTROL_CHANNEL_COUNT];
    uint32_t primask;
    uint32_t wake_tick;
    (void)argument;
    (void)VOFA_Init(&huart4);
    wake_tick = osKernelGetTickCount();

    for (;;)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        memcpy(&snapshot, (const void *)&gimbal_control_state,
               sizeof(snapshot));
        if (primask == 0U) __enable_irq();
        /* Pitch GM6020 diagnostics: target, feedback, PID output and health. */
        channels[0] = snapshot.pitch_target_rad * 57.295779513082320876f;
        channels[1] = snapshot.pitch_encoder_rad * 57.295779513082320876f;
        channels[2] = (snapshot.pitch_target_rad - snapshot.pitch_encoder_rad) *
                      57.295779513082320876f;
        channels[3] = snapshot.pitch_speed_target_rpm;
        channels[4] = snapshot.pitch_speed_rpm;
        channels[5] = (float)snapshot.pitch_can_command;
        channels[6] = snapshot.gravity_feedforward;
        channels[7] = (float)snapshot.can1_gm6020_id2_online;
        channels[8] = snapshot.imu_pitch_rad * 57.295779513082320876f;
        channels[9] = (float)snapshot.active;
        channels[10] = (float)snapshot.feedback_healthy;
        channels[11] = (float)snapshot.control_inhibit_flags;
        channels[12] = (float)snapshot.can_last_send_failure_mask;
        channels[13] = (float)snapshot.can1_tx_free_level;
        channels[14] = (float)snapshot.can_tx_failure_count;
        channels[15] = (float)snapshot.can1_m3508_id2_online;
        (void)VOFA_SendControlFrame(channels);
        ++vofa_heartbeat;
        wake_tick += VOFA_PERIOD_MS;
        if (osDelayUntil(wake_tick) != osOK)
            wake_tick = osKernelGetTickCount();
    }
}
