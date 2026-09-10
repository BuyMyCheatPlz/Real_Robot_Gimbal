#include "vofa.h"
#include "gimbal_control.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "config.h"
#include "can_motor_bus.h"
#include <string.h>

#define VOFA_RX_DMA_LENGTH 64U
/* 帧格式固定为 8 路。Pitch 整定模式下输出完整的串级控制链路；关闭该模式时
 * 恢复综合状态页。发送长度必须与 channels 数组共用同一个通道数宏。 */
#define VOFA_CHANNEL_COUNT VOFA_CONTROL_CHANNEL_COUNT
#define VOFA_PAYLOAD_LENGTH (VOFA_CHANNEL_COUNT * sizeof(float))
#define VOFA_TX_LENGTH      (VOFA_PAYLOAD_LENGTH + 4U)
#define VOFA_COMMAND_QUEUE_DEPTH 4U
/* TX DMA 看门狗：一帧 ~2.5ms 发完；若完成中断丢失/出错导致 tx_busy 卡住，
 * 超过此时间强制复位重发，避免数据流永久停(表现为"数据不更新")。 */
#define VOFA_TX_TIMEOUT_MS 15U

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
static volatile uint32_t vofa_tx_start_ms;
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
    vofa_tx_start_ms = 0U;
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
    uint32_t now;
    if ((vofa_uart == 0) || (channels == 0))
        return HAL_ERROR;
    now = HAL_GetTick();
    if (tx_busy != 0U)
    {
        /* 上一帧仍在发或已卡住。超过看门狗时间说明完成中断丢失，强制复位重发。 */
        if ((now - vofa_tx_start_ms) < VOFA_TX_TIMEOUT_MS)
            return HAL_BUSY;
        tx_busy = 0U;
    }
    /* 数组形参在此处实际是指针。禁止使用 sizeof(channels)，否则在当前目标上
     * 只会复制一个 float。 */
    memcpy(tx_buffer, channels, VOFA_PAYLOAD_LENGTH);
    tx_buffer[VOFA_PAYLOAD_LENGTH] = 0x00U;
    tx_buffer[VOFA_PAYLOAD_LENGTH + 1U] = 0x00U;
    tx_buffer[VOFA_PAYLOAD_LENGTH + 2U] = 0x80U;
    tx_buffer[VOFA_PAYLOAD_LENGTH + 3U] = 0x7FU;
    tx_busy = 1U;
    vofa_tx_start_ms = now;
    if (HAL_UART_Transmit_DMA(vofa_uart, tx_buffer, VOFA_TX_LENGTH) != HAL_OK)
    {
        tx_busy = 0U;
        return HAL_ERROR;
    }
    ++vofa_tx_ok_count;
    return HAL_OK;
}

static void complete_command(void)
{
    if (assembling_length == 0U) return;
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

void VOFA_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t index;
    if (huart != vofa_uart) return;
    for (index = 0U; index < size; ++index)
    {
        char ch = (char)rx_dma_buffer[index];
        if ((ch == '\r') || (ch == '\n'))
        {
            complete_command();
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
    /* 兼容无换行的命令(如 VOFA 滑条发 "PITCH_KP_POS=75.00" 不带 \n)：
     * 本回调在 RX 空闲(或缓冲满)时触发，若仍累积内容则视作一条完整命令。 */
    complete_command();
    (void)start_rx_dma();
}

void VOFA_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != vofa_uart) return;
    assembling_length = 0U;
    tx_busy = 0U;   /* TX 出错/被中止时也复位，避免发送永久卡死 */
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
        /* I0/I1 固定为 Pitch 目标角 / 实际角（rad→°），方便所有模式下
         * 对齐阶跃、重力前馈标定和速度轴诊断数据。 */
        channels[0] = snapshot.pitch_target_rad * 57.295779513082320876f;
        channels[1] = snapshot.pitch_encoder_rad * 57.295779513082320876f;
#if (VOFA_IMU_AXIS_DEBUG_MODE != 0U)
        /* BMI088 三轴方向诊断：
         * I2/I3/I4 是芯片原始 X/Y/Z 角速度(°/s)，未做轴映射和符号；
         * I5/I6/I7 是当前配置映射后的 Roll/Pitch/Yaw 角速度(°/s)。 */
        channels[2] = snapshot.imu_gyro_raw_x_rad_s *
                      57.295779513082320876f;
        channels[3] = snapshot.imu_gyro_raw_y_rad_s *
                      57.295779513082320876f;
        channels[4] = snapshot.imu_gyro_raw_z_rad_s *
                      57.295779513082320876f;
        channels[5] = snapshot.imu_roll_rate_rad_s *
                      57.295779513082320876f;
        channels[6] = snapshot.imu_pitch_rate_rad_s *
                      57.295779513082320876f;
        channels[7] = snapshot.imu_yaw_rate_rad_s *
                      57.295779513082320876f;
#elif (VOFA_PITCH_TUNING_MODE != 0U)
        /* Pitch 调参页：
         * I2=轨迹角，I3=本轴速度目标，I4=IMU 实际速度，I5=速度 PID 输出，
         * I6=电机坐标系总前馈，I7=最终 CAN 电压命令。仅前馈标定时 I6/I7
         * 用于观察重力前馈输出。 */
        channels[2] = snapshot.pitch_profile_target_rad *
                      57.295779513082320876f;
        channels[3] = snapshot.pitch_speed_target_rpm;
        channels[4] = snapshot.pitch_speed_actual_deg_s;
        channels[5] = snapshot.pitch_feedback_output;
        channels[6] = snapshot.pitch_motor_feedforward;
        channels[7] = (float)snapshot.pitch_can_command;
#else
        /* 默认综合状态页：
         * I2/I3=Yaw 目标/实际角；I4/I5=M2006 目标/实际输出角，连发时
         * 目标每 50 ms 按 40° 阶梯递增；I6/I7=两颗 M3508 实测转速。 */
        channels[2] = snapshot.yaw_target_rad * 57.295779513082320876f;
        channels[3] = snapshot.yaw_encoder_rad * 57.295779513082320876f;
        channels[4] = snapshot.m2006_target_deg;
        channels[5] = snapshot.m2006_actual_deg;
        channels[6] = snapshot.m3508_id2_speed_rpm;
        channels[7] = snapshot.m3508_id3_speed_rpm;
#endif
        (void)VOFA_SendControlFrame(channels);
        ++vofa_heartbeat;
        wake_tick += VOFA_PERIOD_MS;
        if (osDelayUntil(wake_tick) != osOK)
            wake_tick = osKernelGetTickCount();
    }
}
