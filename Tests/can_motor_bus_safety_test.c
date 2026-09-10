#include "can_motor_bus.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    uint16_t id;
    uint8_t data[8];
} SentFrame_t;

static uint32_t fake_tick;
static uint32_t fake_error;
static uint8_t fail_transmit;
static uint8_t can1_free_level = 3U;
static uint8_t can2_free_level = 3U;
static uint32_t abort_request_count;
static SentFrame_t sent_frames[16];
static uint8_t sent_count;
static CAN_FilterTypeDef configured_filters[2];
static CAN_RxHeaderTypeDef pending_headers[2];
static uint8_t pending_data[2][8];
static uint8_t pending_rx[2];

static uint8_t can_index(const CAN_HandleTypeDef *hcan)
{
    return (hcan->Instance == (void *)1) ? 0U : 1U;
}

static void inject_rx(CAN_HandleTypeDef *hcan, uint16_t id,
                      const uint8_t data[8])
{
    uint8_t index = can_index(hcan);
    pending_headers[index].StdId = id;
    pending_headers[index].IDE = CAN_ID_STD;
    pending_headers[index].RTR = CAN_RTR_DATA;
    pending_headers[index].DLC = 8U;
    memcpy(pending_data[index], data, 8U);
    pending_rx[index] = 1U;
}

uint32_t __get_PRIMASK(void) { return 0U; }
void __disable_irq(void) {}
void __enable_irq(void) {}
uint32_t HAL_GetTick(void) { return fake_tick; }
void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub)
{
    (void)irq;
    (void)preempt;
    (void)sub;
}
void HAL_NVIC_EnableIRQ(int irq) { (void)irq; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan,
                                       CAN_FilterTypeDef *filter)
{
    configured_filters[can_index(hcan)] = *filter;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan)
{
    hcan->State = HAL_CAN_STATE_LISTENING;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan)
{
    hcan->State = HAL_CAN_STATE_READY;
    return HAL_OK;
}
HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef *hcan)
{
    return hcan->State;
}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan,
                                               uint32_t notifications)
{
    (void)hcan;
    (void)notifications;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan,
                                       CAN_TxHeaderTypeDef *header,
                                       uint8_t data[8], uint32_t *mailbox)
{
    (void)hcan;
    *mailbox = 0U;
    if (fail_transmit != 0U) return HAL_ERROR;
    if (sent_count < (uint8_t)(sizeof(sent_frames) / sizeof(sent_frames[0])))
    {
        sent_frames[sent_count].id = (uint16_t)header->StdId;
        memcpy(sent_frames[sent_count].data, data, 8U);
        ++sent_count;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan,
                                        uint32_t mailboxes)
{
    (void)hcan;
    (void)mailboxes;
    ++abort_request_count;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    fake_error = 0U;
    return HAL_OK;
}
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    return fake_error;
}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *hcan)
{
    return (hcan->Instance == (void *)1) ? can1_free_level : can2_free_level;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *hcan,
                                   uint32_t fifo)
{
    (void)fifo;
    return pending_rx[can_index(hcan)];
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t fifo,
                                       CAN_RxHeaderTypeDef *header,
                                       uint8_t data[8])
{
    (void)fifo;
    uint8_t index = can_index(hcan);
    if (pending_rx[index] == 0U) return HAL_ERROR;
    *header = pending_headers[index];
    memcpy(data, pending_data[index], 8U);
    pending_rx[index] = 0U;
    return HAL_OK;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

static const SentFrame_t *find_frame(uint16_t id)
{
    uint8_t index;
    for (index = 0U; index < sent_count; ++index)
        if (sent_frames[index].id == id) return &sent_frames[index];
    return 0;
}

int main(void)
{
    CAN_HandleTypeDef can1;
    CAN_HandleTypeDef can2;
    CanMotorBusStatus_t status;
    const SentFrame_t *frame;
    uint8_t index;

    memset(&can1, 0, sizeof(can1));
    memset(&can2, 0, sizeof(can2));
    can1.Instance = (void *)1;
    can2.Instance = (void *)2;
    assert(CanMotorBus_Init(&can1, &can2) == HAL_OK);
    /* 已配置的滤波器必须放行所有实装电机的反馈 ID。 */
    assert((configured_filters[0].FilterIdHigh == (0x202U << 5)) &&
           (configured_filters[0].FilterIdLow == (0x203U << 5)) &&
           (configured_filters[0].FilterMaskIdHigh == (0x206U << 5)) &&
           (configured_filters[0].FilterMaskIdLow == (0x206U << 5)));
    assert((configured_filters[1].FilterIdHigh == (0x205U << 5)) &&
           (configured_filters[1].FilterIdLow == (0x301U << 5)) &&
           (configured_filters[1].FilterMaskIdHigh == (0x205U << 5)) &&
           (configured_filters[1].FilterMaskIdLow == (0x301U << 5)));

    /* FIFO0 接收的帧必须通过中断回调进入对应电机解码器，不能只增加计数。 */
    {
        const uint8_t dji_feedback[8] = {0x12U, 0x34U, 0x00U, 0x64U,
                                         0x00U, 0x00U, 0x32U, 0U};
        const uint8_t dm_feedback[8] = {0x01U, 0x23U, 0x00U, 0x64U,
                                        0x00U, 0x32U, 0x28U, 0x29U};
        fake_tick = 42U;
        inject_rx(&can1, 0x206U, dji_feedback);
        HAL_CAN_RxFifo0MsgPendingCallback(&can1);
        inject_rx(&can2, 0x301U, dm_feedback);
        HAL_CAN_RxFifo0MsgPendingCallback(&can2);
        CanMotorBus_GetStatus(&status);
        assert((status.can1_rx_count == 1U) && (status.can2_rx_count == 1U));
        assert((can1_gm6020_id2.feedback.online != 0U) &&
               (can1_gm6020_id2.feedback.last_update_ms == fake_tick));
        assert((can2_dm4310_id1.online != 0U) &&
               (can2_dm4310_id1.encoder == 0x123U) &&
               (can2_dm4310_id1.speed_rpm == 1.0f));
    }

    /* DM4310 恢复反馈前可能需要持续收到控制保活帧。因此反馈丢失时仍要发送
     * 0 电流 0x3FE 帧，不能让电机总线静默并造成离线状态自锁。 */
    sent_count = 0U;
    assert(CanMotorBus_StopGimbal(0.001f) == HAL_OK);
    frame = find_frame(DM4310_CURRENT_CONTROL_ID_1_TO_4);
    assert(frame != 0);
    assert((frame->data[0] == 0U) && (frame->data[1] == 0U));

    can2_dm4310_id1.online = 1U;
    sent_count = 0U;
    assert(CanMotorBus_SendYawTestCurrent(100) == HAL_OK);
    frame = find_frame(0x200U);
    assert(frame != 0);
    assert((frame->data[0] == 0U) && (frame->data[7] == 0U));
    frame = find_frame(0x1FFU);
    assert(frame != 0);
    assert((frame->data[0] == 0U) && (frame->data[7] == 0U));
    frame = find_frame(DM4310_CURRENT_CONTROL_ID_1_TO_4);
    assert(frame != 0);
    assert((frame->data[0] == 100U) && (frame->data[1] == 0U));
    HAL_CAN_TxMailbox0CompleteCallback(&can2);
    CanMotorBus_GetStatus(&status);
    assert(status.can2_tx_complete_count == 1U);
    assert(status.last_dm4310_id1_command == 100);

    /* Yaw 启动稳定过程不能抑制健康的 Pitch 回路。共享 CAN 发布器会发送 Pitch，
     * 同时强制 Yaw 使用精确的零命令，而不是运行其速度 PID。 */
    can1_gm6020_id2.feedback.online = 1U;
    can1_gm6020_id2.feedback.speed_rpm = 0;
    can1_gm6020_id2.filtered_speed_rpm = 0.0f;
    MotorSpeedPid_Init(&can1_gm6020_id2.speed_pid,
                       10.0f, 0.0f, 30000.0f, 30000.0f);
    GM6020_SetSpeed(&can1_gm6020_id2, 10.0f);
    can2_dm4310_id1.speed_rpm = 50.0f;
    MotorSpeedPid_Init(&can2_dm4310_id1.speed_pid,
                       2.0f, 0.0f, 1000.0f, 1000.0f);
    DM4310_SetSpeed(&can2_dm4310_id1, 0.0f);
    sent_count = 0U;
    fake_tick += CAN_COMMAND_PERIOD_MS;
    assert(CanMotorBus_UpdateSelected(0.001f, 1U, 0U) == HAL_OK);
    /* GM6020 ID2 的反馈帧是 0x206，电压命令位于 0x1FF 的 ID2 槽位，
     * 即字节 2..3。 */
    frame = find_frame(0x1FFU);
    assert(frame != 0);
    assert((frame->data[2] == 0x1FU) && (frame->data[3] == 0x40U));
    frame = find_frame(DM4310_CURRENT_CONTROL_ID_1_TO_4);
    assert(frame != 0);
    assert((frame->data[0] == 0U) && (frame->data[1] == 0U));
    CanMotorBus_GetStatus(&status);
    assert((status.last_gm6020_id2_command == 8000) &&
           (status.last_dm4310_id1_command == 0));

    /* CAN1 对 0x200（C620）和 0x1FF（GM6020）命令帧使用相同的非破坏性反压策略。 */
    can1_free_level = 0U;
    assert(CanMotorBus_SendYawTestCurrent(100) == HAL_OK);
    CanMotorBus_GetStatus(&status);
    assert((status.last_send_busy_mask == 3U) &&
           (status.can1_tx_busy_count == 1U) &&
           (CanMotorBus_TxHealthy() != 0U));
    can1_free_level = 3U;

    /* 硬件邮箱已满属于软反压，不是发送失败；下一个控制周期会提交最新命令。 */
    can2_free_level = 0U;
    for (index = 0U; index < 5U; ++index)
        assert(CanMotorBus_SendYawTestCurrent(100) == HAL_OK);
    CanMotorBus_GetStatus(&status);
    assert((status.last_send_busy_mask == 8U) &&
           (status.can2_tx_busy_count == 5U) &&
           (CanMotorBus_TxHealthy() != 0U));
    can2_free_level = 3U;

    fail_transmit = 1U;
    for (index = 0U; index < CAN_MOTOR_TX_FAILURE_LATCH_COUNT; ++index)
    {
        fake_tick += CAN_COMMAND_PERIOD_MS;
        assert(CanMotorBus_Update(0.001f) == HAL_ERROR);
    }
    CanMotorBus_GetStatus(&status);
    assert((status.fault_latched != 0U) &&
           (CanMotorBus_TxHealthy() == 0U));

    fail_transmit = 0U;
    for (index = 0U; index < CAN_MOTOR_TX_RECOVERY_FRAME_COUNT; ++index)
        assert(CanMotorBus_StopAll() == HAL_ERROR);
    assert(CanMotorBus_TxHealthy() != 0U);
    assert(abort_request_count == 0U);
    assert(CanMotorBus_StopAll() == HAL_OK);

    fake_error = HAL_CAN_ERROR_BOF;
    HAL_CAN_ErrorCallback(&can1);
    CanMotorBus_GetStatus(&status);
    assert((status.fault_latched == 0U) &&
           (status.bus_error_count == 1U));
    return 0;
}
