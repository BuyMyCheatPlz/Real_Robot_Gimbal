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
    (void)hcan;
    (void)filter;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    return HAL_OK;
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
    (void)hcan;
    (void)fifo;
    return 0U;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t fifo,
                                       CAN_RxHeaderTypeDef *header,
                                       uint8_t data[8])
{
    (void)hcan;
    (void)fifo;
    (void)header;
    (void)data;
    return HAL_ERROR;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);

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
    assert((frame->data[0] == 0U) && (frame->data[1] == 3U));
    HAL_CAN_TxMailbox0CompleteCallback(&can2);
    CanMotorBus_GetStatus(&status);
    assert(status.can2_tx_complete_count == 1U);
    assert(status.last_dm4310_id1_command == 3);

    /* Yaw startup settling must not suppress an otherwise healthy Pitch
     * loop.  The shared CAN publisher sends Pitch while forcing Yaw to an
     * exact zero command instead of running its speed PID. */
    can1_gm6020_id2.feedback.online = 1U;
    can1_gm6020_id2.feedback.speed_rpm = 0;
    MotorSpeedPid_Init(&can1_gm6020_id2.speed_pid,
                       10.0f, 0.0f, 30000.0f, 30000.0f);
    GM6020_SetSpeed(&can1_gm6020_id2, 10.0f);
    can2_dm4310_id1.speed_rpm = 50.0f;
    MotorSpeedPid_Init(&can2_dm4310_id1.speed_pid,
                       2.0f, 0.0f, 1000.0f, 1000.0f);
    DM4310_SetSpeed(&can2_dm4310_id1, 0.0f);
    sent_count = 0U;
    assert(CanMotorBus_UpdateSelected(0.001f, 1U, 0U) == HAL_OK);
    frame = find_frame(0x1FFU);
    assert(frame != 0);
    assert((frame->data[2] == 0U) && (frame->data[3] == 100U));
    frame = find_frame(DM4310_CURRENT_CONTROL_ID_1_TO_4);
    assert(frame != 0);
    assert((frame->data[0] == 0U) && (frame->data[1] == 0U));
    CanMotorBus_GetStatus(&status);
    assert((status.last_gm6020_id2_command == 100) &&
           (status.last_dm4310_id1_command == 0));

    /* CAN1 uses the same non-destructive back-pressure policy for both
     * 0x200 (C620) and 0x1FF (GM6020) command frames. */
    can1_free_level = 0U;
    assert(CanMotorBus_SendYawTestCurrent(100) == HAL_OK);
    CanMotorBus_GetStatus(&status);
    assert((status.last_send_busy_mask == 3U) &&
           (status.can1_tx_busy_count == 1U) &&
           (CanMotorBus_TxHealthy() != 0U));
    can1_free_level = 3U;

    /* A full hardware mailbox is soft back-pressure, not a transmission
     * failure.  The next control cycle submits the newest command. */
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
        assert(CanMotorBus_Update(0.001f) == HAL_ERROR);
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
