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
    assert(CanMotorBus_Init(&can1, &can2) == HAL_OK);

    can1_m3508_id2.feedback.online = 1U;
    can1_m3508_id3.feedback.online = 1U;
    can2_m2006_id5.feedback.online = 1U;
    MotorSpeedPid_Init(&can1_m3508_id2.speed_pid, 1.0f, 0.0f,
                       M3508_CURRENT_LIMIT, M3508_CURRENT_LIMIT);
    M3508_SetSpeed(&can1_m3508_id2, 100.0f);
    sent_count = 0U;
    assert(CanMotorBus_StopGimbal(0.001f) == HAL_OK);
    assert(can1_m3508_id2.target_speed_rpm == 100.0f);
    frame = find_frame(0x200U);
    assert(frame != 0);
    assert((frame->data[2] == 0x00U) && (frame->data[3] == 0x64U));
    frame = find_frame(DM4310_CURRENT_CONTROL_ID_1_TO_4);
    assert(frame != 0);
    assert((frame->data[0] == 0U) && (frame->data[1] == 0U));

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
    assert(CanMotorBus_StopAll() == HAL_OK);

    fake_error = HAL_CAN_ERROR_BOF;
    HAL_CAN_ErrorCallback(&can1);
    CanMotorBus_GetStatus(&status);
    assert((status.fault_latched != 0U) &&
           (status.bus_error_count == 1U));
    return 0;
}
