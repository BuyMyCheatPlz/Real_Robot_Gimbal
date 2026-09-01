#include "can_motor_bus.h"
#include <string.h>

M3508_t can1_m3508_id2;
M3508_t can1_m3508_id3;
GM6020_t can1_gm6020_id2;
M2006_t can2_m2006_id5;
DM4310_t can2_dm4310_id1;

static CAN_HandleTypeDef *bus_can1;
static CAN_HandleTypeDef *bus_can2;

static HAL_StatusTypeDef configure_filter(CAN_HandleTypeDef *hcan,
                                          uint32_t filter_bank,
                                          uint16_t id1, uint16_t id2,
                                          uint16_t id3, uint16_t id4)
{
    CAN_FilterTypeDef filter;
    memset(&filter, 0, sizeof(filter));
    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = (uint32_t)id1 << 5;
    filter.FilterIdLow = (uint32_t)id2 << 5;
    filter.FilterMaskIdHigh = (uint32_t)id3 << 5;
    filter.FilterMaskIdLow = (uint32_t)id4 << 5;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    return HAL_CAN_ConfigFilter(hcan, &filter);
}

static HAL_StatusTypeDef send_std(CAN_HandleTypeDef *hcan, uint16_t id,
                                  uint8_t data[8])
{
    CAN_TxHeaderTypeDef header;
    uint32_t mailbox;
    header.StdId = id;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

static void pack_slot(uint8_t data[8], uint8_t slot, int16_t command)
{
    if (slot >= 4U) return;
    data[slot * 2U] = (uint8_t)((uint16_t)command >> 8);
    data[slot * 2U + 1U] = (uint8_t)command;
}

static void update_online(DjiMotorFeedback_t *feedback, uint32_t now_ms)
{
    if ((feedback->online != 0U) &&
        ((now_ms - feedback->last_update_ms) > CAN_MOTOR_OFFLINE_TIMEOUT_MS))
    {
        feedback->online = 0U;
    }
}

HAL_StatusTypeDef CanMotorBus_Init(CAN_HandleTypeDef *can1,
                                   CAN_HandleTypeDef *can2)
{
    if ((can1 == 0) || (can2 == 0)) return HAL_ERROR;
    bus_can1 = can1;
    bus_can2 = can2;

    /* 驱动层增益保持为零，任务启动后再加载 config.h 中的实车参数。 */
    M3508_Init(&can1_m3508_id2, 2U, 0.0f, 0.0f);
    M3508_Init(&can1_m3508_id3, 3U, 0.0f, 0.0f);
    GM6020_Init(&can1_gm6020_id2, 2U, 0.0f, 0.0f);
    M2006_Init(&can2_m2006_id5, 5U, 0.0f, 0.0f);
    DM4310_Init(&can2_dm4310_id1, can2, 1U, 0x11U);

    if (configure_filter(can1, 0U, 0x202U, 0x203U, 0x206U, 0x206U) != HAL_OK)
        return HAL_ERROR;
    if (configure_filter(can2, 14U, 0x205U, 0x011U, 0x205U, 0x011U) != HAL_OK)
        return HAL_ERROR;
    if (HAL_CAN_Start(can1) != HAL_OK) return HAL_ERROR;
    if (HAL_CAN_Start(can2) != HAL_OK) return HAL_ERROR;
    if (HAL_CAN_ActivateNotification(can1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        return HAL_ERROR;
    if (HAL_CAN_ActivateNotification(can2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}

void CanMotorBus_CheckOffline(uint32_t now_ms)
{
    update_online(&can1_m3508_id2.feedback, now_ms);
    update_online(&can1_m3508_id3.feedback, now_ms);
    update_online(&can1_gm6020_id2.feedback, now_ms);
    update_online(&can2_m2006_id5.feedback, now_ms);
    if ((can2_dm4310_id1.online != 0U) &&
        ((now_ms - can2_dm4310_id1.last_update_ms) > CAN_MOTOR_OFFLINE_TIMEOUT_MS))
        can2_dm4310_id1.online = 0U;
}

HAL_StatusTypeDef CanMotorBus_Update(float dt_s)
{
    uint8_t can1_c620[8] = {0};
    uint8_t can1_gm[8] = {0};
    uint8_t can2_c610[8] = {0};
    HAL_StatusTypeDef status = HAL_OK;

    if ((bus_can1 == 0) || (bus_can2 == 0) || (dt_s <= 0.0f)) return HAL_ERROR;
    CanMotorBus_CheckOffline(HAL_GetTick());

    pack_slot(can1_c620, 1U, M3508_Update(&can1_m3508_id2, dt_s));
    pack_slot(can1_c620, 2U, M3508_Update(&can1_m3508_id3, dt_s));
    pack_slot(can1_gm, 1U, GM6020_Update(&can1_gm6020_id2, dt_s));
    pack_slot(can2_c610, 0U, M2006_Update(&can2_m2006_id5, dt_s));

    if (send_std(bus_can1, 0x200U, can1_c620) != HAL_OK) status = HAL_ERROR;
    if (send_std(bus_can1, 0x1FFU, can1_gm) != HAL_OK) status = HAL_ERROR;
    if (send_std(bus_can2, 0x1FFU, can2_c610) != HAL_OK) status = HAL_ERROR;
    if ((can2_dm4310_id1.enabled != 0U) &&
        (DM4310_SetVelocity(&can2_dm4310_id1,
                            can2_dm4310_id1.target_velocity_rad_s) != HAL_OK))
        status = HAL_ERROR;
    return status;
}

HAL_StatusTypeDef CanMotorBus_StopAll(void)
{
    M3508_SetSpeed(&can1_m3508_id2, 0.0f);
    M3508_SetSpeed(&can1_m3508_id3, 0.0f);
    GM6020_SetSpeed(&can1_gm6020_id2, 0.0f);
    M2006_SetSpeed(&can2_m2006_id5, 0.0f);
    MotorSpeedPid_Reset(&can1_m3508_id2.speed_pid);
    MotorSpeedPid_Reset(&can1_m3508_id3.speed_pid);
    MotorSpeedPid_Reset(&can1_gm6020_id2.speed_pid);
    MotorSpeedPid_Reset(&can2_m2006_id5.speed_pid);
    if (DM4310_SetVelocity(&can2_dm4310_id1, 0.0f) != HAL_OK) return HAL_ERROR;
    return CanMotorBus_Update(0.001f);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t now_ms = HAL_GetTick();

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
            break;
        if ((header.IDE != CAN_ID_STD) || (header.RTR != CAN_RTR_DATA))
            continue;

        if (hcan == bus_can1)
        {
            if ((header.StdId == 0x202U) && (header.DLC == 8U))
                M3508_Decode(&can1_m3508_id2, data, now_ms);
            else if ((header.StdId == 0x203U) && (header.DLC == 8U))
                M3508_Decode(&can1_m3508_id3, data, now_ms);
            else if ((header.StdId == 0x206U) && (header.DLC == 8U))
                GM6020_Decode(&can1_gm6020_id2, data, now_ms);
        }
        else if (hcan == bus_can2)
        {
            if ((header.StdId == 0x205U) && (header.DLC == 8U))
                M2006_Decode(&can2_m2006_id5, data, now_ms);
            else if ((header.StdId == can2_dm4310_id1.master_id) &&
                     (header.DLC == 8U))
                DM4310_Decode(&can2_dm4310_id1, data, now_ms);
        }
    }
}
