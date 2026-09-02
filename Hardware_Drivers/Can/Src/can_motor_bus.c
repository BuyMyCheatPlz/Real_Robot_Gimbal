#include "can_motor_bus.h"
#include <string.h>

M3508_t can1_m3508_id2;
M3508_t can1_m3508_id3;
GM6020_t can1_gm6020_id2;
M2006_t can2_m2006_id5;
DM4310_t can2_dm4310_id1;

static CAN_HandleTypeDef *bus_can1;
static CAN_HandleTypeDef *bus_can2;
static volatile CanMotorBusStatus_t tx_status;
static uint8_t tx_fault_prepared;

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

static HAL_StatusTypeDef send_commands(int16_t m3508_id2,
                                       int16_t m3508_id3,
                                       int16_t gm6020_id2,
                                       int16_t m2006_id5,
                                       int16_t dm4310_id1)
{
    uint8_t can1_c620[8] = {0};
    uint8_t can1_gm[8] = {0};
    uint8_t can2_c610[8] = {0};
    uint8_t can2_dm[8] = {0};
    uint16_t dm_control_id;
    HAL_StatusTypeDef status = HAL_OK;

    if ((bus_can1 == 0) || (bus_can2 == 0)) return HAL_ERROR;
    pack_slot(can1_c620, 1U, m3508_id2);
    pack_slot(can1_c620, 2U, m3508_id3);
    pack_slot(can1_gm, 1U, gm6020_id2);
    pack_slot(can2_c610, 0U, m2006_id5);
    if (can2_dm4310_id1.online != 0U)
    {
        if (DM4310_PackCurrentCommand(&can2_dm4310_id1, dm4310_id1,
                                      &dm_control_id, can2_dm) == 0U)
            return HAL_ERROR;
    }

    if (send_std(bus_can1, 0x200U, can1_c620) != HAL_OK) status = HAL_ERROR;
    if (send_std(bus_can1, 0x1FFU, can1_gm) != HAL_OK) status = HAL_ERROR;
    if (can2_m2006_id5.feedback.online != 0U)
    {
        if (send_std(bus_can2, 0x1FFU, can2_c610) != HAL_OK)
            status = HAL_ERROR;
    }
    if (can2_dm4310_id1.online != 0U)
    {
        if (send_std(bus_can2, dm_control_id, can2_dm) != HAL_OK)
            status = HAL_ERROR;
    }
    return status;
}

static void reset_gimbal_control(void)
{
    GM6020_SetVoltageFeedforward(&can1_gm6020_id2, 0.0f);
    GM6020_SetSpeed(&can1_gm6020_id2, 0.0f);
    DM4310_SetSpeed(&can2_dm4310_id1, 0.0f);
    DM4310_SetCurrentFeedforward(&can2_dm4310_id1, 0);
    MotorSpeedPid_Reset(&can1_gm6020_id2.speed_pid);
    MotorSpeedPid_Reset(&can2_dm4310_id1.speed_pid);
    can1_gm6020_id2.filtered_speed_rpm =
        (float)can1_gm6020_id2.feedback.speed_rpm;
    can2_dm4310_id1.filtered_speed_rpm = 0.0f;
    can2_dm4310_id1.speed_filter_initialized = 0U;
}

static void reset_all_control(void)
{
    reset_gimbal_control();
    M3508_SetSpeed(&can1_m3508_id2, 0.0f);
    M3508_SetSpeed(&can1_m3508_id3, 0.0f);
    M2006_SetSpeed(&can2_m2006_id5, 0.0f);
    MotorSpeedPid_Reset(&can1_m3508_id2.speed_pid);
    MotorSpeedPid_Reset(&can1_m3508_id3.speed_pid);
    MotorSpeedPid_Reset(&can2_m2006_id5.speed_pid);
    can1_m3508_id2.filtered_speed_rpm =
        (float)can1_m3508_id2.feedback.speed_rpm;
    can1_m3508_id3.filtered_speed_rpm =
        (float)can1_m3508_id3.feedback.speed_rpm;
    can2_m2006_id5.filtered_speed_rpm =
        (float)can2_m2006_id5.feedback.speed_rpm;
}

static void record_tx_result(HAL_StatusTypeDef status, uint8_t recovery_frame)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (status == HAL_OK)
    {
        tx_status.consecutive_tx_failures = 0U;
        if ((recovery_frame != 0U) && (tx_status.fault_latched != 0U))
        {
            if (tx_status.recovery_zero_frames < UINT16_MAX)
                ++tx_status.recovery_zero_frames;
            if (tx_status.recovery_zero_frames >=
                CAN_MOTOR_TX_RECOVERY_FRAME_COUNT)
            {
                tx_status.fault_latched = 0U;
                tx_status.recovery_zero_frames = 0U;
                tx_fault_prepared = 0U;
            }
        }
    }
    else
    {
        ++tx_status.total_tx_failures;
        tx_status.recovery_zero_frames = 0U;
        if (tx_status.consecutive_tx_failures < UINT16_MAX)
            ++tx_status.consecutive_tx_failures;
        if (tx_status.consecutive_tx_failures >=
            CAN_MOTOR_TX_FAILURE_LATCH_COUNT)
            tx_status.fault_latched = 1U;
    }
    if (primask == 0U) __enable_irq();
}

static void prepare_tx_fault(void)
{
    if (tx_fault_prepared != 0U) return;
    reset_all_control();
    if (bus_can1 != 0)
    {
        (void)HAL_CAN_AbortTxRequest(bus_can1, CAN_TX_MAILBOX0 |
                                    CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        (void)HAL_CAN_ResetError(bus_can1);
    }
    if (bus_can2 != 0)
    {
        (void)HAL_CAN_AbortTxRequest(bus_can2, CAN_TX_MAILBOX0 |
                                    CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        (void)HAL_CAN_ResetError(bus_can2);
    }
    tx_fault_prepared = 1U;
}

static HAL_StatusTypeDef guarded_send_commands(int16_t m3508_id2,
                                                int16_t m3508_id3,
                                                int16_t gm6020_id2,
                                                int16_t m2006_id5,
                                                int16_t dm4310_id1)
{
    HAL_StatusTypeDef status;
    if (tx_status.fault_latched != 0U)
    {
        prepare_tx_fault();
        status = send_commands(0, 0, 0, 0, 0);
        record_tx_result(status, 1U);
        return HAL_ERROR;
    }
    status = send_commands(m3508_id2, m3508_id3, gm6020_id2,
                           m2006_id5, dm4310_id1);
    record_tx_result(status, 0U);
    return status;
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
    memset((void *)&tx_status, 0, sizeof(tx_status));
    tx_fault_prepared = 0U;

    /* 驱动层增益保持为零，任务启动后再加载 config.h 中的实车参数。 */
    M3508_Init(&can1_m3508_id2, 2U, 0.0f, 0.0f);
    M3508_Init(&can1_m3508_id3, 3U, 0.0f, 0.0f);
    GM6020_Init(&can1_gm6020_id2, 2U, 0.0f, 0.0f);
    M2006_Init(&can2_m2006_id5, 5U, 0.0f, 0.0f);
    DM4310_Init(&can2_dm4310_id1, 1U, 0.0f, 0.0f);

    if (configure_filter(can1, 0U, 0x202U, 0x203U, 0x206U, 0x206U) != HAL_OK)
        return HAL_ERROR;
    if (configure_filter(can2, 14U, 0x205U,
                         DM4310_FEEDBACK_BASE_ID + can2_dm4310_id1.id,
                         0x205U,
                         DM4310_FEEDBACK_BASE_ID + can2_dm4310_id1.id) != HAL_OK)
        return HAL_ERROR;
    if (HAL_CAN_Start(can1) != HAL_OK) return HAL_ERROR;
    if (HAL_CAN_Start(can2) != HAL_OK) return HAL_ERROR;
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
    if (HAL_CAN_ActivateNotification(can1, CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_ERROR_WARNING |
                                     CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
                                     CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR) != HAL_OK)
        return HAL_ERROR;
    if (HAL_CAN_ActivateNotification(can2, CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_ERROR_WARNING |
                                     CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
                                     CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR) != HAL_OK)
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
    int16_t m3508_id2;
    int16_t m3508_id3;
    int16_t gm6020_id2;
    int16_t m2006_id5;
    int16_t dm4310_id1;

    if ((bus_can1 == 0) || (bus_can2 == 0) || (dt_s <= 0.0f)) return HAL_ERROR;
    CanMotorBus_CheckOffline(HAL_GetTick());

    m3508_id2 = M3508_Update(&can1_m3508_id2, dt_s);
    m3508_id3 = M3508_Update(&can1_m3508_id3, dt_s);
    gm6020_id2 = GM6020_Update(&can1_gm6020_id2, dt_s);
    m2006_id5 = M2006_Update(&can2_m2006_id5, dt_s);
    dm4310_id1 = DM4310_Update(&can2_dm4310_id1, dt_s);
    return guarded_send_commands(m3508_id2, m3508_id3, gm6020_id2,
                                 m2006_id5, dm4310_id1);
}

HAL_StatusTypeDef CanMotorBus_StopGimbal(float dt_s)
{
    int16_t m3508_id2;
    int16_t m3508_id3;
    int16_t m2006_id5;
    if (dt_s <= 0.0f) return HAL_ERROR;
    CanMotorBus_CheckOffline(HAL_GetTick());
    if (CanMotorBus_TxHealthy() == 0U) return CanMotorBus_StopAll();
    reset_gimbal_control();
    m3508_id2 = M3508_Update(&can1_m3508_id2, dt_s);
    m3508_id3 = M3508_Update(&can1_m3508_id3, dt_s);
    m2006_id5 = M2006_Update(&can2_m2006_id5, dt_s);
    return guarded_send_commands(m3508_id2, m3508_id3, 0,
                                 m2006_id5, 0);
}

HAL_StatusTypeDef CanMotorBus_StopAll(void)
{
    reset_all_control();
    return guarded_send_commands(0, 0, 0, 0, 0);
}

uint8_t CanMotorBus_TxHealthy(void)
{
    CanMotorBusStatus_t status;
    CanMotorBus_GetStatus(&status);
    return (uint8_t)((status.fault_latched == 0U) &&
                     (status.consecutive_tx_failures == 0U));
}

void CanMotorBus_GetStatus(CanMotorBusStatus_t *status)
{
    uint32_t primask;
    if (status == 0) return;
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(status, (const void *)&tx_status, sizeof(*status));
    if (primask == 0U) __enable_irq();
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
            else if ((header.StdId ==
                      (DM4310_FEEDBACK_BASE_ID + can2_dm4310_id1.id)) &&
                     (header.DLC == 8U))
                DM4310_Decode(&can2_dm4310_id1, data, now_ms);
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error;
    uint32_t primask;
    if ((hcan != bus_can1) && (hcan != bus_can2)) return;
    error = HAL_CAN_GetError(hcan);
    primask = __get_PRIMASK();
    __disable_irq();
    ++tx_status.bus_error_count;
    ++tx_status.total_tx_failures;
    tx_status.recovery_zero_frames = 0U;
    if (tx_status.consecutive_tx_failures < UINT16_MAX)
        ++tx_status.consecutive_tx_failures;
    if (((error & (HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV)) != 0U) ||
        (tx_status.consecutive_tx_failures >=
         CAN_MOTOR_TX_FAILURE_LATCH_COUNT))
        tx_status.fault_latched = 1U;
    if (primask == 0U) __enable_irq();
}
