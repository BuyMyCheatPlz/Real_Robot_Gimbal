#include "can_motor_bus.h"
#include "config.h"
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
static uint32_t last_command_send_ms;
static uint8_t command_send_initialized;
static volatile uint8_t can1_recovery_pending;
static volatile uint8_t can2_recovery_pending;
static volatile uint8_t can_service_busy;
static volatile uint32_t can1_busy_since_ms;
static volatile uint32_t can2_busy_since_ms;

#define GM6020_FEEDBACK_BASE_ID 0x204U

static void record_gimbal_commands(int16_t gm6020_id2, int16_t dm4310_id1)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    tx_status.last_gm6020_id2_command = gm6020_id2;
    tx_status.last_dm4310_id1_command = dm4310_id1;
    if (primask == 0U) __enable_irq();
}

static void record_tx_diagnostics(uint8_t failure_mask, uint8_t busy_mask)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    tx_status.last_send_failure_mask = failure_mask;
    tx_status.last_send_busy_mask = busy_mask;
    if ((busy_mask & 0x03U) != 0U) ++tx_status.can1_tx_busy_count;
    if ((busy_mask & 0x0CU) != 0U) ++tx_status.can2_tx_busy_count;
    tx_status.can1_tx_free_level =
        (uint8_t)HAL_CAN_GetTxMailboxesFreeLevel(bus_can1);
    tx_status.can2_tx_free_level =
        (uint8_t)HAL_CAN_GetTxMailboxesFreeLevel(bus_can2);
    if (primask == 0U) __enable_irq();
}

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
    /* 邮箱已满属于发送反压；调用方会在下一次固定发布周期提交最新命令。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) return HAL_BUSY;
    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

static uint8_t tx_capacity_available(CAN_HandleTypeDef *hcan,
                                     uint8_t required_mailboxes)
{
    return (uint8_t)(HAL_CAN_GetTxMailboxesFreeLevel(hcan) >=
                     required_mailboxes);
}

/* 若一条总线连续多拍因"邮箱已满/帧未完成"被跳过，说明有发送请求卡死在邮箱里
 * (典型的未 ACK 帧)。此时必须中止该总线全部邮箱把它们释放，否则 capacity 判断
 * 会永久跳过这条总线：CAN1 因需要 2 个空邮箱而整条停发，CAN2 里 M2006(需 2 个)
 * 被饿死、只剩 DM4310(只需 1 个)存活。 */
static void recover_stuck_tx_mailboxes(CAN_HandleTypeDef *hcan,
                                       volatile uint32_t *busy_since_ms,
                                       uint8_t busy_now)
{
    uint32_t now;
    if ((hcan == 0) || (busy_since_ms == 0)) return;
    now = HAL_GetTick();
    if (busy_now == 0U)
    {
        *busy_since_ms = 0U;
        return;
    }
    if (*busy_since_ms == 0U)
    {
        *busy_since_ms = now;
        return;
    }
    if ((now - *busy_since_ms) >= CAN_TX_STUCK_ABORT_MS)
    {
        (void)HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0 |
                                     CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        *busy_since_ms = now;
    }
}

static void accumulate_send_result(HAL_StatusTypeDef result, uint8_t bit,
                                   uint8_t *failure_mask, uint8_t *busy_mask,
                                   HAL_StatusTypeDef *overall_status)
{
    if (result == HAL_BUSY)
    {
        *busy_mask |= bit;
    }
    else if (result != HAL_OK)
    {
        *failure_mask |= bit;
        *overall_status = HAL_ERROR;
    }
}

static void pack_slot(uint8_t data[8], uint8_t slot, int16_t command)
{
    if (slot >= 4U) return;
    data[slot * 2U] = (uint8_t)((uint16_t)command >> 8);
    data[slot * 2U + 1U] = (uint8_t)command;
}

static uint16_t pack_gm6020_command(uint8_t data[8], uint8_t motor_id,
                                     int16_t command)
{
    if ((motor_id >= 1U) && (motor_id <= 4U))
    {
        pack_slot(data, motor_id - 1U, command);
        return 0x1FFU;
    }
    if ((motor_id >= 5U) && (motor_id <= 8U))
    {
        pack_slot(data, motor_id - 5U, command);
        return 0x2FFU;
    }
    return 0U;
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
    uint16_t gm_control_id;
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t failure_mask = 0U;
    uint8_t busy_mask = 0U;
    HAL_StatusTypeDef result;

    if ((bus_can1 == 0) || (bus_can2 == 0))
    {
        record_gimbal_commands(0, 0);
        return HAL_ERROR;
    }
    if (YAW_COMMISSIONING_MODE == 0U)
    {
        if (LAUNCH_MOTOR_OUTPUT_ENABLE != 0U)
        {
            pack_slot(can1_c620, 1U, m3508_id2);
            pack_slot(can1_c620, 2U, m3508_id3);
            pack_slot(can2_c610, 0U, m2006_id5);
        }
        gm_control_id = pack_gm6020_command(can1_gm,
                                             can1_gm6020_id2.id,
                                             gm6020_id2);
        if (gm_control_id == 0U)
        {
            record_gimbal_commands(0, 0);
            return HAL_ERROR;
        }
    }

    /* 先打包 DM4310(离线也发 0 电流，保持 0x3FE 反馈不中断) */
    if (DM4310_PackCurrentCommand(&can2_dm4310_id1,
                                  (can2_dm4310_id1.online != 0U) ?
                                  dm4310_id1 : 0,
                                  &dm_control_id, can2_dm) == 0U)
    {
        record_gimbal_commands(gm6020_id2, 0);
        return HAL_ERROR;
    }
    record_gimbal_commands((YAW_COMMISSIONING_MODE != 0U) ? 0 : gm6020_id2,
                           (can2_dm4310_id1.online != 0U) ? dm4310_id1 : 0);

    /* ===== 逐帧独立发送：每帧只需 1 个空邮箱。任一帧被卡/未 ACK 都只影响它自己，
     * 不会因为"需要 2 个空邮箱"的聚合判断把整条总线饿死。 ===== */
    if (YAW_COMMISSIONING_MODE == 0U)
    {
        if (tx_capacity_available(bus_can1, 1U) != 0U)
        {
            result = send_std(bus_can1, 0x200U, can1_c620);
            accumulate_send_result(result, 1U, &failure_mask, &busy_mask, &status);
        }
        else
            busy_mask |= 0x01U;
        if (tx_capacity_available(bus_can1, 1U) != 0U)
        {
            result = send_std(bus_can1, gm_control_id, can1_gm);
            accumulate_send_result(result, 2U, &failure_mask, &busy_mask, &status);
        }
        else
            busy_mask |= 0x02U;
    }

    /* CAN2：DM4310 优先发；M2006 在 DM 之后还有空位再发(离线也发 0 电流唤醒) */
    if (tx_capacity_available(bus_can2, 1U) != 0U)
    {
        result = send_std(bus_can2, dm_control_id, can2_dm);
        accumulate_send_result(result, 8U, &failure_mask, &busy_mask, &status);
    }
    else
        busy_mask |= 0x08U;
    if (YAW_COMMISSIONING_MODE == 0U)
    {
        if (tx_capacity_available(bus_can2, 1U) != 0U)
        {
            result = send_std(bus_can2, 0x1FFU, can2_c610);
            accumulate_send_result(result, 4U, &failure_mask, &busy_mask, &status);
        }
        else
            busy_mask |= 0x04U;
    }

    /* 卡死邮箱自动释放：连续被 busy 跳过则中止该总线邮箱，防止永久失联 */
    recover_stuck_tx_mailboxes(bus_can1, &can1_busy_since_ms,
                               (uint8_t)((busy_mask & 0x03U) != 0U));
    recover_stuck_tx_mailboxes(bus_can2, &can2_busy_since_ms,
                               (uint8_t)((busy_mask & 0x0CU) != 0U));
    record_tx_diagnostics(failure_mask, busy_mask);
    return status;
}

static void reset_pitch_control(void)
{
    GM6020_SetVoltageFeedforward(&can1_gm6020_id2, 0.0f);
    GM6020_SetSpeed(&can1_gm6020_id2, 0.0f);
    MotorSpeedPid_Reset(&can1_gm6020_id2.speed_pid);
    can1_gm6020_id2.filtered_speed_rpm =
        (float)can1_gm6020_id2.feedback.speed_rpm;
    /* 下次授权会切换到 BMI088 Roll 速度反馈；强制首帧重新初始化，
     * 避免将旧的编码器 rpm 与新的 °/s 混合。 */
    can1_gm6020_id2.speed_filter_initialized = 0U;
    can1_gm6020_id2.use_external_speed_feedback = 0U;  /* 复位后回落到编码器反馈 */
    can1_gm6020_id2.external_speed_rpm = 0.0f;
    can1_gm6020_id2.output_hold = 0U;                  /* 复位解除 yaw 运动期锁存 */
}

static void reset_yaw_control(void)
{
    DM4310_SetSpeed(&can2_dm4310_id1, 0.0f);
    DM4310_SetCurrentFeedforward(&can2_dm4310_id1, 0);
    can2_dm4310_id1.direct_current_en = 0U;   /* 退出直通(辨识)模式 */
    can2_dm4310_id1.direct_current = 0.0f;
    MotorSpeedPid_Reset(&can2_dm4310_id1.speed_pid);
    can2_dm4310_id1.current_quantization_error = 0.0f;
    can2_dm4310_id1.filtered_speed_rpm = 0.0f;
    can2_dm4310_id1.speed_filter_initialized = 0U;
}

static void reset_gimbal_control(void)
{
    reset_pitch_control();
    reset_yaw_control();
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
    /* 此处不要中止邮箱。中止 CAN2 可能取消有效电流命令，并在每个恢复周期用零电流
     * 帧替换它。CAN 初始化已启用 AutoBusOff，真正的总线恢复由外设处理，控制器会
     * 持续请求零输出。 */
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

static uint8_t command_send_due(uint32_t now_ms)
{
    if (command_send_initialized == 0U)
    {
        command_send_initialized = 1U;
        last_command_send_ms = now_ms;
        return 1U;
    }
    if ((now_ms - last_command_send_ms) < CAN_COMMAND_PERIOD_MS)
        return 0U;
    last_command_send_ms = now_ms;
    return 1U;
}

static void update_online(DjiMotorFeedback_t *feedback, uint32_t now_ms)
{
    if ((feedback->online != 0U) &&
        ((now_ms - feedback->last_update_ms) > CAN_MOTOR_OFFLINE_TIMEOUT_MS))
    {
        feedback->online = 0U;
    }
}

static void recover_can_if_needed(CAN_HandleTypeDef *hcan,
                                  volatile uint8_t *pending,
                                  volatile uint32_t *recovery_count)
{
    if (*pending == 0U) return;
    *pending = 0U;
    /* ABOM 已由硬件自动恢复总线：bus-off 后经过 128×11 个隐性位，硬件自动把
     * TEC/REC 清零并回到 error-active，无需软件干预。这里的 GetError/ResetError
     * 只清 HAL 软件层的 ErrorCode 标志（HAL_CAN_ResetError 不触碰硬件 TEC/REC、
     * 不进初始化模式、也不会截断在发的帧），避免旧错误标志残留影响后续判断。 */
    (void)HAL_CAN_GetError(hcan);
    (void)HAL_CAN_ResetError(hcan);
    if (recovery_count != 0U)
        ++(*recovery_count);
}

static void process_rx_fifo(CAN_HandleTypeDef *hcan, uint32_t now_ms)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
            break;
        if ((header.IDE != CAN_ID_STD) || (header.RTR != CAN_RTR_DATA) ||
            (header.DLC != 8U))
            continue;

        if (hcan == bus_can1)
            ++tx_status.can1_rx_count;
        else if (hcan == bus_can2)
            ++tx_status.can2_rx_count;

        if (hcan == bus_can1)
        {
            if (header.StdId == 0x202U)
                M3508_Decode(&can1_m3508_id2, data, now_ms);
            else if (header.StdId == 0x203U)
                M3508_Decode(&can1_m3508_id3, data, now_ms);
            else if (header.StdId ==
                     (GM6020_FEEDBACK_BASE_ID + can1_gm6020_id2.id))
                GM6020_Decode(&can1_gm6020_id2, data, now_ms);
        }
        else if (hcan == bus_can2)
        {
            tx_status.last_can2_rx_std_id = (uint16_t)header.StdId;
            if (header.StdId == 0x205U)
                M2006_Decode(&can2_m2006_id5, data, now_ms);
            else if (header.StdId ==
                     (DM4310_FEEDBACK_BASE_ID + can2_dm4310_id1.id))
            {
                DM4310_Decode(&can2_dm4310_id1, data, now_ms);
                ++tx_status.dm4310_feedback_count;
            }
        }
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
    last_command_send_ms = 0U;
    command_send_initialized = 0U;
    can1_recovery_pending = 0U;
    can2_recovery_pending = 0U;
    can_service_busy = 0U;

    /* 驱动层增益保持为零，任务启动后再加载 config.h 中的实车参数。 */
    M3508_Init(&can1_m3508_id2, 2U, 0.0f, 0.0f);
    M3508_Init(&can1_m3508_id3, 3U, 0.0f, 0.0f);
    GM6020_Init(&can1_gm6020_id2, PITCH_GM6020_CAN_ID, 0.0f, 0.0f);
    M2006_Init(&can2_m2006_id5, 5U, 0.0f, 0.0f);
    DM4310_Init(&can2_dm4310_id1, 1U, 0.0f, 0.0f);

    if (configure_filter(can1, 0U, 0x202U, 0x203U,
                         GM6020_FEEDBACK_BASE_ID + can1_gm6020_id2.id,
                         GM6020_FEEDBACK_BASE_ID + can1_gm6020_id2.id) != HAL_OK)
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
    if (HAL_CAN_ActivateNotification(can1, CAN_IT_TX_MAILBOX_EMPTY |
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_ERROR_WARNING |
                                     CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
                                     CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR) != HAL_OK)
        return HAL_ERROR;
    if (HAL_CAN_ActivateNotification(can2, CAN_IT_TX_MAILBOX_EMPTY |
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_ERROR_WARNING |
                                     CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
                                     CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}

void CanMotorBus_CheckOffline(uint32_t now_ms)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    if (can_service_busy != 0U)
    {
        if (primask == 0U) __enable_irq();
        return;
    }
    can_service_busy = 1U;
    if (primask == 0U) __enable_irq();

    /* RX FIFO 只能由 RX0 中断消费。任务上下文绝不能再调 process_rx_fifo：
     * 否则 ISR 与任务并发读同一个硬件 FIFO，RFOM 二次释放会把整帧弹掉没人解码，
     * 表现为"总线反馈稳定却偶发判离线"。任务上下文只做离线检查与总线恢复。 */
    if (bus_can1 != 0)
    {
        recover_can_if_needed(bus_can1, &can1_recovery_pending,
                              &tx_status.can1_recovery_count);
    }
    if (bus_can2 != 0)
    {
        recover_can_if_needed(bus_can2, &can2_recovery_pending,
                              &tx_status.can2_recovery_count);
    }
    update_online(&can1_m3508_id2.feedback, now_ms);
    update_online(&can1_m3508_id3.feedback, now_ms);
    update_online(&can1_gm6020_id2.feedback, now_ms);
    update_online(&can2_m2006_id5.feedback, now_ms);
    if ((can2_dm4310_id1.online != 0U) &&
        ((now_ms - can2_dm4310_id1.last_update_ms) > CAN_MOTOR_OFFLINE_TIMEOUT_MS))
        can2_dm4310_id1.online = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    can_service_busy = 0U;
    if (primask == 0U) __enable_irq();
}

HAL_StatusTypeDef CanMotorBus_Update(float dt_s)
{
    return CanMotorBus_UpdateSelected(dt_s, 1U, 1U);
}

HAL_StatusTypeDef CanMotorBus_UpdateSelected(float dt_s,
                                             uint8_t pitch_enabled,
                                             uint8_t yaw_enabled)
{
    uint32_t now_ms;
    int16_t m3508_id2;
    int16_t m3508_id3;
    int16_t gm6020_id2;
    int16_t m2006_id5;
    int16_t dm4310_id1;

    if ((bus_can1 == 0) || (bus_can2 == 0) || (dt_s <= 0.0f)) return HAL_ERROR;
    now_ms = HAL_GetTick();
    CanMotorBus_CheckOffline(now_ms);

    m3508_id2 = M3508_Update(&can1_m3508_id2, dt_s);
    m3508_id3 = M3508_Update(&can1_m3508_id3, dt_s);
    if (pitch_enabled != 0U)
        gm6020_id2 = GM6020_Update(&can1_gm6020_id2, dt_s);
    else
    {
        reset_pitch_control();
        gm6020_id2 = 0;
    }
    m2006_id5 = M2006_Update(&can2_m2006_id5, dt_s);
    if (yaw_enabled != 0U)
        dm4310_id1 = DM4310_Update(&can2_dm4310_id1, dt_s);
    else
    {
        /* Yaw 启动阶段有意绕过零速 PID。未稳定的速度反馈即使目标为零，也可能经过
         * PID 产生较大的制动命令。 */
        reset_yaw_control();
        dm4310_id1 = 0;
    }
    /* Keep the PID loop fast, but publish the latest command at a fixed lower
     * rate so CAN mailbox pressure does not become control noise. */
    if (command_send_due(now_ms) == 0U) return HAL_OK;
    return guarded_send_commands(m3508_id2, m3508_id3, gm6020_id2,
                                 m2006_id5, dm4310_id1);
}

HAL_StatusTypeDef CanMotorBus_SendYawTestCurrent(int16_t current)
{
    CanMotorBus_CheckOffline(HAL_GetTick());
    if (can2_dm4310_id1.online == 0U) return HAL_ERROR;
    if (current > (int16_t)DM4310_CURRENT_COMMAND_LIMIT)
        current = (int16_t)DM4310_CURRENT_COMMAND_LIMIT;
    if (current < (int16_t)-DM4310_CURRENT_COMMAND_LIMIT)
        current = (int16_t)-DM4310_CURRENT_COMMAND_LIMIT;
    return guarded_send_commands(0, 0, 0, 0, current);
}

HAL_StatusTypeDef CanMotorBus_StopGimbal(float dt_s)
{
    uint32_t now_ms;
    int16_t m3508_id2;
    int16_t m3508_id3;
    int16_t m2006_id5;
    if (dt_s <= 0.0f) return HAL_ERROR;
    now_ms = HAL_GetTick();
    CanMotorBus_CheckOffline(now_ms);
    if (CanMotorBus_TxHealthy() == 0U) return CanMotorBus_StopAll();
    reset_gimbal_control();
    m3508_id2 = M3508_Update(&can1_m3508_id2, dt_s);
    m3508_id3 = M3508_Update(&can1_m3508_id3, dt_s);
    m2006_id5 = M2006_Update(&can2_m2006_id5, dt_s);
    if (command_send_due(now_ms) == 0U) return HAL_OK;
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

static void record_tx_complete(CAN_HandleTypeDef *hcan)
{
    uint32_t primask;
    if ((hcan != bus_can1) && (hcan != bus_can2)) return;
    primask = __get_PRIMASK();
    __disable_irq();
    if (hcan == bus_can1)
        ++tx_status.can1_tx_complete_count;
    else
        ++tx_status.can2_tx_complete_count;
    if (primask == 0U) __enable_irq();
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    record_tx_complete(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    record_tx_complete(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    record_tx_complete(hcan);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == bus_can1)
        process_rx_fifo(hcan, HAL_GetTick());
    else if (hcan == bus_can2)
        process_rx_fifo(hcan, HAL_GetTick());
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
    if (hcan == bus_can1)
    {
        tx_status.can1_last_error = error;
        if ((error & HAL_CAN_ERROR_BOF) != 0U)
        {
            ++tx_status.can1_busoff_count;
            can1_recovery_pending = 1U;
        }
    }
    else
    {
        tx_status.can2_last_error = error;
        if ((error & HAL_CAN_ERROR_BOF) != 0U)
        {
            ++tx_status.can2_busoff_count;
            can2_recovery_pending = 1U;
        }
    }
    if (primask == 0U) __enable_irq();
}
