#ifndef CAN_MOTOR_BUS_H
#define CAN_MOTOR_BUS_H

#include "stm32f4xx_hal.h"
#include "m3508.h"
#include "m2006.h"
#include "gm6020.h"
#include "dm4310.h"

/* 反馈离线超时：DJI 电机反馈约 1kHz(1ms/帧)。收紧到 10ms，让单个电机反馈一断
 * 就自己判离线→速度环复位断输出，不再拿着旧转速疯转，也不拖累其它电机。
 * (每个电机独立判定，DM4310 有优先发送/独立反馈所以最稳。) */
#define CAN_MOTOR_OFFLINE_TIMEOUT_MS 10U
#define CAN_MOTOR_TX_FAILURE_LATCH_COUNT 3U
#define CAN_MOTOR_TX_RECOVERY_FRAME_COUNT 10U

typedef struct
{
    uint32_t total_tx_failures;
    uint32_t bus_error_count;
    uint32_t dm4310_feedback_count;
    uint32_t can1_tx_complete_count;
    uint32_t can2_tx_complete_count;
    uint32_t can1_tx_busy_count;
    uint32_t can2_tx_busy_count;
    uint32_t can1_rx_count;
    uint32_t can2_rx_count;
    uint32_t can1_last_error;
    uint32_t can2_last_error;
    uint32_t can1_busoff_count;
    uint32_t can2_busoff_count;
    uint32_t can1_recovery_count;
    uint32_t can2_recovery_count;
    uint16_t last_can2_rx_std_id;
    uint16_t consecutive_tx_failures;
    uint16_t recovery_zero_frames;
    int16_t last_gm6020_id2_command;
    int16_t last_dm4310_id1_command;
    uint8_t last_send_failure_mask;
    uint8_t last_send_busy_mask;
    uint8_t can1_tx_free_level;
    uint8_t can2_tx_free_level;
    uint8_t fault_latched;
} CanMotorBusStatus_t;

extern M3508_t can1_m3508_id2;
extern M3508_t can1_m3508_id3;
extern GM6020_t can1_gm6020_id2;
extern M2006_t can2_m2006_id5;
extern DM4310_t can2_dm4310_id1;

HAL_StatusTypeDef CanMotorBus_Init(CAN_HandleTypeDef *can1,
                                   CAN_HandleTypeDef *can2);
HAL_StatusTypeDef CanMotorBus_Update(float dt_s);
HAL_StatusTypeDef CanMotorBus_UpdateSelected(float dt_s,
                                             uint8_t pitch_enabled,
                                             uint8_t yaw_enabled);
HAL_StatusTypeDef CanMotorBus_SendYawTestCurrent(int16_t current);
HAL_StatusTypeDef CanMotorBus_StopGimbal(float dt_s);
HAL_StatusTypeDef CanMotorBus_StopAll(void);
void CanMotorBus_CheckOffline(uint32_t now_ms);
uint8_t CanMotorBus_TxHealthy(void);
void CanMotorBus_GetStatus(CanMotorBusStatus_t *status);

#endif
