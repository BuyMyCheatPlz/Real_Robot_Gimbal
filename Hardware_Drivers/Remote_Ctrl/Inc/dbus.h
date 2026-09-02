#ifndef DBUS_H
#define DBUS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define DBUS_CHANNEL_COUNT 18U
#define DBUS_FRAME_LENGTH  18U

typedef struct
{
    uint16_t channel[DBUS_CHANNEL_COUNT];
    uint8_t frame_lost;
    uint8_t failsafe;
    uint8_t online;
    uint32_t last_update_ms;
} DBusData_t;

typedef DBusData_t SBusData_t;

HAL_StatusTypeDef DBus_Init(UART_HandleTypeDef *huart);
uint8_t DBus_GetData(DBusData_t *data);
float DBus_ChannelNormalized(uint8_t channel);
void DBus_CheckOffline(uint32_t now_ms, uint32_t timeout_ms);

#define SBus_Init DBus_Init
#define SBus_GetData DBus_GetData
#define SBus_ChannelNormalized DBus_ChannelNormalized
#define SBus_CheckOffline DBus_CheckOffline
#define SBUS_CHANNEL_COUNT DBUS_CHANNEL_COUNT
#define SBUS_FRAME_LENGTH DBUS_FRAME_LENGTH

#endif
