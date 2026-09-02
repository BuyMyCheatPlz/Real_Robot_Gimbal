#ifndef BMI088_H
#define BMI088_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *acc_cs_port;
    uint16_t acc_cs_pin;
    GPIO_TypeDef *gyro_cs_port;
    uint16_t gyro_cs_pin;
    float acceleration_m_s2[3];
    float angular_rate_rad_s[3];
    float temperature_c;
    uint8_t initialized;
    volatile uint8_t dma_busy;
    volatile uint8_t dma_data_ready;
    volatile HAL_StatusTypeDef dma_status;
    volatile uint8_t dma_recovery_required;
    volatile uint32_t dma_started_ms;
    volatile uint32_t dma_recovery_count;
    volatile uint32_t last_update_ms;
    volatile uint32_t sample_count;
} BMI088_t;

HAL_StatusTypeDef BMI088_Init(BMI088_t *imu, SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *acc_cs_port, uint16_t acc_cs_pin,
                              GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin);
HAL_StatusTypeDef BMI088_Read(BMI088_t *imu);
HAL_StatusTypeDef BMI088_ReadAcceleration(BMI088_t *imu);
HAL_StatusTypeDef BMI088_ReadGyroscope(BMI088_t *imu);
HAL_StatusTypeDef BMI088_ReadTemperature(BMI088_t *imu);
HAL_StatusTypeDef BMI088_StartReadDMA(BMI088_t *imu);
HAL_StatusTypeDef BMI088_ServiceDMA(BMI088_t *imu, uint32_t now_ms);
uint8_t BMI088_DMADataReady(const BMI088_t *imu);

#endif
