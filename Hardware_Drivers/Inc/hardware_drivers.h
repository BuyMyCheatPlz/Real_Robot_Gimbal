#ifndef HARDWARE_DRIVERS_H
#define HARDWARE_DRIVERS_H

#include "stm32f4xx_hal.h"
#include "bmi088.h"

extern BMI088_t bmi088;
extern volatile uint32_t bmi088_gyro_drdy_count;
extern volatile uint32_t bmi088_accel_drdy_count;

HAL_StatusTypeDef HardwareDrivers_Init(void);

#endif
