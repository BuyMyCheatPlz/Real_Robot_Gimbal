#include "hardware_drivers.h"
#include "can.h"
#include "usart.h"
#include "can_motor_bus.h"
#include "sbus.h"
#include "spi.h"
#include "main.h"

BMI088_t bmi088;
volatile uint32_t bmi088_gyro_drdy_count;
volatile uint32_t bmi088_accel_drdy_count;

HAL_StatusTypeDef HardwareDrivers_Init(void)
{
    if (CanMotorBus_Init(&hcan1, &hcan2) != HAL_OK) return HAL_ERROR;
    if (SBus_Init(&huart2) != HAL_OK) return HAL_ERROR;
    if (BMI088_Init(&bmi088, &hspi1,
                    CS_Accel_GPIO_Port, CS_Accel_Pin,
                    CS_Gyro_GPIO_Port, CS_Gyro_Pin) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (gpio_pin == INT_Gyro_Pin)
    {
        ++bmi088_gyro_drdy_count;
        (void)BMI088_StartReadDMA(&bmi088);
    }
    else if (gpio_pin == INT_Accel_Pin)
    {
        ++bmi088_accel_drdy_count;
    }
}
