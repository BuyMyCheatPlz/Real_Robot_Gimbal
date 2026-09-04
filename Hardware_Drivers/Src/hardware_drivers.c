#include "hardware_drivers.h"
#include "can.h"
#include "usart.h"
#include "can_motor_bus.h"
#include "dbus.h"
#include "spi.h"
#include "main.h"

BMI088_t bmi088;
volatile uint32_t bmi088_gyro_drdy_count;
volatile uint32_t bmi088_accel_drdy_count;

HAL_StatusTypeDef HardwareDrivers_Init(void)
{
    HAL_StatusTypeDef bmi_status;
    if (CanMotorBus_Init(&hcan1, &hcan2) != HAL_OK) return HAL_ERROR;
    if (DBus_Init(&huart2) != HAL_OK) return HAL_ERROR;

    /* CubeMX 已配置 EXTI；BMI 完整首读前禁止 DRDY 回调进入 SPI DMA。 */
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(INT_Accel_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(INT_Gyro_Pin);
    bmi_status = BMI088_Init(&bmi088, &hspi1,
                             CS_Accel_GPIO_Port, CS_Accel_Pin,
                             CS_Gyro_GPIO_Port, CS_Gyro_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(INT_Accel_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(INT_Gyro_Pin);
    if (bmi_status != HAL_OK) return HAL_ERROR;
    /* 与 CAN/SPI/DMA 中断同级（优先级 5），避免 1 kHz 的 BMI088 DRDY 中断以
     * 默认优先级 0 抢占 CAN RX0，导致反馈帧丢失、电机超时掉线。 */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5U, 0U);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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
