#include "bmi088.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#define ACC_CHIP_ID_REG   0x00U
#define ACC_CHIP_ID       0x1EU
#define ACC_DATA_REG      0x12U
#define ACC_PWR_CONF_REG  0x7CU
#define ACC_PWR_CTRL_REG  0x7DU
#define ACC_SOFTRESET_REG 0x7EU
#define GYRO_CHIP_ID_REG  0x00U
#define GYRO_CHIP_ID      0x0FU

typedef enum
{
    DEVICE_NONE = 0,
    DEVICE_ACCEL,
    DEVICE_GYRO
} Device_t;

static GPIO_TypeDef acc_port;
static GPIO_TypeDef gyro_port;
static Device_t selected_device;
static uint8_t read_reg;
static uint8_t acc_spi_active;
static uint8_t acc_power_configured;
static uint8_t acc_enabled;

uint32_t HAL_GetTick(void) { return 0U; }
void HAL_Delay(uint32_t delay_ms) { (void)delay_ms; }

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t state)
{
    (void)pin;
    if (state == GPIO_PIN_RESET)
    {
        selected_device = (port == &acc_port) ? DEVICE_ACCEL : DEVICE_GYRO;
    }
    else if (((selected_device == DEVICE_ACCEL) && (port == &acc_port)) ||
             ((selected_device == DEVICE_GYRO) && (port == &gyro_port)))
    {
        selected_device = DEVICE_NONE;
    }
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data,
                                   uint16_t length, uint32_t timeout)
{
    uint8_t reg;

    (void)hspi;
    (void)timeout;
    assert(selected_device != DEVICE_NONE);
    if (length == 1U)
    {
        read_reg = (uint8_t)(data[0] & 0x7FU);
        return HAL_OK;
    }

    assert(length == 2U);
    reg = (uint8_t)(data[0] & 0x7FU);
    if (selected_device == DEVICE_ACCEL)
    {
          /* ACC 在复位后的 SPI 读取选择接口前会忽略 SPI 写入，并且必须先配置
              PWR_CONF，再配置 PWR_CTRL。 */
        if (acc_spi_active == 0U) return HAL_OK;
        if ((reg == ACC_SOFTRESET_REG) && (data[1] == 0xB6U))
        {
            acc_spi_active = 0U;
            acc_power_configured = 0U;
            acc_enabled = 0U;
        }
        else if ((reg == ACC_PWR_CONF_REG) && (data[1] == 0x00U))
        {
            acc_power_configured = 1U;
        }
        else if ((reg == ACC_PWR_CTRL_REG) && (data[1] == 0x04U) &&
                 (acc_power_configured != 0U))
        {
            acc_enabled = 1U;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *data,
                                  uint16_t length, uint32_t timeout)
{
    static uint8_t accel_read_phase;

    (void)hspi;
    (void)timeout;
    assert(selected_device != DEVICE_NONE);
    memset(data, 0, length);

    if ((selected_device == DEVICE_ACCEL) && (length == 1U) &&
        (accel_read_phase == 0U))
    {
        accel_read_phase = 1U; /* BMI088 ACC SPI 占位字节。 */
        return HAL_OK;
    }

    if (selected_device == DEVICE_ACCEL)
    {
        if ((read_reg == ACC_CHIP_ID_REG) && (length == 1U) &&
            (acc_spi_active != 0U))
        {
            data[0] = ACC_CHIP_ID;
        }
        else if ((read_reg == ACC_DATA_REG) && (length == 6U) &&
                 (acc_spi_active != 0U) && (acc_enabled != 0U))
        {
            /* x=+1000、y=-1000、z=+16384 原始计数。 */
            data[0] = 0xE8U;
            data[1] = 0x03U;
            data[2] = 0x18U;
            data[3] = 0xFCU;
            data[4] = 0x00U;
            data[5] = 0x40U;
        }
        /* 完成读取后，复位后的 ACC SPI 接口才可用。 */
        acc_spi_active = 1U;
        accel_read_phase = 0U;
    }
    else if ((read_reg == GYRO_CHIP_ID_REG) && (length == 1U))
    {
        data[0] = GYRO_CHIP_ID;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi,
                                              uint8_t *tx, uint8_t *rx,
                                              uint16_t length)
{
    (void)hspi;
    (void)tx;
    (void)rx;
    (void)length;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    return HAL_OK;
}

int main(void)
{
    BMI088_t imu;
    SPI_HandleTypeDef spi;

    memset(&imu, 0, sizeof(imu));
    memset(&spi, 0, sizeof(spi));
    assert(BMI088_Init(&imu, &spi, &acc_port, 1U, &gyro_port, 2U) == HAL_OK);
    assert(acc_enabled != 0U);
    assert(fabsf(imu.acceleration_m_s2[0]) > 1.0f);
    assert(fabsf(imu.acceleration_m_s2[1]) > 1.0f);
    assert(imu.acceleration_m_s2[2] > 20.0f);
    return 0;
}
