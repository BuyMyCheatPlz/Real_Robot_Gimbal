#include "bmi088.h"
#include <string.h>

#define BMI088_SPI_TIMEOUT_MS       10U
#define BMI088_DMA_TIMEOUT_MS       5U
#define BMI088_ACC_CHIP_ID_REG      0x00U
#define BMI088_ACC_CHIP_ID          0x1EU
#define BMI088_ACC_DATA_REG         0x12U
#define BMI088_ACC_TEMP_REG         0x22U
#define BMI088_ACC_CONF_REG         0x40U
#define BMI088_ACC_RANGE_REG        0x41U
#define BMI088_ACC_INT1_IO_REG      0x53U
#define BMI088_ACC_INT_MAP_DATA_REG 0x58U
#define BMI088_ACC_PWR_CONF_REG     0x7CU
#define BMI088_ACC_PWR_CTRL_REG     0x7DU
#define BMI088_ACC_SOFTRESET_REG    0x7EU
#define BMI088_GYRO_CHIP_ID_REG     0x00U
#define BMI088_GYRO_CHIP_ID         0x0FU
#define BMI088_GYRO_DATA_REG        0x02U
#define BMI088_GYRO_RANGE_REG       0x0FU
#define BMI088_GYRO_BW_REG          0x10U
#define BMI088_GYRO_LPM1_REG        0x11U
#define BMI088_GYRO_SOFTRESET_REG   0x14U
#define BMI088_GYRO_INT_CTRL_REG    0x15U
#define BMI088_GYRO_INT_IO_REG      0x16U
#define BMI088_GYRO_INT_MAP_REG     0x18U

#define BMI088_ACC_SCALE ((6.0f * 9.80665f) / 32768.0f)
#define BMI088_GYRO_SCALE ((2000.0f * 0.017453292519943295f) / 32768.0f)

typedef enum
{
    BMI088_DMA_IDLE = 0,
    BMI088_DMA_ACCELERATION,
    BMI088_DMA_GYROSCOPE_PENDING,
    BMI088_DMA_GYROSCOPE,
    BMI088_DMA_TEMPERATURE_PENDING,
    BMI088_DMA_TEMPERATURE
} BMI088_DMAState_t;

static BMI088_t *dma_imu;
static volatile BMI088_DMAState_t dma_state = BMI088_DMA_IDLE;
static uint8_t dma_tx[8];
static uint8_t dma_rx[8];

static void select(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static void deselect(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static void request_dma_recovery(BMI088_t *imu, HAL_StatusTypeDef status)
{
    if (imu == 0) return;
    deselect(imu->acc_cs_port, imu->acc_cs_pin);
    deselect(imu->gyro_cs_port, imu->gyro_cs_pin);
    dma_state = BMI088_DMA_IDLE;
    imu->dma_status = status;
    imu->dma_data_ready = 0U;
    imu->dma_busy = 0U;
    imu->dma_recovery_required = 1U;
}

static uint8_t dma_handles_ready(const BMI088_t *imu)
{
    return (uint8_t)((imu != 0) && (imu->hspi != 0) &&
                     (imu->hspi->hdmarx != 0) && (imu->hspi->hdmatx != 0) &&
                     (imu->hspi->State == HAL_SPI_STATE_READY) &&
                     (imu->hspi->hdmarx->State == HAL_DMA_STATE_READY) &&
                     (imu->hspi->hdmatx->State == HAL_DMA_STATE_READY));
}

static HAL_StatusTypeDef write_reg(BMI088_t *imu, GPIO_TypeDef *port,
                                   uint16_t pin, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {(uint8_t)(reg & 0x7FU), value};
    HAL_StatusTypeDef status;
    select(port, pin);
    status = HAL_SPI_Transmit(imu->hspi, data, 2U, BMI088_SPI_TIMEOUT_MS);
    deselect(port, pin);
    return status;
}

static HAL_StatusTypeDef read_regs(BMI088_t *imu, GPIO_TypeDef *port,
                                   uint16_t pin, uint8_t reg, uint8_t *data,
                                   uint16_t length, uint8_t acc_dummy)
{
    uint8_t address = reg | 0x80U;
    uint8_t dummy = 0xFFU;
    HAL_StatusTypeDef status;
    select(port, pin);
    status = HAL_SPI_Transmit(imu->hspi, &address, 1U, BMI088_SPI_TIMEOUT_MS);
    if ((status == HAL_OK) && (acc_dummy != 0U))
        status = HAL_SPI_Receive(imu->hspi, &dummy, 1U, BMI088_SPI_TIMEOUT_MS);
    if (status == HAL_OK)
        status = HAL_SPI_Receive(imu->hspi, data, length, BMI088_SPI_TIMEOUT_MS);
    deselect(port, pin);
    return status;
}

static int16_t little_endian_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static void parse_acceleration(BMI088_t *imu, const uint8_t data[6])
{
    uint8_t axis;
    for (axis = 0U; axis < 3U; ++axis)
        imu->acceleration_m_s2[axis] =
            (float)little_endian_i16(&data[axis * 2U]) * BMI088_ACC_SCALE;
}

static void parse_gyroscope(BMI088_t *imu, const uint8_t data[6])
{
    uint8_t axis;
    for (axis = 0U; axis < 3U; ++axis)
        imu->angular_rate_rad_s[axis] =
            (float)little_endian_i16(&data[axis * 2U]) * BMI088_GYRO_SCALE;
}

static void parse_temperature(BMI088_t *imu, const uint8_t data[2])
{
    int16_t raw = (int16_t)(((uint16_t)data[0] << 3) | (data[1] >> 5));
    if (raw > 1023) raw -= 2048;
    imu->temperature_c = (float)raw * 0.125f + 23.0f;
}

static HAL_StatusTypeDef start_dma(BMI088_t *imu, GPIO_TypeDef *port,
                                   uint16_t pin, uint8_t reg, uint16_t length,
                                   BMI088_DMAState_t state)
{
    HAL_StatusTypeDef status;
    memset(dma_tx, 0xFF, sizeof(dma_tx));
    memset(dma_rx, 0, sizeof(dma_rx));
    dma_tx[0] = reg | 0x80U;
    dma_state = state;
    select(port, pin);
    status = HAL_SPI_TransmitReceive_DMA(imu->hspi, dma_tx, dma_rx, length);
    if (status != HAL_OK)
    {
        request_dma_recovery(imu, status);
    }
    else
        imu->dma_started_ms = HAL_GetTick();
    return status;
}

static HAL_StatusTypeDef continue_dma_chain(BMI088_t *imu)
{
    if (dma_handles_ready(imu) == 0U) return HAL_BUSY;
    if (dma_state == BMI088_DMA_GYROSCOPE_PENDING)
        return start_dma(imu, imu->gyro_cs_port, imu->gyro_cs_pin,
                         BMI088_GYRO_DATA_REG, 7U, BMI088_DMA_GYROSCOPE);
    if (dma_state == BMI088_DMA_TEMPERATURE_PENDING)
        return start_dma(imu, imu->acc_cs_port, imu->acc_cs_pin,
                         BMI088_ACC_TEMP_REG, 4U, BMI088_DMA_TEMPERATURE);
    return HAL_OK;
}

HAL_StatusTypeDef BMI088_Init(BMI088_t *imu, SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *acc_cs_port, uint16_t acc_cs_pin,
                              GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin)
{
    uint8_t id;
    if ((imu == 0) || (hspi == 0) || (acc_cs_port == 0) || (gyro_cs_port == 0))
        return HAL_ERROR;
    if (dma_state != BMI088_DMA_IDLE) return HAL_BUSY;

    imu->hspi = hspi;
    imu->acc_cs_port = acc_cs_port;
    imu->acc_cs_pin = acc_cs_pin;
    imu->gyro_cs_port = gyro_cs_port;
    imu->gyro_cs_pin = gyro_cs_pin;
    imu->initialized = 0U;
    imu->dma_busy = 0U;
    imu->dma_data_ready = 0U;
    imu->dma_status = HAL_OK;
    imu->dma_recovery_required = 0U;
    imu->dma_started_ms = 0U;
    imu->dma_recovery_count = 0U;
    imu->last_update_ms = 0U;
    imu->sample_count = 0U;
    deselect(acc_cs_port, acc_cs_pin);
    deselect(gyro_cs_port, gyro_cs_pin);
    HAL_Delay(5U);

    /* 第一次访问加速度计会把接口从 I2C 切换到 SPI。 */
    if (read_regs(imu, acc_cs_port, acc_cs_pin, BMI088_ACC_CHIP_ID_REG,
                  &id, 1U, 1U) != HAL_OK) return HAL_ERROR;
    if (read_regs(imu, acc_cs_port, acc_cs_pin, BMI088_ACC_CHIP_ID_REG,
                  &id, 1U, 1U) != HAL_OK) return HAL_ERROR;
    if (id != BMI088_ACC_CHIP_ID) return HAL_ERROR;

    if (write_reg(imu, acc_cs_port, acc_cs_pin, BMI088_ACC_SOFTRESET_REG,
                  0xB6U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(50U);

     /* ACC 软复位会将接口恢复为默认的 I2C 状态。第一次 SPI 读取只用于切换到 SPI，
         因此在检查芯片 ID 前丢弃该次读取结果。 */
    if (read_regs(imu, acc_cs_port, acc_cs_pin, BMI088_ACC_CHIP_ID_REG,
                  &id, 1U, 1U) != HAL_OK) return HAL_ERROR;
    if (read_regs(imu, acc_cs_port, acc_cs_pin, BMI088_ACC_CHIP_ID_REG,
                  &id, 1U, 1U) != HAL_OK) return HAL_ERROR;
    if (id != BMI088_ACC_CHIP_ID) return HAL_ERROR;

    /* 按要求的顺序解除加速度计的挂起状态。 */
    if (write_reg(imu, acc_cs_port, acc_cs_pin, BMI088_ACC_PWR_CONF_REG,
                  0x00U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(5U);
    if (write_reg(imu, acc_cs_port, acc_cs_pin, BMI088_ACC_PWR_CTRL_REG,
                  0x04U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(5U);
    if (write_reg(imu, acc_cs_port, acc_cs_pin, BMI088_ACC_CONF_REG,
                  0xABU) != HAL_OK) return HAL_ERROR;
    if (write_reg(imu, acc_cs_port, acc_cs_pin, BMI088_ACC_RANGE_REG,
                  0x01U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(50U);

    if (read_regs(imu, gyro_cs_port, gyro_cs_pin, BMI088_GYRO_CHIP_ID_REG,
                  &id, 1U, 0U) != HAL_OK) return HAL_ERROR;
    if (id != BMI088_GYRO_CHIP_ID) return HAL_ERROR;
    if (write_reg(imu, gyro_cs_port, gyro_cs_pin, BMI088_GYRO_SOFTRESET_REG,
                  0xB6U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(50U);
    if (write_reg(imu, gyro_cs_port, gyro_cs_pin, BMI088_GYRO_RANGE_REG,
                  0x00U) != HAL_OK) return HAL_ERROR;
    if (write_reg(imu, gyro_cs_port, gyro_cs_pin, BMI088_GYRO_BW_REG,
                  0x02U) != HAL_OK) return HAL_ERROR;
    if (write_reg(imu, gyro_cs_port, gyro_cs_pin, BMI088_GYRO_LPM1_REG,
                  0x00U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(30U);

    /* 加速度计 INT1：脉冲输出、高电平有效、推挽数据就绪中断。 */
    if (write_reg(imu, acc_cs_port, acc_cs_pin, BMI088_ACC_INT1_IO_REG,
                  0x0AU) != HAL_OK) return HAL_ERROR;
    if (write_reg(imu, acc_cs_port, acc_cs_pin, BMI088_ACC_INT_MAP_DATA_REG,
                  0x04U) != HAL_OK) return HAL_ERROR;

    /* 陀螺仪 INT3：高电平有效、推挽数据就绪中断。 */
    if (write_reg(imu, gyro_cs_port, gyro_cs_pin, BMI088_GYRO_INT_IO_REG,
                  0x01U) != HAL_OK) return HAL_ERROR;
    if (write_reg(imu, gyro_cs_port, gyro_cs_pin, BMI088_GYRO_INT_MAP_REG,
                  0x01U) != HAL_OK) return HAL_ERROR;
    imu->initialized = 1U;
    if (BMI088_Read(imu) != HAL_OK)
    {
        imu->initialized = 0U;
        return HAL_ERROR;
    }
    /* 同步首读完成后才允许陀螺仪产生 DRDY，避免初始化阶段 DMA 抢占 SPI。 */
    if (write_reg(imu, gyro_cs_port, gyro_cs_pin, BMI088_GYRO_INT_CTRL_REG,
                  0x80U) != HAL_OK)
    {
        imu->initialized = 0U;
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef BMI088_ReadAcceleration(BMI088_t *imu)
{
    uint8_t data[6];
    if ((imu == 0) || (imu->initialized == 0U)) return HAL_ERROR;
    if (imu->dma_busy != 0U) return HAL_BUSY;
    if (read_regs(imu, imu->acc_cs_port, imu->acc_cs_pin, BMI088_ACC_DATA_REG,
                  data, 6U, 1U) != HAL_OK) return HAL_ERROR;
    parse_acceleration(imu, data);
    return HAL_OK;
}

HAL_StatusTypeDef BMI088_ReadGyroscope(BMI088_t *imu)
{
    uint8_t data[6];
    if ((imu == 0) || (imu->initialized == 0U)) return HAL_ERROR;
    if (imu->dma_busy != 0U) return HAL_BUSY;
    if (read_regs(imu, imu->gyro_cs_port, imu->gyro_cs_pin, BMI088_GYRO_DATA_REG,
                  data, 6U, 0U) != HAL_OK) return HAL_ERROR;
    parse_gyroscope(imu, data);
    return HAL_OK;
}

HAL_StatusTypeDef BMI088_ReadTemperature(BMI088_t *imu)
{
    uint8_t data[2];
    if ((imu == 0) || (imu->initialized == 0U)) return HAL_ERROR;
    if (imu->dma_busy != 0U) return HAL_BUSY;
    if (read_regs(imu, imu->acc_cs_port, imu->acc_cs_pin, BMI088_ACC_TEMP_REG,
                  data, 2U, 1U) != HAL_OK) return HAL_ERROR;
    parse_temperature(imu, data);
    return HAL_OK;
}

HAL_StatusTypeDef BMI088_Read(BMI088_t *imu)
{
    if (BMI088_ReadAcceleration(imu) != HAL_OK) return HAL_ERROR;
    if (BMI088_ReadGyroscope(imu) != HAL_OK) return HAL_ERROR;
    return BMI088_ReadTemperature(imu);
}

HAL_StatusTypeDef BMI088_StartReadDMA(BMI088_t *imu)
{
    if ((imu == 0) || (imu->initialized == 0U)) return HAL_ERROR;
    if ((imu->dma_recovery_required != 0U) || (imu->dma_busy != 0U) ||
        (dma_state != BMI088_DMA_IDLE)) return HAL_BUSY;
    dma_imu = imu;
    imu->dma_busy = 1U;
    imu->dma_data_ready = 0U;
    imu->dma_status = HAL_BUSY;
    /* 加速度计 DMA 读取包含地址、一个空字节和六个数据字节。 */
    return start_dma(imu, imu->acc_cs_port, imu->acc_cs_pin,
                     BMI088_ACC_DATA_REG, 8U, BMI088_DMA_ACCELERATION);
}

HAL_StatusTypeDef BMI088_ServiceDMA(BMI088_t *imu, uint32_t now_ms)
{
    HAL_StatusTypeDef status;
    if ((imu == 0) || (imu->initialized == 0U)) return HAL_ERROR;

    if ((imu->dma_busy != 0U) &&
        ((now_ms - imu->dma_started_ms) > BMI088_DMA_TIMEOUT_MS))
        request_dma_recovery(imu, HAL_TIMEOUT);

    if (imu->dma_recovery_required != 0U)
    {
        (void)HAL_SPI_Abort(imu->hspi);
        deselect(imu->acc_cs_port, imu->acc_cs_pin);
        deselect(imu->gyro_cs_port, imu->gyro_cs_pin);
        dma_state = BMI088_DMA_IDLE;
        imu->dma_busy = 0U;
        imu->dma_data_ready = 0U;
        imu->dma_recovery_required = 0U;
        ++imu->dma_recovery_count;
        return HAL_ERROR;
    }

    if ((dma_state == BMI088_DMA_GYROSCOPE_PENDING) ||
        (dma_state == BMI088_DMA_TEMPERATURE_PENDING))
    {
        status = continue_dma_chain(imu);
        if (status == HAL_ERROR) return HAL_ERROR;
    }
    return HAL_OK;
}

uint8_t BMI088_DMADataReady(const BMI088_t *imu)
{
    return (imu == 0) ? 0U : imu->dma_data_ready;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if ((dma_imu == 0) || (hspi != dma_imu->hspi)) return;

    switch (dma_state)
    {
        case BMI088_DMA_ACCELERATION:
            deselect(dma_imu->acc_cs_port, dma_imu->acc_cs_pin);
            parse_acceleration(dma_imu, &dma_rx[2]);
            dma_state = BMI088_DMA_GYROSCOPE_PENDING;
            (void)continue_dma_chain(dma_imu);
            break;

        case BMI088_DMA_GYROSCOPE:
            deselect(dma_imu->gyro_cs_port, dma_imu->gyro_cs_pin);
            parse_gyroscope(dma_imu, &dma_rx[1]);
            dma_state = BMI088_DMA_TEMPERATURE_PENDING;
            (void)continue_dma_chain(dma_imu);
            break;

        case BMI088_DMA_TEMPERATURE:
            deselect(dma_imu->acc_cs_port, dma_imu->acc_cs_pin);
            parse_temperature(dma_imu, &dma_rx[2]);
            dma_state = BMI088_DMA_IDLE;
            dma_imu->dma_status = HAL_OK;
            dma_imu->last_update_ms = HAL_GetTick();
            ++dma_imu->sample_count;
            dma_imu->dma_busy = 0U;
            dma_imu->dma_data_ready = 1U;
            break;

        default:
            break;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if ((dma_imu == 0) || (hspi != dma_imu->hspi)) return;
    request_dma_recovery(dma_imu, HAL_ERROR);
}
