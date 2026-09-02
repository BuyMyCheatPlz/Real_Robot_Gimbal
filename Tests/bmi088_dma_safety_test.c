#include "bmi088.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

static uint32_t fake_tick;
static uint32_t dma_start_count;
static uint32_t abort_count;

uint32_t HAL_GetTick(void) { return fake_tick; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t state)
{
    (void)port;
    (void)pin;
    (void)state;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data,
                                   uint16_t length, uint32_t timeout)
{
    (void)hspi;
    (void)data;
    (void)length;
    (void)timeout;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *data,
                                  uint16_t length, uint32_t timeout)
{
    (void)hspi;
    (void)timeout;
    memset(data, 0, length);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi,
                                              uint8_t *tx, uint8_t *rx,
                                              uint16_t length)
{
    (void)tx;
    (void)rx;
    (void)length;
    ++dma_start_count;
    hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    hspi->hdmarx->State = HAL_DMA_STATE_BUSY;
    hspi->hdmatx->State = HAL_DMA_STATE_BUSY;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi)
{
    ++abort_count;
    hspi->State = HAL_SPI_STATE_READY;
    hspi->hdmarx->State = HAL_DMA_STATE_READY;
    hspi->hdmatx->State = HAL_DMA_STATE_READY;
    return HAL_OK;
}
void HAL_Delay(uint32_t delay_ms) { fake_tick += delay_ms; }

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

static void complete_rx_only(SPI_HandleTypeDef *spi)
{
    spi->State = HAL_SPI_STATE_READY;
    spi->hdmarx->State = HAL_DMA_STATE_READY;
    spi->hdmatx->State = HAL_DMA_STATE_BUSY;
    HAL_SPI_TxRxCpltCallback(spi);
}

int main(void)
{
    BMI088_t imu;
    SPI_HandleTypeDef spi;
    DMA_HandleTypeDef rx_dma;
    DMA_HandleTypeDef tx_dma;
    GPIO_TypeDef acc_port;
    GPIO_TypeDef gyro_port;

    memset(&imu, 0, sizeof(imu));
    memset(&spi, 0, sizeof(spi));
    spi.State = HAL_SPI_STATE_READY;
    rx_dma.State = HAL_DMA_STATE_READY;
    tx_dma.State = HAL_DMA_STATE_READY;
    spi.hdmarx = &rx_dma;
    spi.hdmatx = &tx_dma;
    imu.hspi = &spi;
    imu.acc_cs_port = &acc_port;
    imu.acc_cs_pin = 1U;
    imu.gyro_cs_port = &gyro_port;
    imu.gyro_cs_pin = 2U;
    imu.initialized = 1U;

    assert(BMI088_StartReadDMA(&imu) == HAL_OK);
    assert(dma_start_count == 1U);

    complete_rx_only(&spi);
    assert(dma_start_count == 1U);
    tx_dma.State = HAL_DMA_STATE_READY;
    assert(BMI088_ServiceDMA(&imu, fake_tick) == HAL_OK);
    assert(dma_start_count == 2U);

    complete_rx_only(&spi);
    assert(dma_start_count == 2U);
    tx_dma.State = HAL_DMA_STATE_READY;
    assert(BMI088_ServiceDMA(&imu, fake_tick) == HAL_OK);
    assert(dma_start_count == 3U);

    spi.State = HAL_SPI_STATE_READY;
    rx_dma.State = HAL_DMA_STATE_READY;
    tx_dma.State = HAL_DMA_STATE_READY;
    HAL_SPI_TxRxCpltCallback(&spi);
    assert((imu.dma_busy == 0U) && (imu.dma_data_ready != 0U));
    assert(imu.sample_count == 1U);

    fake_tick = 10U;
    assert(BMI088_StartReadDMA(&imu) == HAL_OK);
    assert(BMI088_ServiceDMA(&imu, 16U) == HAL_ERROR);
    assert((abort_count == 1U) && (imu.dma_recovery_count == 1U));
    assert((imu.dma_busy == 0U) && (imu.dma_recovery_required == 0U));
    return 0;
}
