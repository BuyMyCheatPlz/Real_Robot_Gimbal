#ifndef TEST_STM32F4XX_HAL_H
#define TEST_STM32F4XX_HAL_H

#include <stdint.h>

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

typedef struct
{
    void *Instance;
    uint32_t ErrorCode;
} CAN_HandleTypeDef;

typedef struct
{
    uint32_t State;
} DMA_HandleTypeDef;

typedef struct
{
    DMA_HandleTypeDef *hdmarx;
} UART_HandleTypeDef;

typedef struct
{
    uint32_t State;
    DMA_HandleTypeDef *hdmarx;
    DMA_HandleTypeDef *hdmatx;
} SPI_HandleTypeDef;

typedef struct
{
    uint32_t unused;
} GPIO_TypeDef;

typedef struct
{
    uint32_t FilterBank;
    uint32_t FilterMode;
    uint32_t FilterScale;
    uint32_t FilterIdHigh;
    uint32_t FilterIdLow;
    uint32_t FilterMaskIdHigh;
    uint32_t FilterMaskIdLow;
    uint32_t FilterFIFOAssignment;
    uint32_t FilterActivation;
    uint32_t SlaveStartFilterBank;
} CAN_FilterTypeDef;

typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    uint32_t TransmitGlobalTime;
} CAN_TxHeaderTypeDef;

typedef struct
{
    uint32_t StdId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
} CAN_RxHeaderTypeDef;

#define CAN_FILTERMODE_IDLIST            1U
#define CAN_FILTERSCALE_16BIT            1U
#define CAN_RX_FIFO0                     0U
#define CAN_ID_STD                       0U
#define CAN_RTR_DATA                     0U
#define ENABLE                           1U
#define DISABLE                          0U
#define CAN_IT_RX_FIFO0_MSG_PENDING      (1U << 0)
#define CAN_IT_ERROR_WARNING             (1U << 1)
#define CAN_IT_ERROR_PASSIVE             (1U << 2)
#define CAN_IT_BUSOFF                    (1U << 3)
#define CAN_IT_LAST_ERROR_CODE           (1U << 4)
#define CAN_IT_ERROR                     (1U << 5)
#define CAN_TX_MAILBOX0                  (1U << 0)
#define CAN_TX_MAILBOX1                  (1U << 1)
#define CAN_TX_MAILBOX2                  (1U << 2)
#define HAL_CAN_ERROR_EPV                (1U << 1)
#define HAL_CAN_ERROR_BOF                (1U << 2)
#define CAN1_SCE_IRQn                    1
#define CAN2_SCE_IRQn                    2
#define HAL_SPI_STATE_READY              0U
#define HAL_SPI_STATE_BUSY_TX_RX         1U
#define HAL_DMA_STATE_READY              0U
#define HAL_DMA_STATE_BUSY               1U
#define GPIO_PIN_RESET                    0U
#define GPIO_PIN_SET                      1U
#define DMA_IT_HT                         1U
#define __HAL_DMA_DISABLE_IT(handle, interrupt) ((void)(handle), (void)(interrupt))

uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
uint32_t HAL_GetTick(void);
void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(int irq);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan,
                                       CAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan,
                                               uint32_t notifications);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan,
                                       CAN_TxHeaderTypeDef *header,
                                       uint8_t data[8], uint32_t *mailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan,
                                        uint32_t mailboxes);
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *hcan,
                                   uint32_t fifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t fifo,
                                       CAN_RxHeaderTypeDef *header,
                                       uint8_t data[8]);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t state);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data,
                                   uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *data,
                                  uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi,
                                              uint8_t *tx, uint8_t *rx,
                                              uint16_t length);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi);
void HAL_Delay(uint32_t delay_ms);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                               uint8_t *data,
                                               uint16_t length);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);

#endif
