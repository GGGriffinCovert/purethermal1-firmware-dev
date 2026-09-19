#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

#include "lepton.h"
#include "wedge_lab.h"

#include "project_config.h"

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

#define LEPTON_USART_PORT (USART2)

#define LEPTON_RESET_L_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)
#define LEPTON_RESET_L_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)

#define LEPTON_PW_DWN_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define LEPTON_PW_DWN_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

extern SPI_HandleTypeDef hspi2;

// These replace HAL library functions as they're a lot shorter and more specialized
static inline HAL_StatusTypeDef start_lepton_spi_dma(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
static inline HAL_StatusTypeDef setup_lepton_spi_rx(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
static void lepton_spi_rx_dma_cplt(DMA_HandleTypeDef *hdma);

void 	init_lepton_task();

lepton_status complete_lepton_transfer(lepton_buffer* buffer)
{
  // TODO: additional synchronization desired?
  return buffer->status;
}

void lepton_transfer(lepton_buffer *buf, int nlines)
{
  HAL_StatusTypeDef status;

  // DEBUG_PRINTF("Transfer starting: %p@%p\r\n", buf, packet);

  int packet_size = FRAME_HEADER_LENGTH +
		  ((g_format_y16 ? sizeof(uint16_t) : sizeof(rgb_t)) * FRAME_LINE_LENGTH) / sizeof(uint16_t);
  status = setup_lepton_spi_rx(&hspi2, buf->lines.data, packet_size * nlines);

  if (status != HAL_OK)
  {
    DEBUG_PRINTF("Error setting up SPI DMA receive: %d\r\n", status);
    buf->status = LEPTON_STATUS_RESYNC;
    return;
  }

  buf->status = LEPTON_STATUS_TRANSFERRING;
}

/* ---- wedge lab helpers --------------------------------------------------
 * /CS is PB12 (SPI2_NSS, hardware output: low for as long as SPE = 1). To
 * raise it, borrow the pin as a GPIO output; to lower it, hand it back to the
 * SPI. Only PB12's MODER bits change: GPIOB also carries the sensor power
 * enables (PB5, PB7) and I2C1 (PB8, PB9). */
#define LAB_PIN_MODE(pin, mode) \
  (GPIOB->MODER = (GPIOB->MODER & ~(3u << (2u * (pin)))) | ((uint32_t)(mode) << (2u * (pin))))

void lepton_cs_release(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  GPIOB->BSRR = (1u << 12);                  /* ODR12 = 1 first */
  LAB_PIN_MODE(12, 1u);                      /* then output: /CS high */
  __set_PRIMASK(primask);

  for (volatile int i = 0; i < 32; i++) {}
  g_lab.cs_idr_last = GPIOB->IDR;
  if ((g_lab.cs_idr_last & (1u << 12)) == 0)
    g_lab.cs_stuck_low++;
}

void lepton_cs_restore(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  LAB_PIN_MODE(12, 2u);                      /* AF5 SPI2_NSS: /CS low */
  __set_PRIMASK(primask);
}

/* Sensor hardware reset, same pins as lepton_init(). The caller waits between
 * the steps (190 ms, 190 ms, then the boot time). */
void lepton_hw_reset_assert(void)  { LEPTON_RESET_L_LOW; LEPTON_PW_DWN_LOW; }
void lepton_hw_pwdn_release(void)  { LEPTON_PW_DWN_HIGH; }
void lepton_hw_reset_release(void) { LEPTON_RESET_L_HIGH; }

/* MCU-side reset of the SPI path only: SPI2 through RCC, both DMA streams
 * through the HAL. /CS is held low and SCK held at its idle-high level as
 * plain GPIO outputs throughout, so the sensor sees no /CS edge and no stray
 * clock. Must be called with no transfer in flight. */
void lepton_spi_reinit(void)
{
  uint32_t cr1 = hspi2.Instance->CR1;
  uint32_t cr2 = hspi2.Instance->CR2 & ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  GPIOB->BSRR = (1u << (12 + 16)) | (1u << 13);   /* ODR12 = 0, ODR13 = 1 */
  LAB_PIN_MODE(12, 1u);
  LAB_PIN_MODE(13, 1u);
  __set_PRIMASK(primask);

  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();

  HAL_DMA_DeInit(hspi2.hdmarx);
  HAL_DMA_DeInit(hspi2.hdmatx);
  HAL_DMA_Init(hspi2.hdmarx);
  HAL_DMA_Init(hspi2.hdmatx);
  hspi2.hdmarx->XferCpltCallback = lepton_spi_rx_dma_cplt;  /* DeInit clears these */
  hspi2.hdmatx->XferCpltCallback = NULL;
  hspi2.hdmatx->XferErrorCallback = NULL;
  hspi2.State = HAL_SPI_STATE_READY;

  hspi2.Instance->CR2 = cr2;
  hspi2.Instance->CR1 = cr1 & ~SPI_CR1_SPE;
  hspi2.Instance->CR1 = cr1 | SPI_CR1_SPE;

  primask = __get_PRIMASK();
  __disable_irq();
  LAB_PIN_MODE(13, 2u);                      /* SCK back to AF5 (SPI idles it high) */
  LAB_PIN_MODE(12, 2u);                      /* /CS back to AF5: low, SPE = 1 */
  __set_PRIMASK(primask);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  DEBUG_PRINTF("SPI error!\n\r");
}

static void lepton_spi_rx_dma_cplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  lepton_buffer *buffer = (lepton_buffer*)hspi->pRxBuffPtr;

  /* Disable Rx/Tx DMA Requests and reset some peripheral state */
  hspi->Instance->CR2 &= (uint32_t)(~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN));
  hspi->TxXferCount = hspi->RxXferCount = 0;
  hspi->State = HAL_SPI_STATE_READY;

  buffer->status = LEPTON_STATUS_OK;
}

void lepton_init(void )
{
	LEPTON_RESET_L_LOW;
  LEPTON_PW_DWN_LOW;

  HAL_Delay(190);
  LEPTON_PW_DWN_HIGH;

	HAL_Delay(190);
  LEPTON_RESET_L_HIGH;

  hspi2.hdmarx->XferCpltCallback = lepton_spi_rx_dma_cplt;

  /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
  is performed in DMA reception complete callback  */
  hspi2.hdmatx->XferCpltCallback = NULL;
  hspi2.hdmatx->XferErrorCallback = NULL;

  /* Clear DBM bit */
  hspi2.hdmarx->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);
  hspi2.hdmatx->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

  /*Init field not used in handle to zero */
  hspi2.RxISR = 0;
  hspi2.TxISR = 0;

  /* Enable SPI peripheral */
  __HAL_SPI_ENABLE(&hspi2);

  init_lepton_task();
}

static inline HAL_StatusTypeDef start_lepton_spi_dma(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  hdma->Instance->CR &= ~DMA_SxCR_EN;

  /* Configure DMA Stream data length */
  hdma->Instance->NDTR = DataLength;

  /* Memory to Peripheral */
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Stream destination address */
    hdma->Instance->PAR = DstAddress;

    /* Configure DMA Stream source address */
    hdma->Instance->M0AR = SrcAddress;
  }
  /* Peripheral to Memory */
  else
  {
    /* Configure DMA Stream source address */
    hdma->Instance->PAR = SrcAddress;

    /* Configure DMA Stream destination address */
    hdma->Instance->M0AR = DstAddress;
  }

  hdma->Instance->CR |= (DMA_IT_TC | DMA_SxCR_EN);

  return HAL_OK;
}

static inline HAL_StatusTypeDef setup_lepton_spi_rx(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  /* Configure communication */
  hspi->State       = HAL_SPI_STATE_BUSY_RX;
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

  hspi->pTxBuffPtr  = hspi->pRxBuffPtr  = (uint8_t*)pData;
  hspi->TxXferSize  = hspi->RxXferSize  = Size;
  hspi->TxXferCount = hspi->RxXferCount = Size;

  /* Enable the Tx DMA Stream */
  start_lepton_spi_dma(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DR, hspi->TxXferCount);

  /* Enable the Rx DMA Stream */
  start_lepton_spi_dma(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)hspi->pRxBuffPtr, hspi->RxXferCount);

  /* Enable Rx DMA Request */
  /* Enable Tx DMA Request */
  hspi->Instance->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

  return HAL_OK;

}
