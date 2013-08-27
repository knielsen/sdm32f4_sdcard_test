#include "stm324x7i_eval.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_dma.h"

#include "my_sdio.h"


void SD_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOC and GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | SD_DETECT_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_SDIO);

  /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Configure PC.12 pin: CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /*!< Configure SD_SPI_DETECT_PIN pin: SD Card detect pin */
  GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);

  /* Enable the SDIO APB2 Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);

  /* Enable the DMA2 Clock */
  RCC_AHB1PeriphClockCmd(SD_SDIO_DMA_CLK, ENABLE);
}


void SD_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /*!< Disable SDIO Clock */
  SDIO_ClockCmd(DISABLE);
  
  /*!< Set Power State to OFF */
  SDIO_SetPowerState(SDIO_PowerState_OFF);

  /*!< DeInitializes the SDIO peripheral */
  SDIO_DeInit();
  
  /* Disable the SDIO APB2 Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, DISABLE);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_MCO);

  /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Configure PC.12 pin: CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}


/**
  * @brief  Configures the DMA2 Channel4 for SDIO Tx request.
  * @param  BufferSRC: pointer to the source buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;

  DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF | SD_SDIO_DMA_FLAG_DMEIF | SD_SDIO_DMA_FLAG_TEIF | SD_SDIO_DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

  /* DMA2 Stream3  or Stream6 disable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

  /* DMA2 Stream3  or Stream6 Config */
  DMA_DeInit(SD_SDIO_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferSRC;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SD_SDIO_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

  /* DMA2 Stream3  or Stream6 enable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
    
}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Rx request.
  * @param  BufferDST: pointer to the destination buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;

  DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF | SD_SDIO_DMA_FLAG_DMEIF | SD_SDIO_DMA_FLAG_TEIF | SD_SDIO_DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

  /* DMA2 Stream3  or Stream6 disable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

  /* DMA2 Stream3 or Stream6 Config */
  DMA_DeInit(SD_SDIO_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferDST;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SD_SDIO_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

  /* DMA2 Stream3 or Stream6 enable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
}


const char *
sdio_error_name(SD_Error err)
{
  const char *err_name;
  switch(err)
  {
  case SD_CMD_CRC_FAIL: err_name = "SD_CMD_CRC_FAIL"; break;
  case SD_DATA_CRC_FAIL: err_name = "SD_DATA_CRC_FAIL"; break;
  case SD_CMD_RSP_TIMEOUT: err_name = "SD_CMD_RSP_TIMEOUT"; break;
  case SD_DATA_TIMEOUT: err_name = "SD_DATA_TIMEOUT"; break;
  case SD_TX_UNDERRUN: err_name = "SD_TX_UNDERRUN"; break;
  case SD_RX_OVERRUN: err_name = "SD_RX_OVERRUN"; break;
  case SD_START_BIT_ERR: err_name = "SD_START_BIT_ERR"; break;
  case SD_CMD_OUT_OF_RANGE: err_name = "SD_CMD_OUT_OF_RANGE"; break;
  case SD_ADDR_MISALIGNED: err_name = "SD_ADDR_MISALIGNED"; break;
  case SD_BLOCK_LEN_ERR: err_name = "SD_BLOCK_LEN_ERR"; break;
  case SD_ERASE_SEQ_ERR: err_name = "SD_ERASE_SEQ_ERR"; break;
  case SD_BAD_ERASE_PARAM: err_name = "SD_BAD_ERASE_PARAM"; break;
  case SD_WRITE_PROT_VIOLATION: err_name = "SD_WRITE_PROT_VIOLATION"; break;
  case SD_LOCK_UNLOCK_FAILED: err_name = "SD_LOCK_UNLOCK_FAILED"; break;
  case SD_COM_CRC_FAILED: err_name = "SD_COM_CRC_FAILED"; break;
  case SD_ILLEGAL_CMD: err_name = "SD_ILLEGAL_CMD"; break;
  case SD_CARD_ECC_FAILED: err_name = "SD_CARD_ECC_FAILED"; break;
  case SD_CC_ERROR: err_name = "SD_CC_ERROR"; break;
  case SD_GENERAL_UNKNOWN_ERROR: err_name = "SD_GENERAL_UNKNOWN_ERROR"; break;
  case SD_STREAM_READ_UNDERRUN: err_name = "SD_STREAM_READ_UNDERRUN"; break;
  case SD_STREAM_WRITE_OVERRUN: err_name = "SD_STREAM_WRITE_OVERRUN"; break;
  case SD_CID_CSD_OVERWRITE: err_name = "SD_CID_CSD_OVERWRITE"; break;
  case SD_WP_ERASE_SKIP: err_name = "SD_WP_ERASE_SKIP"; break;
  case SD_CARD_ECC_DISABLED: err_name = "SD_CARD_ECC_DISABLED"; break;
  case SD_ERASE_RESET: err_name = "SD_ERASE_RESET"; break;
  case SD_AKE_SEQ_ERROR: err_name = "SD_AKE_SEQ_ERROR"; break;
  case SD_INVALID_VOLTRANGE: err_name = "SD_INVALID_VOLTRANGE"; break;
  case SD_ADDR_OUT_OF_RANGE: err_name = "SD_ADDR_OUT_OF_RANGE"; break;
  case SD_SWITCH_ERROR: err_name = "SD_SWITCH_ERROR"; break;
  case SD_SDIO_DISABLED: err_name = "SD_SDIO_DISABLED"; break;
  case SD_SDIO_FUNCTION_BUSY: err_name = "SD_SDIO_FUNCTION_BUSY"; break;
  case SD_SDIO_FUNCTION_FAILED: err_name = "SD_SDIO_FUNCTION_FAILED"; break;
  case SD_SDIO_UNKNOWN_FUNCTION: err_name = "SD_SDIO_UNKNOWN_FUNCTION"; break;
  case SD_INTERNAL_ERROR: err_name = "SD_INTERNAL_ERROR"; break;
  case SD_NOT_CONFIGURED: err_name = "SD_NOT_CONFIGURED"; break;
  case SD_REQUEST_PENDING: err_name = "SD_REQUEST_PENDING"; break;
  case SD_REQUEST_NOT_APPLICABLE: err_name = "SD_REQUEST_NOT_APPLICABLE"; break;
  case SD_INVALID_PARAMETER: err_name = "SD_INVALID_PARAMETER"; break;
  case SD_UNSUPPORTED_FEATURE: err_name = "SD_UNSUPPORTED_FEATURE"; break;
  case SD_UNSUPPORTED_HW: err_name = "SD_UNSUPPORTED_HW"; break;
  case SD_ERROR: err_name = "SD_ERROR"; break;
  case SD_OK: err_name = "SD_OK"; break;
  default:
    err_name = "UNKNOWN";
  }
  return err_name;
}
