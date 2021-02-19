/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "dataMGR.h"
#include "CE32_ioncom.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern uint8_t data_buf_RX1[BUF_SIZE];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_RX2[BUF_SIZE];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_RX3[BUF_SIZE];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_RX4[BUF_SIZE];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_TX[BUF_SIZE];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_RX_CDC[BUF_SIZE];
extern dataMGR MGR_RX1;
extern dataMGR MGR_RX2;
extern dataMGR MGR_RX3;
extern dataMGR MGR_RX4;
extern dataMGR MGR_TX;
extern dataMGR MGR_CDC;

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;

extern CE32_IONCOM_Handle IC_handle1;
extern CE32_IONCOM_Handle IC_handle2;
extern CE32_IONCOM_Handle IC_handle3;
extern CE32_IONCOM_Handle IC_handle4;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_HS(void);
static int8_t CDC_DeInit_HS(void);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_HS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_HS =
{
  CDC_Init_HS,
  CDC_DeInit_HS,
  CDC_Control_HS,
  CDC_Receive_HS,
  CDC_TransmitCplt_HS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
  switch(cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:
				UART_DISABLE(IC_handle1.huart->Instance);
		IC_handle1.huart->Init.BaudRate = *(uint32_t*)pbuf;
		IC_handle1.huart->Init.WordLength = UART_WORDLENGTH_8B;
		IC_handle1.huart->Init.StopBits = UART_STOPBITS_1;
		IC_handle1.huart->Init.Parity = UART_PARITY_NONE;
		IC_handle1.huart->Init.Mode = UART_MODE_TX_RX;
		IC_handle1.huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
		IC_handle1.huart->Init.OverSampling = UART_OVERSAMPLING_16;
		IC_handle1.huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		IC_handle1.huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		if (HAL_UART_Init(IC_handle1.huart) != HAL_OK)
		{
			Error_Handler();
		}
		//IC_handle.huart->Instance->CR1|=USART_CR1_RXNEIE;  //Enable RX interrput to receive commands
		//configure USART4 DMA
		UART_IONCOM_DMA_Init(&IC_handle1);
		IC_handle1.huart->hdmarx->Instance->PAR = (uint32_t)&IONCOM_UART_RDR(IC_handle1.huart->Instance);
		IC_handle1.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle1.RX_MGR.dataPtr; //Set the DMA to be in circular mode and automatic filling the buffer
		IC_handle1.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle1.RX_MGR.dataPtr+IC_handle1.DMA_TransSize; //Set the DMA to be in circular mode and automatic filling the buffer
		IC_handle1.huart->hdmarx->Instance->NDTR = IC_handle1.DMA_TransSize;
		
		//SET_BIT(USART6->CR3, USART_CR3_DMAT); //enable UART_DMA_request
		
	
		UART_DISABLE(IC_handle2.huart->Instance);
		IC_handle2.huart->Init.BaudRate = *(uint32_t*)pbuf;
		IC_handle2.huart->Init.WordLength = UART_WORDLENGTH_8B;
		IC_handle2.huart->Init.StopBits = UART_STOPBITS_1;
		IC_handle2.huart->Init.Parity = UART_PARITY_NONE;
		IC_handle2.huart->Init.Mode = UART_MODE_TX_RX;
		IC_handle2.huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
		IC_handle2.huart->Init.OverSampling = UART_OVERSAMPLING_16;
		IC_handle2.huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		IC_handle2.huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		if (HAL_UART_Init(IC_handle2.huart) != HAL_OK)
		{
			Error_Handler();
		}
		//IC_handle.huart->Instance->CR1|=USART_CR1_RXNEIE;  //Enable RX interrput to receive commands
		//configure USART4 DMA
		UART_IONCOM_DMA_Init(&IC_handle2);
		IC_handle2.huart->hdmarx->Instance->PAR = (uint32_t)&IONCOM_UART_RDR(IC_handle2.huart->Instance);
		IC_handle2.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle2.RX_MGR.dataPtr; //Set the DMA to be in circular mode and automatic filling the buffer
		IC_handle2.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle2.RX_MGR.dataPtr+IC_handle2.DMA_TransSize; //Set the DMA to be in circular mode and automatic filling the buffer
		IC_handle2.huart->hdmarx->Instance->NDTR = IC_handle2.DMA_TransSize;
		
		//SET_BIT(USART6->CR3, USART_CR3_DMAT); //enable UART_DMA_request
		
		UART_DISABLE(IC_handle3.huart->Instance);
		IC_handle3.huart->Init.BaudRate = *(uint32_t*)pbuf;
		IC_handle3.huart->Init.WordLength = UART_WORDLENGTH_8B;
		IC_handle3.huart->Init.StopBits = UART_STOPBITS_1;
		IC_handle3.huart->Init.Parity = UART_PARITY_NONE;
		IC_handle3.huart->Init.Mode = UART_MODE_TX_RX;
		IC_handle3.huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
		IC_handle3.huart->Init.OverSampling = UART_OVERSAMPLING_16;
		IC_handle3.huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		IC_handle3.huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		if (HAL_UART_Init(IC_handle3.huart) != HAL_OK)
		{
			Error_Handler();
		}
		//IC_handle.huart->Instance->CR1|=USART_CR1_RXNEIE;  //Enable RX interrput to receive commands
		//configure USART4 DMA
		UART_IONCOM_DMA_Init(&IC_handle3);
		IC_handle3.huart->hdmarx->Instance->PAR = (uint32_t)&IONCOM_UART_RDR(IC_handle3.huart->Instance);
		IC_handle3.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle3.RX_MGR.dataPtr; //Set the DMA to be in circular mode and automatic filling the buffer
		IC_handle3.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle3.RX_MGR.dataPtr+IC_handle3.DMA_TransSize; //Set the DMA to be in circular mode and automatic filling the buffer
		IC_handle3.huart->hdmarx->Instance->NDTR = IC_handle3.DMA_TransSize;
		
		//SET_BIT(USART6->CR3, USART_CR3_DMAT); //enable UART_DMA_request
		
		UART_DISABLE(IC_handle4.huart->Instance);
		IC_handle4.huart->Init.BaudRate = *(uint32_t*)pbuf;
		IC_handle4.huart->Init.WordLength = UART_WORDLENGTH_8B;
		IC_handle4.huart->Init.StopBits = UART_STOPBITS_1;
		IC_handle4.huart->Init.Parity = UART_PARITY_NONE;
		IC_handle4.huart->Init.Mode = UART_MODE_TX_RX;
		IC_handle4.huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
		IC_handle4.huart->Init.OverSampling = UART_OVERSAMPLING_16;
		IC_handle4.huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		IC_handle4.huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		if (HAL_UART_Init(IC_handle4.huart) != HAL_OK)
		{
			Error_Handler();
		}
		//IC_handle.huart->Instance->CR1|=USART_CR1_RXNEIE;  //Enable RX interrput to receive commands
		//configure USART4 DMA
		UART_IONCOM_DMA_Init(&IC_handle4);
		IC_handle4.huart->hdmarx->Instance->PAR = (uint32_t)&IONCOM_UART_RDR(IC_handle4.huart->Instance);
		IC_handle4.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle4.RX_MGR.dataPtr; //Set the DMA to be in circular mode and automatic filling the buffer
		IC_handle4.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle4.RX_MGR.dataPtr+IC_handle4.DMA_TransSize; //Set the DMA to be in circular mode and automatic filling the buffer
		IC_handle4.huart->hdmarx->Instance->NDTR = IC_handle4.DMA_TransSize;
		
		//SET_BIT(USART6->CR3, USART_CR3_DMAT); //enable UART_DMA_request
		dataMGR_init(&MGR_TX,(char*) data_buf_TX,sizeof(data_buf_TX));					//FIFO setup 
		dataMGR_init(&MGR_RX1,(char*) data_buf_RX4,sizeof(data_buf_RX4));					//RX FIFO setup 
		dataMGR_init(&MGR_RX2,(char*) data_buf_RX4,sizeof(data_buf_RX4));					//RX FIFO setup 
		dataMGR_init(&MGR_RX3,(char*) data_buf_RX4,sizeof(data_buf_RX4));					//RX FIFO setup 
		dataMGR_init(&MGR_RX4,(char*) data_buf_RX4,sizeof(data_buf_RX4));					//RX FIFO setup 
		dataMGR_init(&MGR_CDC,(char*) data_buf_RX_CDC,sizeof(data_buf_RX_CDC));					//RX FIFO setup 
		
		DMA_DISABLE(IC_handle1.huart->hdmarx->Instance);
		UART_DISABLE(IC_handle1.huart->Instance);
		IC_handle1.huart->hdmarx->Instance->CR|=DMA_IT_TC|DMA_SxCR_DBM;  //ENABLE DMA INterrupt and double buffer mode
		UART_IONCOM_Bank_Reset_DBM(&IC_handle1);
		
		DMA_DISABLE(IC_handle2.huart->hdmarx->Instance);
		UART_DISABLE(IC_handle2.huart->Instance);
		IC_handle2.huart->hdmarx->Instance->CR|=DMA_IT_TC|DMA_SxCR_DBM;  //ENABLE DMA INterrupt and double buffer mode
		UART_IONCOM_Bank_Reset_DBM(&IC_handle2);

		DMA_DISABLE(IC_handle3.huart->hdmarx->Instance);
		UART_DISABLE(IC_handle3.huart->Instance);
		IC_handle3.huart->hdmarx->Instance->CR|=DMA_IT_TC|DMA_SxCR_DBM;  //ENABLE DMA INterrupt and double buffer mode
		UART_IONCOM_Bank_Reset_DBM(&IC_handle3);
		
		DMA_DISABLE(IC_handle4.huart->hdmarx->Instance);
		UART_DISABLE(IC_handle4.huart->Instance);
		IC_handle4.huart->hdmarx->Instance->CR|=DMA_IT_TC|DMA_SxCR_DBM;  //ENABLE DMA INterrupt and double buffer mode
		UART_IONCOM_Bank_Reset_DBM(&IC_handle4);
		
		UART_IONCOM_DMA_Enable(&IC_handle1);
		UART_IONCOM_DMA_Enable(&IC_handle2);
		UART_IONCOM_DMA_Enable(&IC_handle3);		
		UART_IONCOM_DMA_Enable(&IC_handle4);
		
		DMA_ENABLE(IC_handle1.huart->hdmarx->Instance);
		UART_ENABLE(IC_handle1.huart->Instance);
		DMA_ENABLE(IC_handle2.huart->hdmarx->Instance);
		UART_ENABLE(IC_handle2.huart->Instance);
		DMA_ENABLE(IC_handle3.huart->hdmarx->Instance);
		UART_ENABLE(IC_handle3.huart->Instance);
		DMA_ENABLE(IC_handle4.huart->hdmarx->Instance);
		UART_ENABLE(IC_handle4.huart->Instance);
    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
  /* USER CODE END 12 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_HS
  *         Data transmited callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
