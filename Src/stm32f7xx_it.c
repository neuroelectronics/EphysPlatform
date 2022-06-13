/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dataMGR.h"
#include "stm32f7xx_hal_dma.h"
#include "CE32_ioncom.h"
#include "CE32_macro.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint8_t data_buf_RX1[0x10];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_RX2[0x10];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_RX3[0x10];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_RX4[0x10];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_TX[0x100];		// This records @ 20kHz x 32CH
extern uint8_t data_buf_RX_CDC[BUF_SIZE];

extern dataMGR MGR_RX1;
extern dataMGR MGR_RX2;
extern dataMGR MGR_RX3;
extern dataMGR MGR_RX4;
extern dataMGR MGR_TX;
extern dataMGR MGR_CDC;

extern CE32_IONCOM_Handle IC_handle1;
extern CE32_IONCOM_Handle IC_handle2;
extern CE32_IONCOM_Handle IC_handle3;
extern CE32_IONCOM_Handle IC_handle4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern DMA_HandleTypeDef hdma_adc1;
extern SD_HandleTypeDef hsd1;
extern TIM_HandleTypeDef htim14;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */
extern ADC_HandleTypeDef hadc1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	//hadc1.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
	//USART3_RX
	DMA_Base_Registers *regs = (DMA_Base_Registers *)  hdma_usart3_rx.StreamBaseAddress;
	regs->IFCR = DMA_FLAG_TCIF0_4 << hdma_usart3_rx.StreamIndex;
	regs->IFCR = DMA_FLAG_HTIF0_4<< hdma_usart3_rx.StreamIndex;
	dataMGR_enQueue_Nbytes(&IC_handle3.RX_MGR,IC_handle3.DMA_TransSize);
	UART_IONCOM_Bank_EnqueueBank(&IC_handle3);
	if((IC_handle3.huart->hdmarx->Instance->CR&DMA_SxCR_CT)!=0) //Check which buffer is being used currently
	{
		IC_handle3.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle3.RX_MGR.dataPtr + IC_handle3.DMA_bank_in*(IC_handle3.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
	else
	{
		IC_handle3.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle3.RX_MGR.dataPtr + IC_handle3.DMA_bank_in*(IC_handle3.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
  /* USER CODE END DMA1_Stream1_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
	//USART2_RX
	DMA_Base_Registers *regs = (DMA_Base_Registers *)  hdma_usart2_rx.StreamBaseAddress;
	regs->IFCR = DMA_FLAG_TCIF0_4 << hdma_usart2_rx.StreamIndex;
	regs->IFCR = DMA_FLAG_HTIF0_4<< hdma_usart2_rx.StreamIndex;
	dataMGR_enQueue_Nbytes(&IC_handle2.RX_MGR,IC_handle2.DMA_TransSize);
	UART_IONCOM_Bank_EnqueueBank(&IC_handle2);
	if((IC_handle2.huart->hdmarx->Instance->CR&DMA_SxCR_CT)!=0) //Check which buffer is being used currently
	{
		IC_handle2.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle2.RX_MGR.dataPtr + IC_handle2.DMA_bank_in*(IC_handle2.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
	else
	{
		IC_handle2.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle2.RX_MGR.dataPtr + IC_handle2.DMA_bank_in*(IC_handle2.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
  /* USER CODE END DMA1_Stream5_IRQn 0 */
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */
	TIM14->SR&=~TIM_SR_UIF;
	HAL_GPIO_WritePin(MODE1_LED_GPIO_Port ,MODE1_LED_Pin|MODE2_LED_Pin|MODE3_LED_Pin|MODE4_LED_Pin,GPIO_PIN_RESET);	
  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC1_IRQn 0 */

  /* USER CODE END SDMMC1_IRQn 0 */
  /* USER CODE BEGIN SDMMC1_IRQn 1 */

  /* USER CODE END SDMMC1_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
	DMA_Base_Registers *regs = (DMA_Base_Registers *)  hdma_adc1.StreamBaseAddress;
	int cnt=0;
	regs->IFCR = DMA_FLAG_TCIF0_4 << hdma_adc1.StreamIndex;
	regs->IFCR = DMA_FLAG_HTIF0_4<< hdma_adc1.StreamIndex;

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */
	dataMGR_enQueue_Nbytes(&MGR_CDC,IC_handle1.DMA_TransSize);
	UART_IONCOM_Bank_EnqueueBank(&IC_handle1);
	if((hadc1.DMA_Handle->Instance->CR&DMA_SxCR_CT)!=0) //Check which buffer is being used currently
	{
		hadc1.DMA_Handle->Instance->M0AR = (uint32_t)MGR_CDC.dataPtr + IC_handle1.DMA_bank_in*(IC_handle1.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
	else
	{
		hadc1.DMA_Handle->Instance->M1AR = (uint32_t)MGR_CDC.dataPtr + IC_handle1.DMA_bank_in*(IC_handle1.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
	//dataMGR_enQueue_Nbytes(&MGR_CDC,cnt*BUF_SIZE/2);
  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
	//USART6_RX
	DMA_Base_Registers *regs = (DMA_Base_Registers *)  hdma_usart6_rx.StreamBaseAddress;
	regs->IFCR = DMA_FLAG_TCIF0_4 << hdma_usart6_rx.StreamIndex;
	regs->IFCR = DMA_FLAG_HTIF0_4<< hdma_usart6_rx.StreamIndex;
	dataMGR_enQueue_Nbytes(&IC_handle4.RX_MGR,IC_handle4.DMA_TransSize);
	UART_IONCOM_Bank_EnqueueBank(&IC_handle4);
	if((IC_handle4.huart->hdmarx->Instance->CR&DMA_SxCR_CT)!=0) //Check which buffer is being used currently
	{
		IC_handle4.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle4.RX_MGR.dataPtr + IC_handle4.DMA_bank_in*(IC_handle4.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
	else
	{
		IC_handle4.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle4.RX_MGR.dataPtr + IC_handle4.DMA_bank_in*(IC_handle4.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
  /* USER CODE END DMA2_Stream1_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
	//USART1_RX
	DMA_Base_Registers *regs = (DMA_Base_Registers *)  hdma_usart1_rx.StreamBaseAddress;
	regs->IFCR = DMA_FLAG_TCIF0_4 << hdma_usart1_rx.StreamIndex;
	regs->IFCR = DMA_FLAG_HTIF0_4<< hdma_usart1_rx.StreamIndex;
	dataMGR_enQueue_Nbytes(&IC_handle1.RX_MGR,IC_handle1.DMA_TransSize);
	UART_IONCOM_Bank_EnqueueBank(&IC_handle1);
	if((IC_handle1.huart->hdmarx->Instance->CR&DMA_SxCR_CT)!=0) //Check which buffer is being used currently
	{
		IC_handle1.huart->hdmarx->Instance->M0AR = (uint32_t)IC_handle1.RX_MGR.dataPtr + IC_handle1.DMA_bank_in*(IC_handle1.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
	else
	{
		IC_handle1.huart->hdmarx->Instance->M1AR = (uint32_t)IC_handle1.RX_MGR.dataPtr + IC_handle1.DMA_bank_in*(IC_handle1.DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	}
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS global interrupt.
  */
void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */

  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN UART8_IRQn 0 */

  /* USER CODE END UART8_IRQn 0 */
  /* USER CODE BEGIN UART8_IRQn 1 */

  /* USER CODE END UART8_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
