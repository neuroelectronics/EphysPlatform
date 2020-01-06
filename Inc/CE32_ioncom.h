#include "stm32f7xx_hal.h"
#include "dataMGR.h"
#include "CE32_macro.h"
#include "main.h"

#ifndef __CE32_IONCOM
#define __CE32_IONCOM

//Define the following part accroding to the project setup


#define HJ_CMD_BUFSIZE 512
#define HJ_CMD_SEQ 4

#ifdef __STM32F4xx_HAL_H
	#define __NDTR_ADDR(__handle__) ((__handle__)->Instance->NDTR)
	#define IONCOM_HANDLE_RDR handle->huart->Instance->DR
	#define IONCOM_UART_RDR(__handle__) (__handle__->DR)
	#define IONCOM_HANDLE_TDR handle->huart->Instance->DR
	#define IONCOM_UART_TDR(__handle__) (__handle__->DR)
	#define UART_CHECK_TXE(__handle__) (((__handle__)->SR) & (uint32_t)USART_SR_TXE)  
	#define UART_CHECK_RXNE(__handle__) (((__handle__)->SR) & (uint32_t)USART_SR_RXNE)  
	#define UART_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->SR) & (__FLAG__)) == (__FLAG__))
	#define UART_ENABLE(__HANDLE__) ((__HANDLE__)->CR1 |=  USART_CR1_UE)
	#define UART_DISABLE(__HANDLE__) ((__HANDLE__)->CR1 &= (~USART_CR1_UE))
	#define DMA_ENABLE(__HANDLE__)      ((__HANDLE__)->CR |=  DMA_SxCR_EN)
	#define DMA_DISABLE(__HANDLE__)     ((__HANDLE__)->CR &=  ~DMA_SxCR_EN);while(((__HANDLE__)->CR &  DMA_SxCR_EN)!=0)
	#define PULL_UART_BSY(__HANDLE__) while(!(UART_GET_FLAG((__HANDLE__),UART_FLAG_TXE)));
#endif


#if defined(__STM32L4xx_HAL_H) || defined(__STM32F3xx_HAL_H)
	#define __NDTR_ADDR(__handle__) ((__handle__)->Instance->CNDTR)
	#define IONCOM_UART_RDR(__handle__) (__handle__->TDR)
	#define UART_CHECK_TXE(__handle__) (READ_REG((__handle__)->ISR)&(uint32_t)(USART_ISR_TXE))
	#define UART_CHECK_RXNE(__handle__) (READ_REG((__handle__)->ISR)&(uint32_t)(USART_ISR_RXNE))
#endif

#if defined(__STM32F7xx_HAL_H)
	#define __NDTR_ADDR(__handle__) ((__handle__)->Instance->NDTR)
	#define IONCOM_HANDLE_RDR handle->huart->Instance->RDR
	#define IONCOM_UART_RDR(__handle__) (__handle__->RDR)
	#define IONCOM_HANDLE_TDR handle->huart->Instance->TDR
	#define IONCOM_UART_TDR(__handle__) (__handle__->TDR)
	#define UART_CHECK_TXE(__handle__) (((__handle__)->ISR) & (uint32_t)USART_ISR_TXE)  
	#define UART_CHECK_RXNE(__handle__) (((__handle__)->ISR) & (uint32_t)USART_ISR_RXNE)  
	#define UART_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->ISR) & (__FLAG__)) == (__FLAG__))
	#define UART_ENABLE(__HANDLE__) ((__HANDLE__)->CR1 |=  USART_CR1_UE)
	#define UART_DISABLE(__HANDLE__) ((__HANDLE__)->CR1 &= (~USART_CR1_UE))
	#define DMA_ENABLE(__HANDLE__)      ((__HANDLE__)->CR |=  DMA_SxCR_EN)
	#define DMA_DISABLE(__HANDLE__)     ((__HANDLE__)->CR &=  ~DMA_SxCR_EN);while(((__HANDLE__)->CR &  DMA_SxCR_EN)!=0)
	#define PULL_UART_BSY(__HANDLE__) while(!(UART_GET_FLAG((__HANDLE__),UART_FLAG_TXE)));
#endif
typedef struct 
{
	uint16_t state;
	uint16_t config;
	dataMGR RX_MGR;
	dataMGR TX_MGR;
	IRQn_Type IRQn;
	UART_HandleTypeDef *huart;
	uint16_t DMA_bank_in;
	uint16_t DMA_bank_out;
	int16_t DMA_BankPend;
	uint16_t DMA_TotalBanks;
	uint16_t DMA_TransSize;
}CE32_IONCOM_Handle;

#define IONCOM_STATE_IDLE  			0x0001
#define IONCOM_STATE_TXON  			0x0002
#define IONCOM_STATE_SLEEP 			0x0004
#define IONCOM_STATE_CONNECTED 	0x0008
#define IONCOM_STATE_ERROR  		0x0010
#define IONCOM_STATE_RXON				0x0020
#define IONCOM_STATE_STOP 			0x0040

#define IONCOM_CONFIG_RXDMA 		0x0001
#define IONCOM_CONFIG_TXDMA 		0x0002

int 	CE32_Ioncom_Setting(CE32_IONCOM_Handle *handle,char* RX_buf,uint32_t RX_size,char* TX_buf, uint32_t TX_size);
int 	CE32_Ioncom_Init(CE32_IONCOM_Handle *handle,char* RX_buf,uint32_t RX_size,char* TX_buf, uint32_t TX_size);
void 	CE32_Ioncom_RXISR(CE32_IONCOM_Handle *handle);
void 	CE32_Ioncom_UartSend(UART_HandleTypeDef *huart,char * buf);
void 	CE32_Ioncom_UART_TRANSMIT_DMA(CE32_IONCOM_Handle *handle,uint8_t *data,uint16_t size,int16_t sig1,int32_t sig2);
void 	CE32_Ioncom_UART_RECEIVE_DMA(CE32_IONCOM_Handle *handle,uint8_t *data,uint16_t size);
void 	CE32_Ioncom_BlockTransmit(CE32_IONCOM_Handle *handle,char * buf,uint32_t size);

__forceinline void UART_IONCOM_Bank_Reset(CE32_IONCOM_Handle *handle)
{
	handle->DMA_bank_in=0;
	handle->DMA_BankPend=0;
	handle->DMA_bank_out=0;
}

__forceinline void UART_IONCOM_Bank_EnqueueBank(CE32_IONCOM_Handle *handle)
{
	handle->DMA_bank_in++;
	handle->DMA_BankPend++;
	if(handle->DMA_bank_in>=handle->DMA_TotalBanks)
	{
		handle->DMA_bank_in=0;
	}
}

__forceinline void UART_IONCOM_Bank_Reset_DBM(CE32_IONCOM_Handle *handle) //Double buffer mode
{
	handle->huart->hdmarx->Instance->CR&=~DMA_SxCR_CT;
	handle->huart->hdmarx->Instance->M0AR = (uint32_t)handle->RX_MGR.dataPtr + handle->DMA_bank_in*(handle->DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer
	UART_IONCOM_Bank_EnqueueBank(handle);
	handle->huart->hdmarx->Instance->M1AR = (uint32_t)handle->RX_MGR.dataPtr + handle->DMA_bank_in*(handle->DMA_TransSize); //Set the DMA to be in circular mode and automatic filling the buffer

	//handle->DMA_bank_in=1;
	handle->DMA_BankPend=0;
	handle->DMA_bank_out=0;
}



__forceinline uint16_t UART_IONCOM_Bank_DequeueBank(CE32_IONCOM_Handle *handle)
{
	handle->DMA_bank_out++;
	handle->DMA_BankPend--;
	if(handle->DMA_bank_out>=handle->DMA_TotalBanks)
	{
		handle->DMA_bank_out=0;
	}
	return handle->DMA_bank_out;
}

__forceinline void UART_IONCOM_DMA_Init(CE32_IONCOM_Handle *handle)
{
	PULL_UART_BSY(handle->huart->Instance)
	/* Configure DMA Channel source address */
	
	UART_DISABLE(handle->huart->Instance);
	
	if((handle->config&IONCOM_CONFIG_TXDMA)!=0)
	{
		DMA_DISABLE(handle->huart->hdmatx->Instance);
		handle->huart->hdmatx->Instance->PAR = (uint32_t)&IONCOM_HANDLE_RDR;
		SET_BIT(handle->huart->Instance->CR3, USART_CR3_DMAT); //enable UART_DMA_request TX
	}
	if((handle->config&IONCOM_CONFIG_RXDMA)!=0)
	{
		DMA_DISABLE(handle->huart->hdmarx->Instance);
		handle->huart->hdmarx->Instance->PAR  = (uint32_t)&IONCOM_HANDLE_RDR;
		SET_BIT(handle->huart->Instance->CR3, USART_CR3_DMAR); //enable UART_DMA_request RX
	}
	//UART_ENABLE(handle->huart->Instance);
}

__forceinline void UART_IONCOM_DMA_Enable(CE32_IONCOM_Handle *handle){
	PULL_UART_BSY(handle->huart->Instance)
	/* Configure DMA Channel source address */
	//SD_DMA_TX->PAR = (uint32_t)&SD_SPI->DR;
	//SD_DMA_TX->NDTR=256;
	//UART_DISABLE(HJ_UART);
	if((handle->config&IONCOM_CONFIG_TXDMA)!=0)
	{
		DMA_Base_Registers *regs_TX = (DMA_Base_Registers *)handle->huart->hdmatx->StreamBaseAddress;
		regs_TX->IFCR = DMA_FLAG_TCIF0_4 << handle->huart->hdmatx->StreamIndex;
		regs_TX->IFCR = DMA_FLAG_HTIF0_4 << handle->huart->hdmatx->StreamIndex;
		regs_TX->IFCR = DMA_FLAG_TEIF0_4 << handle->huart->hdmatx->StreamIndex;
		SET_BIT(handle->huart->Instance->CR3, USART_CR3_DMAT); //enable UART_DMA_request
		DMA_ENABLE(handle->huart->hdmatx->Instance);
		//handle->huart->hdmatx->Instance->CR|=DMA_IT_TC;			//Enable UART DMA TX interrupt
	}
	
	if((handle->config&IONCOM_CONFIG_RXDMA)!=0)
	{
		DMA_Base_Registers *regs_RX = (DMA_Base_Registers *)handle->huart->hdmarx->StreamBaseAddress;
		regs_RX->IFCR = DMA_FLAG_TCIF0_4 << handle->huart->hdmarx->StreamIndex;
		regs_RX->IFCR = DMA_FLAG_HTIF0_4 << handle->huart->hdmarx->StreamIndex;
		regs_RX->IFCR = DMA_FLAG_TEIF0_4 << handle->huart->hdmarx->StreamIndex;
		SET_BIT(handle->huart->Instance->CR3, USART_CR3_DMAR); //enable UART_DMA_request
		DMA_ENABLE(handle->huart->hdmarx->Instance);
		//handle->huart->hdmarx->Instance->CR|=DMA_IT_TC;			//Enable UART DMA RX interrupt
	}
	//UART_ENABLE(HJ_UART);
}

__forceinline void UART_IONCOM_DMA_Disable(CE32_IONCOM_Handle *handle)
{
	//PULL_UART_BSY(handle->huart->Instance)
	DMA_DISABLE(handle->huart->hdmatx->Instance);
	UART_DISABLE(handle->huart->Instance);
	CLEAR_BIT(handle->huart->Instance->CR3, USART_CR3_DMAT); //disable spi_DMA_request
	UART_ENABLE(handle->huart->Instance);
}


__forceinline void MSG_Enqueue(uint32_t* ptr,uint8_t* buf,uint8_t data)
{
	(*ptr)++;
	(*ptr)&=HJ_CMD_BUFSIZE-1;
	(*(buf+*ptr))=data;
}

#endif


