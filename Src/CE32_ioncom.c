#include "CE32_ioncom.h"
#include "CE32_macro.h"
#include "dataMGR.h"
#include <string.h>


int CE32_IonCom_Init_device(UART_HandleTypeDef *huart,IRQn_Type UART_IRQn)
{
	//make sure the UART and DMA are configured as below
	
  //huart->Init.BaudRate = 19200;
  //huart->Init.WordLength = UART_WORDLENGTH_8B;
  //huart->Init.StopBits = UART_STOPBITS_1;
  //huart->Init.Parity = UART_PARITY_NONE;
  //huart->Init.Mode = UART_MODE_TX_RX;
  //huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//	huart->Instance=handle->huart->Instance;
	 /* USART_RX Init */
    huart->hdmarx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    huart->hdmarx->Init.PeriphInc = DMA_PINC_DISABLE;
    huart->hdmarx->Init.MemInc = DMA_MINC_ENABLE;
    huart->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    huart->hdmarx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    huart->hdmarx->Init.Mode = DMA_CIRCULAR;
    huart->hdmarx->Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(huart->hdmarx) != HAL_OK)
    {
      return -1;
    }

    __HAL_LINKDMA(huart,hdmarx,*huart->hdmarx);

    /* USART1_TX Init */
    huart->hdmatx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    huart->hdmatx->Init.PeriphInc = DMA_PINC_DISABLE;
    huart->hdmatx->Init.MemInc = DMA_MINC_ENABLE;
    huart->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    huart->hdmatx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    huart->hdmatx->Init.Mode = DMA_NORMAL;
    huart->hdmatx->Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(huart->hdmatx) != HAL_OK)
    {
      return -1;
    }

    __HAL_LINKDMA(huart,hdmatx,*huart->hdmatx);
		
    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(UART_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART_IRQn);
		
		//ENABLE UART
		//__HAL_UART_ENABLE(huart);
		CLEAR_BIT(huart->Instance->CR1, USART_CR1_RXNEIE); //ENABLE RX interrupr
		CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE); //CLEAR TX interrupr
		huart->RxState&=~HAL_UART_STATE_BUSY_RX; //clear RX flag to be ready for the data receiption
		return 0;
}
int CE32_Ioncom_Init(CE32_IONCOM_Handle *handle,char* RX_buf,uint32_t RX_size,char* TX_buf, uint32_t TX_size)
{
	//set up dataManager
	handle->state=0;

	if(CE32_Ioncom_Setting(handle,RX_buf,RX_size,TX_buf,TX_size)!=0)
	{
		return 1;
	}
	handle->state|=IONCOM_STATE_TXON;
	return 0;
}


int CE32_Ioncom_Setting(CE32_IONCOM_Handle *handle,char* RX_buf,uint32_t RX_size,char* TX_buf, uint32_t TX_size)
{
	if((handle->config&IONCOM_CONFIG_RXDMA)!=0)
	{
		dataMGR_init_DMA(&handle->RX_MGR,RX_buf,RX_size,(__IO uint32_t*)__NDTR_ADDR(handle->huart->hdmarx),(__IO uint32_t*)__NDTR_ADDR(handle->huart->hdmarx));
	}
	if((handle->config&IONCOM_CONFIG_TXDMA)!=0)
	{
		dataMGR_init_DMA(&handle->TX_MGR,TX_buf,TX_size,(__IO uint32_t*)__NDTR_ADDR(handle->huart->hdmatx),(__IO uint32_t*)__NDTR_ADDR(handle->huart->hdmatx));
	}
	CE32_IonCom_Init_device(handle->huart,handle->IRQn);

	return 0;
}


void CE32_Ioncom_UartSend(UART_HandleTypeDef *huart,char * buf)
{ 
	char i=0;
	while (1)
	{ if (buf[i]!=0) 
		{
		  IONCOM_UART_TDR(huart->Instance) = buf[i];
			while((__HAL_UART_GET_FLAG(huart, (UART_FLAG_TXE) ) ? SET : RESET) == RESET){};
			i++;
		} 
		else return; 
	} 
}

void CE32_Ioncom_RXISR(CE32_IONCOM_Handle *handle)
{
	//TXE and RXE are automatically set and reset by hardware
	//Just put the ISR based on the interrupt flag here
	if(UART_CHECK_RXNE(handle->huart->Instance)!=0)
	{
		dataMGR_enQueue_byte(&handle->RX_MGR,IONCOM_HANDLE_RDR&0xff); //if buffer not empty,read one byte to RX buffer	
	}
	if((handle->state&IONCOM_STATE_TXON)!=0)
	{		
		if(UART_CHECK_TXE(handle->huart->Instance)!=0)
		{
			if(handle->TX_MGR.bufferUsed[0]>0){
				IONCOM_HANDLE_TDR=dataMGR_deQueue_byte(&handle->TX_MGR,0); //if buffer not empty, send one byte from TX buffer
			}
			else
			{
				CLEAR_BIT(handle->huart->Instance->CR1, USART_CR1_TXEIE); //clear TXE interrupt if buffer is empty
			}
		}
	}
}

void CE32_Ioncom_BlockTransmit(CE32_IONCOM_Handle *handle,char * buf,uint32_t size)
{
	IONCOM_HANDLE_TDR = 0xdd; //send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	for(int idx=0;idx<size;idx++)
	{ 
			IONCOM_HANDLE_TDR = *(buf+idx);
			while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	} 
}

void CE32_Ioncom_UART_TRANSMIT_DMA(CE32_IONCOM_Handle *handle,uint8_t *data,uint16_t size,int16_t sig1,int32_t sig2){
	UART_IONCOM_DMA_Disable(handle);
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = 0xAA; //send header (AA 55 are charge balanced)
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = 0xAA; //send header (AA 55 are charge balanced)
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = 0xAA; //send header (AA 55 are charge balanced)
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = 0xAA; //send header (AA 55 are charge balanced)
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = 0x55; //send header
#ifndef MANCHESTER
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (sig1&0x00ff); 			//send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (sig1&0xff00)>>8; //send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (sig2&0x000000ff); 			//send header	
  while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (sig2&0x0000ff00)>>8; //send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (sig2&0x00ff0000)>>16; 			//send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (sig2&0xff000000)>>24; //send header
#else
	uint32_t temp;
	temp=Manchester_converter_Short(sig1);
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0x00ff); 			//send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0xff00)>>8; //send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0x00ff0000)>>16; 			//send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0xff000000)>>24; //send header
	temp=Manchester_converter_Short(sig2&0x0000ffff);
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0x000000ff); 			//send header	
  while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0x0000ff00)>>8; //send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0x00ff0000)>>16; 			//send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0xff000000)>>24; //send header
		
	temp=Manchester_converter_Short((sig2&0xffff0000)>>16);
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0x000000ff); 			//send header	
  while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0x0000ff00)>>8; //send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0x00ff0000)>>16; 			//send header
	while(UART_CHECK_TXE(handle->huart->Instance)==0){};
	IONCOM_HANDLE_TDR = (temp&0xff000000)>>24; //send header
#endif

		
	handle->huart->hdmatx->Instance->CR&=~DMA_CIRCULAR;
	handle->huart->hdmatx->Instance->CR|=DMA_MINC_ENABLE;
	handle->huart->hdmatx->Instance->CR|=DMA_IT_TC;
	handle->huart->hdmatx->Instance->M0AR=(uint32_t)data;
	handle->huart->hdmatx->Instance->NDTR=size/4;

		
	UART_IONCOM_DMA_Enable(handle);
}

void CE32_Ioncom_UART_RECEIVE_DMA(CE32_IONCOM_Handle *handle,uint8_t *data,uint16_t size)
{
	
}
