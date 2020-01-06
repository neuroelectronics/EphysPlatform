#include "CE32_HJ580.h"
#include "CE32_macro.h"
#include "dataMGR.h"
#include <string.h>

int HJ_WaitCmdResp(struct CE32_HJ580_Handle *handle){
	handle->state|=HJ580_STATE_WAITRESP;
	int cnt=0;
	while((handle->state&HJ580_STATE_CMD_RESP_PEND)==0)
	{
		if(cnt++>20000)
		{
			handle->state|=HJ580_STATE_ERROR;
			return 1;
		}
	}
	handle->state&=~HJ580_STATE_WAITRESP;
	return 0;
}
int CE32_HJ_Init_device(UART_HandleTypeDef *huart)
{
	//make sure the UART and DMA are configured as below
	
  //huart->Init.BaudRate = 19200;
  //huart->Init.WordLength = UART_WORDLENGTH_8B;
  //huart->Init.StopBits = UART_STOPBITS_1;
  //huart->Init.Parity = UART_PARITY_NONE;
  //huart->Init.Mode = UART_MODE_TX_RX;
  //huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//	huart->Instance=HJ_UART;
	 /* USART_RX Init */
    huart->hdmarx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    huart->hdmarx->Init.PeriphInc = DMA_PINC_DISABLE;
    huart->hdmarx->Init.MemInc = DMA_MINC_ENABLE;
    huart->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    huart->hdmarx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    huart->hdmarx->Init.Mode = DMA_NORMAL;
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
    HAL_NVIC_SetPriority(HJ_UART_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(HJ_UART_IRQn);
		
		//ENABLE UART
		__HAL_UART_ENABLE(huart);
		SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE); //ENABLE RX interrupr
		CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE); //CLEAR TX interrupr
		huart->RxState&=~HAL_UART_STATE_BUSY_RX; //clear RX flag to be ready for the data receiption
		return 0;
}
int CE32_HJ_Init_Slave(struct CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size)
{
	//set up dataManager
	handle->state=0;
#ifndef __HJ_SIMPLE
	PIN_SET(HJ_RST);		//RESET DEVICE
	PIN_SET(HJ_ROLE);		//Set high to enter slave mode
#endif
	handle->state&=~(HJ580_STATE_MASTER);
#ifndef __HJ_2WIREMODE
	if(CE32_HJ_Setting(handle,RX_buf,RX_size,TX_buf,TX_size)!=0)
	{
		return 1;
	}
#else
	CE32_HJ_Setting(handle,RX_buf,RX_size,TX_buf,TX_size);
	handle->state&=~(HJ580_STATE_CONFIG);
#endif
	handle->state|=HJ580_STATE_TXON;
	return 0;
}

int CE32_HJ_Init_Master(struct CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size)
{
	//set up dataManager
	handle->state=0;
#ifndef __HJ_SIMPLE
	PIN_SET(HJ_RST);		//RESET DEVICE
	PIN_RESET(HJ_ROLE);		//Set low to enter master mode
#endif
	handle->state|=HJ580_STATE_MASTER;
	if(CE32_HJ_Setting(handle,RX_buf,RX_size,TX_buf,TX_size)!=0)
	{
		return 1;
	}
	handle->state|=HJ580_STATE_TXON|HJ580_STATE_MASTER;
	return 0;
}
int CE32_HJ_Setting(struct CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size)
{
	dataMGR_init_DMA(&handle->RX_MGR,RX_buf,RX_size,(__IO uint32_t*)__NDTR_ADDR(handle->huart->hdmarx),(__IO uint32_t*)__NDTR_ADDR(handle->huart->hdmatx));
	dataMGR_init_DMA(&handle->TX_MGR,TX_buf,TX_size,(__IO uint32_t*)__NDTR_ADDR(handle->huart->hdmarx),(__IO uint32_t*)__NDTR_ADDR(handle->huart->hdmatx));
#ifndef __HJ_SIMPLE
	PIN_RESET(HJ_RST);	//Set low to disable resetting
	HAL_Delay(100);
#endif 
	PIN_RESET(HJ_CONFIG); 	//Set low to start set-up
	HAL_Delay(20);
	handle->state|=HJ580_STATE_CONFIG;
	PIN_RESET(HJ_WAKE);	//Set low to exit sleep mode
	HAL_Delay(20);
		
	CE32_HJ_Init_device(handle->huart);
	
//	HJ_UartSend(handle->huart,"<MNAME>\0");
//	HAL_Delay(100);
//	HJ_WaitCmdResp(handle);
//	memcpy(handle->DeviceName,handle->cmd_buf,handle->cmd_ptr); 
	
	HJ_UartSend(handle->huart,"<MAC>\0");
	HAL_Delay(100);
	if(HJ_WaitCmdResp(handle)==1)
	{
		return 1;
	}
	strcat((char*)handle->cmd_buf,"\0");
	memcpy(handle->DeviceMAC,handle->cmd_buf,handle->cmd_ptr_in); 
	
	//ADD MAC to adv data
	for(int i=0;i<6;i++)
	{
		handle->ADVdata[2*i]=handle->DeviceMAC[i];
		handle->ADVdata[2*i+1]=handle->DeviceMAC[i]; //It's a bug in HJ580, we have to do this twices
	}
	
	HJ_UartSend(handle->huart,"<NAME\0");
	HJ_UartSend_LimLen(handle->huart,(char*)handle->DeviceName,36);
	HJ_UartSend(handle->huart,">\0");
	HAL_Delay(100);
	HJ_WaitCmdResp(handle);
	

	HJ_UartSend(handle->huart,"<ADVDATA\0");
	HJ_UartSend_LimLen(handle->huart,(char*)handle->ADVdata,44);
	HJ_UartSend(handle->huart,">\0");
	HAL_Delay(100);
	HJ_WaitCmdResp(handle);
	
	
	if(handle->BondMAC[0]!=0)
	{
		HJ_UartSend(handle->huart,"<BONDMAC\0");
		HJ_UartSend_LimLen(handle->huart,(char*)handle->BondMAC,6);
		HJ_UartSend(handle->huart,">");
		HAL_Delay(100);
   	HJ_WaitCmdResp(handle);
	
	}
	
	//QUIT config mode and enter transmit mode
	PIN_SET(HJ_CONFIG);
	PIN_RESET(HJ_WAKE);
	handle->state&=~(HJ580_STATE_CONFIG);
	return 0;
}


void HJ_UartSend(UART_HandleTypeDef *huart,char * buf)
{ 
	char i=0;
	while (1)
	{ if (buf[i]!=0) 
		{
			HJ_UART_DR(HJ_UART) = buf[i];
			while((__HAL_UART_GET_FLAG(huart, (UART_FLAG_TXE) ) ? SET : RESET) == RESET){};
			i++;
		} 
		else return; 
	} 
}

void HJ_UartSend_LimLen(UART_HandleTypeDef *huart,char * buf,int maxLen)
{ 
	char i=0;
	for(int i=0;i<maxLen;i++)
	{ if (buf[i]!=0) 
		{
			HJ_UART_DR(HJ_UART) = buf[i];
			while((__HAL_UART_GET_FLAG(huart, (UART_FLAG_TXE) ) ? SET : RESET) == RESET){};
			i++;
		} 
		else return; 
	} 
}
void CE32_HJ_RXISR(struct CE32_HJ580_Handle *handle)
{
	//TXE and RXE are automatically set and reset by hardware
	//Just put the ISR based on the interrupt flag here
	if(UART_CHECK_RXNE!=0)
	{
		if((handle->state&HJ580_STATE_CONFIG)!=0) //check if the received packet is command or data
		{
			char data=HJ_UART->DR&0xff; //read data
			if((handle->state&HJ580_STATE_CMDRESP)==0) //if not in command receiving mode
			{
				if(data=='<')
				{
					handle->state|=HJ580_STATE_CMDRESP; //Enter command mode
					handle->cmd_ptr_in=0;
				}
			}
			else
			{
				if(data=='>')
				{
					handle->state&=~HJ580_STATE_CMDRESP; //QUIT command mode
					handle->state|=HJ580_STATE_CMD_RESP_PEND; //set command pending tag
				}
				else //command body
				{
					handle->cmd_buf[handle->cmd_idx_rec][handle->cmd_ptr_in++]=data; //copy data to command buffer
				}
			}
		} 
		else //if in data mode
		{
			dataMGR_enQueue_byte(&handle->RX_MGR,HJ_UART->DR&0xff); //if buffer not empty,read one byte to RX buffer
		}
		
	}
	if((handle->state&HJ580_STATE_TXON)!=0)
	{		
		if(UART_CHECK_TXE!=0)
		{
			if(handle->TX_MGR.bufferUsed[0]>0){
				HJ_UART->DR=dataMGR_deQueue_byte(&handle->TX_MGR,0); //if buffer not empty, send one byte from TX buffer
			}
			else
			{
				CLEAR_BIT(HJ_UART->CR1, USART_CR1_TXEIE); //clear TXE interrupt if buffer is empty
			}
		}
	}
}

void HJ_BlockTransmit(char * buf,uint32_t size)
{
	HJ_UART->DR = 0xdd; //send header
	while(UART_CHECK_TXE==0){};
	for(int idx=0;idx<size;idx++)
	{ 
			HJ_UART->DR = *(buf+idx);
			while(UART_CHECK_TXE==0){};
	} 
}

void CE32_UART_TRANSMIT_DMA(uint8_t *data,uint16_t size,int16_t sig1,int32_t sig2){
	UART_DMA_Disable();
	while(UART_CHECK_TXE==0){};
	HJ_UART->DR = 0xAD; //send header
	while(UART_CHECK_TXE==0){};
	HJ_UART->DR = 0xAD; //send header
	while(UART_CHECK_TXE==0){};
	HJ_UART->DR = (sig1&0x00ff); 			//send header
	while(UART_CHECK_TXE==0){};
	HJ_UART->DR = (sig1&0xff00)>>8; //send header
		
	while(UART_CHECK_TXE==0){};
	HJ_UART->DR = (sig2&0x000000ff); 			//send header	
  while(UART_CHECK_TXE==0){};
	HJ_UART->DR = (sig2&0x0000ff00)>>8; //send header
	while(UART_CHECK_TXE==0){};
	HJ_UART->DR = (sig2&0x00ff0000)>>16; 			//send header
	while(UART_CHECK_TXE==0){};
	HJ_UART->DR = (sig2&0xff000000)>>24; //send header
	
	HJ_DMA_TX->M0AR=(uint32_t)data;
	HJ_DMA_TX->NDTR=size;
	HJ_DMA_TX->CR|=DMA_IT_TC;
	UART_DMA_Enable();
}
