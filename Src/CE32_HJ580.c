#include "CE32_HJ580.h"
#include "CE32_macro.h"
#include "dataMGR.h"
#include <string.h>
#include <stdio.h>

int HJ_WaitCmdResp(CE32_HJ580_Handle *handle){
	handle->state|=HJ580_STATE_WAITRESP;
	int cnt=0;
	int time_stamp=HAL_GetTick();
	while((handle->Config_CMD_TX.cmd_pending)>0);//wait command to be sent
	while((handle->Config_CMD_RX.cmd_pending)==0)
	{
		if(HAL_GetTick()-time_stamp>200)
		{
			handle->state|=HJ580_STATE_ERROR;
			handle->state&=~HJ580_STATE_WAITRESP;
			return 1;
		}
	}
	handle->state&=~(HJ580_STATE_WAITRESP|HJ580_STATE_CMD_RESP_PEND); 
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
int CE32_HJ_Init_Slave(CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size)
{
	//set up dataManager
	handle->state=0;
#ifndef __HJ_SIMPLE
	PIN_SET(HJ_RST);		//RESET DEVICE
	PIN_SET(HJ_ROLE);		//Set High to enter slave mode
	PIN_RESET(HJ_RST); //SET LOW RESET PIN 
#endif
	handle->state&=~(HJ580_STATE_MASTER);
	CE32_UART_Init_unconfigState(handle->COMM.huart); //try unconfiged mode first
	if(CE32_HJ_Setting(handle,RX_buf,RX_size,TX_buf,TX_size)!=0)
	{
		CE32_UART_Init_configedState(handle->COMM.huart); //try configed mode then
		if(CE32_HJ_Setting(handle,RX_buf,RX_size,TX_buf,TX_size)!=0)
		{
			CE32_UART_Init_unconfigState(handle->COMM.huart); //enter configured state
			return 1;
		}
	}
	CE32_UART_Init_unconfigState(handle->COMM.huart); //enter unconfigured state
	handle->state|=HJ580_STATE_TXON;
	return 0;
}

int CE32_HJ_Init_Master(CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size)
{
	//set up dataManager
	handle->state=0;
#ifndef __HJ_SIMPLE
	PIN_SET(HJ_RST);		//RESET DEVICE
	PIN_RESET(HJ_ROLE);		//Set low to enter master mode
	PIN_RESET(HJ_RST); //SET LOW RESET PIN 
#endif
	handle->state|=HJ580_STATE_MASTER;
	if(CE32_HJ_Setting(handle,RX_buf,RX_size,TX_buf,TX_size)!=0)
	{
		return 1;
	}
	handle->state|=HJ580_STATE_TXON|HJ580_STATE_MASTER;
	return 0;
}
int CE32_HJ_Setting(CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size)
{
	dataMGR_init_DMA(&handle->RX_MGR,RX_buf,RX_size,(__IO uint32_t*)__NDTR_ADDR(handle->COMM.huart->hdmarx),(__IO uint32_t*)__NDTR_ADDR(handle->COMM.huart->hdmatx));
	dataMGR_init_DMA(&handle->TX_MGR,TX_buf,TX_size,(__IO uint32_t*)__NDTR_ADDR(handle->COMM.huart->hdmarx),(__IO uint32_t*)__NDTR_ADDR(handle->COMM.huart->hdmatx));
#ifndef __HJ_SIMPLE
	PIN_RESET(HJ_RST);	//Set low to disable resetting
	HAL_Delay(100);
#endif 
	HJ580_EnterConfigMode(handle);
	PIN_RESET(HJ_WAKE);	//Set low to exit sleep mode
	HAL_Delay(20);
	CE32_HJ_Init_device(handle->COMM.huart);
	
//	HJ_UartSend(handle->huart,"<MNAME>\0");
//	HAL_Delay(100);
//	HJ_WaitCmdResp(handle);
//	memcpy(handle->DeviceName,handle->cmd_buf,handle->cmd_ptr); 
#ifdef __HJ_SIMPLE
	char* cmd="<RESET>\0";
	CE32_COMMAND_EnqueueCmd_MEMCopy(&handle->Config_CMD_TX,(uint8_t*) cmd,strlen(cmd));
	CE32_HJ580_TX_Start(handle);
	HAL_Delay(500);
#endif
	char* cmd1="<MAC>\0";
	CE32_COMMAND_EnqueueCmd_MEMCopy(&handle->Config_CMD_TX,(uint8_t*) cmd1,strlen(cmd1));
	CE32_HJ580_TX_Start(handle);
	if(HJ_WaitCmdResp(handle)==1)
	{
		HJ580_ExitConfigMode(handle);
		return 1;
	}
	
	
	uint8_t* dataptr;
	uint32_t dataLen;
	CE32_COMMAND_DequeueCmd(&handle->Config_CMD_RX,&dataptr,&dataLen);
	memcpy(handle->DeviceMAC,dataptr,dataLen); 
	strcat((char*)handle->DeviceMAC,"\0");
	//ADD MAC to adv data
	for(int i=0;i<6;i++)
	{
		handle->ADVdata[2*i]=handle->DeviceMAC[i];
		handle->ADVdata[2*i+1]=handle->DeviceMAC[i]; //It's a bug in HJ580, we have to do this twices
	}
	char cmd2[50];
	strcat(cmd2,"<NAME\0");
	strncat(cmd2,(char*)handle->DeviceName,36);
	strcat(cmd2,">\0");
	CE32_COMMAND_EnqueueCmd_MEMCopy(&handle->Config_CMD_TX,(uint8_t*) cmd2,strlen(cmd2));
	CE32_HJ580_TX_Start(handle);
	HJ_WaitCmdResp(handle);
	CE32_COMMAND_DequeueCmd(&handle->Config_CMD_RX,NULL,NULL);

//	HJ_UartSend(handle->COMM.huart,"<ADVDATA\0");
//	HJ_UartSend_LimLen(handle->COMM.huart,(char*)handle->ADVdata,44);
//	HJ_UartSend(handle->COMM.huart,">\0");
//	HAL_Delay(100);
//	HJ_WaitCmdResp(handle);
	
	
	if(handle->BondMAC[0]!=0)
	{
		char cmd_b[40];
		strcat(cmd_b,"<BONDMAC ");
		strncat(cmd_b,(char*)handle->BondMAC,6);
		strcat(cmd_b,"> ");
		CE32_COMMAND_EnqueueCmd_MEMCopy(&handle->Config_CMD_TX,(uint8_t*) cmd_b,strlen(cmd_b));
		CE32_HJ580_TX_Start(handle);
   	HJ_WaitCmdResp(handle);
		CE32_COMMAND_DequeueCmd(&handle->Config_CMD_RX,NULL,NULL);
	}
//	char baud_char[11];
//	sprintf(baud_char,"%ld",(long)handle->baud); 
//	HJ_UartSend(handle->COMM.huart,"<BAUD\0");	//Set baudrate
//	HJ_UartSend_LimLen(handle->COMM.huart,(char*)handle->baud,7);
//	HJ_UartSend(handle->COMM.huart,">\0");
//	HAL_Delay(100);
//	HJ_WaitCmdResp(handle);
	
//	switch(handle->parity)	//Set parity
//	{
//		case HJ580_PARITY_NA:
//			HJ_UartSend(handle->COMM.huart,"<PNO>\0");
//			break;
//		case HJ580_PARITY_EVEN:
//			HJ_UartSend(handle->COMM.huart,"<PEVEN>\0");
//			break;
//		case HJ580_PARITY_ODD:
//			HJ_UartSend(handle->COMM.huart,"<PODD>\0");
//			break;
//	}
//	HAL_Delay(100);
//	HJ_WaitCmdResp(handle);
	
//	switch(handle->stop_bit)	//Set stopbit
//	{
//		case HJ580_STOPBIT_1:
//			HJ_UartSend(handle->COMM.huart,"<1SB>\0");
//			break;
//		case HJ580_STOPBIT_2:
//			HJ_UartSend(handle->COMM.huart,"<2SB>\0");
//			break;
//	}
//	HAL_Delay(100);
//	HJ_WaitCmdResp(handle);
	
	//	Read bonded MAC

//	HJ_UartSend(handle->COMM.huart,"<BONDMAC123456>\0");
//	HAL_Delay(100);
//	HJ_WaitCmdResp(handle);
//	
	char* cmd_dis="<DISBOND>\0";
	CE32_COMMAND_EnqueueCmd_MEMCopy(&handle->Config_CMD_TX,(uint8_t*) cmd_dis,strlen(cmd_dis));
	CE32_HJ580_TX_Start(handle);
	HJ_WaitCmdResp(handle);
	CE32_COMMAND_DequeueCmd(&handle->Config_CMD_RX,NULL,NULL);
	//QUIT config mode and enter transmit mode
	PIN_SET(HJ_CONFIG);
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


void HJ_BlockTransmit(char * buf,uint32_t size)
{
	HJ_UART_DR(HJ_UART) = 0xdd; //send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	for(int idx=0;idx<size;idx++)
	{ 
			HJ_UART_DR(HJ_UART) = *(buf+idx);
			while(UART_CHECK_TXE(HJ_UART)==0){};
	} 
}

void CE32_UART_TRANSMIT_PREV_DMA(uint8_t *data,uint16_t size,int16_t sig1,int32_t sig2,uint16_t* sig3){
#ifdef __PROFILE
	Profile_CNT[4]=DWT->CYCCNT;
#endif
	UART_DMA_Disable();
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = 0xAD; //send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = 0xAD; //send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig1&0x00ff); 			//send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig1&0xff00)>>8; //send header
		
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig2&0x000000ff); 			//send header	
  while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig2&0x0000ff00)>>8; //send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig2&0x00ff0000)>>16; 			//send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig2&0xff000000)>>24; //send header
		
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (*sig3&0x00ff); 			//send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (*sig3&0xff00)>>8; //send header
	*sig3=0;//clear report sig
	HJ_DMA_TX->M0AR=(uint32_t)data;
	HJ_DMA_TX->NDTR=size;
	HJ_DMA_TX->CR|=DMA_IT_TC;
	UART_DMA_Enable();
}

void CE32_UART_TRANSMIT_IT(UART_HandleTypeDef* huart,uint8_t *data,uint16_t size,int16_t sig1,int32_t sig2,uint16_t* sig3){
#ifdef __PROFILE
	Profile_CNT[4]=DWT->CYCCNT;
#endif
	HJ_UART->CR1|=USART_CR1_UE|USART_CR1_TE; //ENable UART
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = 0xAD; //send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = 0xAD; //send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig1&0x00ff); 			//send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig1&0xff00)>>8; //send header
		
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig2&0x000000ff); 			//send header	
  while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig2&0x0000ff00)>>8; //send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig2&0x00ff0000)>>16; 			//send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (sig2&0xff000000)>>24; //send header
		
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (*sig3&0x00ff); 			//send header
	while(UART_CHECK_TXE(HJ_UART)==0){};
	HJ_UART_DR(HJ_UART) = (*sig3&0xff00)>>8; //send header
	*sig3=0;// reset report signal
	huart->pTxBuffPtr=data;
	huart->TxXferCount=size;
	while((HJ_UART->CR1&USART_CR1_TXEIE)==0)
	{
		HJ_UART->CR1|=USART_CR1_TXEIE;//Enable TX ISR
	}
}

void CE32_UART_TRANSMIT_VAR_HEADER_DMA(uint8_t *header,uint16_t sizeHeader, uint8_t *data,uint16_t size){
	UART_DMA_Disable();
	for(int i=0;i<sizeHeader;i++)
	{
		while(UART_CHECK_TXE(HJ_UART)==0){};
		HJ_UART_DR(HJ_UART) = header[i]; //send header
	}
	HJ_DMA_TX->M0AR=(uint32_t)data;
	HJ_DMA_TX->NDTR=size;
	HJ_DMA_TX->CR|=DMA_IT_TC;
	UART_DMA_Enable();
}

void CE32_UART_Init_unconfigState(UART_HandleTypeDef *huart){

  huart->Instance = USART1;
  huart->Init.BaudRate = 19200;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(huart) != HAL_OK)
  {
    Error_Handler();
  }
}

void CE32_UART_Init_configedState(UART_HandleTypeDef *huart){

  huart->Instance = USART1;
  huart->Init.BaudRate = 19200;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_EVEN;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(huart) != HAL_OK)
  {
    Error_Handler();
  }
}

int  CE32_HJ_HandShake(CE32_HJ580_Handle *handle)
{
	if(CE32_HJ_CHECK_CONN)
	{
		uint8_t cmd[3]={'<',0x80,'>'};
		CE32_INTERCOM_TX_EnqueueCmd((CE32_INTERCOM_Handle*)&handle,cmd,sizeof(cmd));  //Send handshake signal, response should be send back and processed in information loop
	}
	return 0;
}

int CE32_HJ_READPEERINFO(CE32_HJ580_Handle *handle)
{
	int resp=-1;
	if(CE32_HJ_CHECK_CONN)
	{
		WAIT_TILL_TXE;
		PIN_RESET(HJ_CONFIG); //Set to configure mode
		HAL_Delay(10);
		INTERCOM_STATE_END_RX_RECEIVE((CE32_INTERCOM_Handle *)handle); //Reset RX state
		handle->state|=HJ580_STATE_CONFIG; //Set configure flag
		char* cmd="<PEERMAC>\0";
		CE32_COMMAND_EnqueueCmd_MEMCopy(&handle->Config_CMD_TX,(uint8_t*) cmd,strlen(cmd));
		CE32_HJ580_TX_Start(handle);
		HJ_WaitCmdResp(handle);
		uint8_t* receive_ptr;
		uint32_t receive_len;
		CE32_COMMAND_DequeueCmd(&handle->Config_CMD_RX,&receive_ptr,&receive_len);
		if(receive_len==7)
		{
			memcpy(handle->PeerMAC,&receive_ptr[1],receive_len-1); 
			memcpy(handle->DEV.MAC,&receive_ptr[1],receive_len-1);
			resp=0;
		}			
		PIN_SET(HJ_CONFIG); //Exit configure mode
		handle->state&=~HJ580_STATE_CONFIG; //Set configure flag
		
	}
	return resp;
}
int  CE32_HJ_READRSSI(CE32_HJ580_Handle *handle)
{
	int resp=0;
	if(CE32_HJ_CHECK_CONN)
	{
		if((handle->DEV.state&CE32_STATE_PREV)!=0)
		{
			return -1;
//			uint8_t cmd[]="<\x41>"; //Stop Preview
//			CE32_INTERCOM_TX_EnqueueCmd((CE32_INTERCOM_Handle*)handle,cmd,sizeof(cmd));  //Send start recording signal
		}
			
		WAIT_TILL_TXE;
		PIN_RESET(HJ_CONFIG); //Set to configure mode
		//HAL_Delay(200);
		INTERCOM_STATE_END_RX_RECEIVE((CE32_INTERCOM_Handle *)handle); //Reset RX state
		handle->state|=HJ580_STATE_CONFIG; //Set configure flag
		char* cmd="<RSSI>\0";
		CE32_COMMAND_EnqueueCmd_MEMCopy(&handle->Config_CMD_TX,(uint8_t*) cmd,strlen(cmd));
		CE32_HJ580_TX_Start(handle);
		HJ_WaitCmdResp(handle);
		uint8_t* receive_ptr;
		uint32_t receive_len;
		CE32_COMMAND_DequeueCmd(&handle->Config_CMD_RX,&receive_ptr,&receive_len);
		
		if(receive_ptr[0]=='R')
		{
			handle->DEV.RSSI=receive_ptr[1];
		}
		else
		{
			resp=-1;
		}
		PIN_SET(HJ_CONFIG); //Exit configure mode
		handle->state&=~HJ580_STATE_CONFIG; //Set configure flag
//		if((handle->DEV.state&CE32_STATE_PREV)!=0)
//		{
//			uint8_t cmd[]="<\x40>";	//Continue Preview
//			CE32_INTERCOM_TX_EnqueueCmd((CE32_INTERCOM_Handle*)handle,cmd,sizeof(cmd));  //Send start recording signal
//		}
	}
	return resp;
}

int  CE32_HJ_BondMAC(CE32_HJ580_Handle *handle)
{
	int resp=-1;
	if(CE32_HJ_CHECK_CONN)
	{
		WAIT_TILL_TXE;
		PIN_RESET(HJ_CONFIG); //Set to configure mode
		HAL_Delay(100);
		uint8_t temp=HJ_UART_DR(((CE32_INTERCOM_Handle *)handle)->UART);//purge buffer
		handle->state|=HJ580_STATE_CONFIG; //Set configure flag
		
//		char cmd0[]="<PEERMAC>\0";
//		INTERCOM_STATE_END_RX_RECEIVE((CE32_INTERCOM_Handle *)handle); //Reset RX state
//		HJ_UartSend(handle->COMM.huart,cmd0);
//		HAL_Delay(100);
//		HJ_WaitCmdResp(handle);
		
		char cmd[16]="<BONDMAC000000>\0";
		memcpy(&cmd[8],handle->PeerMAC,6); 
		INTERCOM_STATE_END_RX_RECEIVE((CE32_INTERCOM_Handle *)handle); //Reset RX state
		CE32_COMMAND_EnqueueCmd_MEMCopy(&handle->Config_CMD_TX,(uint8_t*) cmd,strlen(cmd));
		CE32_HJ580_TX_Start(handle);
		HJ_WaitCmdResp(handle);
		CE32_COMMAND_DequeueCmd(&handle->Config_CMD_RX,NULL,NULL);
//		char cmd1[]="<RBMAC>\0";
//		INTERCOM_STATE_END_RX_RECEIVE((CE32_INTERCOM_Handle *)handle); //Reset RX state
//		HJ_UartSend(handle->COMM.huart,cmd1);
//		HAL_Delay(100);
//		HJ_WaitCmdResp(handle);
		
		PIN_SET(HJ_CONFIG); //Exit configure mode
		handle->state&=~HJ580_STATE_CONFIG; //Set configure flag
		HAL_Delay(100);
		resp=0;
	}
	return resp;
}

int  CE32_HJ_DisbondMAC(CE32_HJ580_Handle *handle)
{
	int resp=0;
	if(CE32_HJ_CHECK_CONN)
	{	
		WAIT_TILL_TXE;
		HJ580_EnterConfigMode(handle);
		
		INTERCOM_STATE_END_RX_RECEIVE((CE32_INTERCOM_Handle *)handle); //Reset RX state
		char* cmd="<DISBOND>\0";
		CE32_COMMAND_EnqueueCmd_MEMCopy(&handle->Config_CMD_TX,(uint8_t*) cmd,strlen(cmd));
		CE32_HJ580_TX_Start(handle);
		HJ_WaitCmdResp(handle);
		CE32_COMMAND_DequeueCmd(&handle->Config_CMD_RX,NULL,NULL);
		
		char* cmd1="<STARTSCAN>\0";
		CE32_COMMAND_EnqueueCmd_MEMCopy(&handle->Config_CMD_TX,(uint8_t*) cmd1,strlen(cmd1));
		CE32_HJ580_TX_Start(handle);
		HJ_WaitCmdResp(handle);
		CE32_COMMAND_DequeueCmd(&handle->Config_CMD_RX,NULL,NULL);
		
		HJ580_ExitConfigMode(handle);
	}
	return resp;
}

void  CE32_HJ_Upload_State(CE32_HJ580_Handle *handle)
{
	static int EXTsig_buff; //Buffer for ext signal
	if((handle->DEV.state&CE32_STATE_CONNECTED)!=0)	//Only send command when device is connected
	{
		uint8_t cmd[4]={'<',0x83,handle->DEV.EXT_SIG,'>'};
		if(EXTsig_buff!=handle->DEV.EXT_SIG)
		{
			EXTsig_buff=handle->DEV.EXT_SIG; //Update buffer
			CE32_INTERCOM_TX_EnqueueCmd((CE32_INTERCOM_Handle*)handle,cmd,sizeof(cmd));  //Update EXT signal, only when signal has changed(save bandwidth)
		}
		//	PULL_UART_BSY(handle->COMM.UART); //Wait till previous transmit compelete
		//	CE32_UART_SEND(handle->COMM.UART,0x3C);
		//	PULL_UART_BSY(handle->COMM.UART); //Wait till previous transmit compelete
		//	CE32_UART_SEND(handle->COMM.UART,0x83);
		//	PULL_UART_BSY(handle->COMM.UART); //Wait till previous transmit compelete
		//	CE32_UART_SEND(handle->COMM.UART,handle->EXT_SIG);
		//	PULL_UART_BSY(handle->COMM.UART); //Wait till previous transmit compelete
		//	CE32_UART_SEND(handle->COMM.UART,0x3E);
	}
}

int CE32_HJ580_RX_CONFIG_ISR(CE32_HJ580_Handle *handle_BLE)
{
	int resp=-1;
	CE32_INTERCOM_Handle *handle_IC=&handle_BLE->COMM;
	CE32_command* handle=&handle_BLE->Config_CMD_RX;
	if(INTERCOM_UART_CHECK_RXNEIE(handle_IC))
	{
		if(INTERCOM_CHECK_RXNE(handle_IC))
		{
			handle_IC->UART->ISR&=~USART_ISR_RXNE;   //Clear RXNE flag
		 	uint8_t data_temp=handle_IC->UART->RDR; //Read data into buffer first
			if(INTERCOM_RX_IS_ON(handle_IC)) //check if already in cmd receiveing mode
			{
				if(handle->stream_in.logging==0)//If not logging
				{
					CE32_COMMAND_Enqueue_Allocate(handle,CMD_MAXSIZE);//Decode command length, preallocate space in memory
				}
				if(CE32_COMMAND_Enqueue_BytesLeft(handle)<0)
				{
					CE32_COMMAND_Enqueue_Byte_Abort(handle);
					INTERCOM_STATE_END_RX_RECEIVE(handle_IC); //Reset flags to no cmd detected mode
					resp=-2; //Receive error, END package not detected
				}
				else
				{
					if(data_temp==INTERCOM_PKG_TAIL) //If end package detectected
					{
						handle->stream_in.len=handle->stream_in.idx;
						CE32_COMMAND_EnqueueCmd_buffered(handle); //Push command into buffer
						CE32_COMMAND_Enqueue_Byte_Abort(handle);
						INTERCOM_STATE_END_RX_RECEIVE(handle_IC); //Reset flags to no cmd detected mode
						handle_BLE->state|=HJ580_STATE_CMD_RESP_PEND; //set command pending tag
						resp=0;
					}
					else{
						if(CE32_COMMAND_Enqueue_Byte(handle,data_temp)==-1) //Enqueue data to current RX buffer
						{
							//Buffer is full
							CE32_COMMAND_Enqueue_Byte_Abort(handle);
							INTERCOM_STATE_END_RX_RECEIVE(handle_IC); //Reset flags to no cmd detected mode
							resp=-1;
						}
					}
				}
			}
			else //If recording is not started
			{
				if(data_temp==INTERCOM_PKG_HEAD)
				{
					INTERCOM_STATE_START_RX_RECEIVE(handle_IC);
					resp=0;
				}
			}
		}
	}
	
	return resp;
}

#ifdef __BLE_HOST
int CE32_HJ580_RX_ISR(CE32_HJ580_Handle *handle_BLE)
{
	int resp=-1;
	static unsigned short Rdata16b;
	static unsigned short end_cnt;
	//CE32_INTERCOM_Handle *handle_IC=&handle_BLE->COMM;
	CE32_command* handle=&handle_BLE->COMM.CMD_RX;
	if(INTERCOM_UART_CHECK_RXNEIE(((CE32_INTERCOM_Handle *)handle_BLE)))
	{
		if(INTERCOM_CHECK_RXNE(((CE32_INTERCOM_Handle *)handle_BLE)))
		{
			((CE32_INTERCOM_Handle *)handle_BLE)->UART->SR&=~USART_SR_RXNE;   //Clear RXNE flag
			uint8_t data_temp=((CE32_INTERCOM_Handle *)handle_BLE)->UART->DR; //Read data into buffer first
			if(INTERCOM_RX_IS_ON(((CE32_INTERCOM_Handle *)handle_BLE))) //check if already in cmd receiveing mode
			{
				if(handle->stream_in.logging==0)
				{
					if(CE32_INTERCOM_Incoming_CMD_Len(data_temp)>0)
					{
						CE32_COMMAND_Enqueue_Allocate(handle,CE32_INTERCOM_Incoming_CMD_Len(data_temp)); //Preallocate space
					}
					else
					{
						CE32_COMMAND_Enqueue_Byte_Abort(handle);
					}
				}
				if(CE32_COMMAND_Enqueue_Byte(handle,data_temp)==-1) //Enqueue data to current RX buffer
				{
						*((uint8_t*)(&Rdata16b)+end_cnt++)=data_temp;
						//Rdata16b=Rdata16b<<8|data_temp;
						//if(Rdata16b==0xDADA){
						if(end_cnt==2)
						{
						if(Rdata16b==0xDADA)
							{
								Rdata16b=0;
								end_cnt=0;
								CE32_INTERCOM_RX_EnqueueCmd(((CE32_INTERCOM_Handle *)handle_BLE));
								INTERCOM_STATE_END_RX_RECEIVE(((CE32_INTERCOM_Handle *)handle_BLE)); //Reset flags to no cmd detected mode
								resp=0;
							}
							else
							{
								CE32_COMMAND_Enqueue_Byte_Abort(handle);
								INTERCOM_STATE_END_RX_RECEIVE(((CE32_INTERCOM_Handle *)handle_BLE)); //Reset flags to no cmd detected mode
								resp=-2;
							}
						}
				}
			}
			else //If recording is not started
			{
				if(data_temp==0xAD)
				{
					INTERCOM_STATE_START_RX_RECEIVE(((CE32_INTERCOM_Handle *)handle_BLE));
					resp=0;
					Rdata16b=0; //reset 16B tail detector
					end_cnt=0;
				}
			}
		}
	}
	
	return resp;
}

#else
int CE32_HJ580_RX_ISR(CE32_HJ580_Handle *handle_BLE)
{
	int resp=-1;
	static unsigned short Rdata16b;
	static unsigned short end_cnt;
	//CE32_INTERCOM_Handle *handle_IC=&handle_BLE->COMM;
	CE32_command* handle=&handle_BLE->COMM.CMD_RX;
	if(INTERCOM_UART_CHECK_RXNEIE(((CE32_INTERCOM_Handle *)handle_BLE)))
	{
		if(INTERCOM_CHECK_RXNE(((CE32_INTERCOM_Handle *)handle_BLE)))
		{
			((CE32_INTERCOM_Handle *)handle_BLE)->UART->ISR&=~USART_ISR_RXNE;   //Clear RXNE flag
			uint8_t data_temp=((CE32_INTERCOM_Handle *)handle_BLE)->UART->RDR; //Read data into buffer first
			if(INTERCOM_RX_IS_ON(((CE32_INTERCOM_Handle *)handle_BLE))) //check if already in cmd receiveing mode
			{
				if(handle->stream_in.logging==0)
				{
					if(CE32_INTERCOM_Incoming_CMD_Len(data_temp)>0)
					{
						CE32_COMMAND_Enqueue_Allocate(handle,CE32_INTERCOM_Incoming_CMD_Len(data_temp)); //Preallocate space
					}
					else
					{
						CE32_COMMAND_Enqueue_Byte_Abort(handle);
					}
				}
				if(CE32_COMMAND_Enqueue_Byte(handle,data_temp)==-1) //Enqueue data to current RX buffer
				{
					if(data_temp==0x3E)
					{
						CE32_INTERCOM_RX_EnqueueCmd(((CE32_INTERCOM_Handle *)handle_BLE));
						INTERCOM_STATE_END_RX_RECEIVE(((CE32_INTERCOM_Handle *)handle_BLE)); //Reset flags to no cmd detected mode
						resp=0;
					}
					else
					{
						CE32_COMMAND_Enqueue_Byte_Abort(handle);
						INTERCOM_STATE_END_RX_RECEIVE(((CE32_INTERCOM_Handle *)handle_BLE)); //Reset flags to no cmd detected mode
						resp=-2;
					}
				}
			}
			else //If recording is not started
			{
				if(data_temp==0x3C)
				{
					INTERCOM_STATE_START_RX_RECEIVE(((CE32_INTERCOM_Handle *)handle_BLE));
					resp=0;
					Rdata16b=0; //reset 16B tail detector
					end_cnt=0;
				}
			}
		}
	}
	
	return resp;
}
#endif
	
int CE32_HJ580_RX_PREV_ISR(CE32_HJ580_Handle *handle_BLE)
{
	int resp=-1;
	static unsigned short Rdata16b;
	static unsigned short head_cnt;
	static unsigned short end_cnt;
	static unsigned short  start_pkg;
	//CE32_INTERCOM_Handle *handle_IC=&handle_BLE->COMM;
	CE32_command* handle=&handle_BLE->COMM.CMD_RX;
	if(INTERCOM_UART_CHECK_RXNEIE(((CE32_INTERCOM_Handle *)handle_BLE)))
	{
		if(INTERCOM_CHECK_RXNE(((CE32_INTERCOM_Handle *)handle_BLE)))
		{
			((CE32_INTERCOM_Handle *)handle_BLE)->UART->ISR&=~USART_ISR_RXNE;   //Clear RXNE flag
			uint8_t data_temp=((CE32_INTERCOM_Handle *)handle_BLE)->UART->RDR; //Read data into buffer first
			if(INTERCOM_RX_IS_ON(((CE32_INTERCOM_Handle *)handle_BLE))) //check if already in cmd receiveing mode
			{
				if(CE32_COMMAND_Enqueue_Byte(handle,data_temp)==-1) //Enqueue data to current RX buffer
				{
					*((uint8_t*)(&Rdata16b)+end_cnt++)=data_temp;
					Rdata16b=Rdata16b<<8|data_temp;
					if(end_cnt==2)
					{
						if(Rdata16b==0xDADA)
						{
							Rdata16b=0;
							end_cnt=0;
							CE32_INTERCOM_RX_EnqueueCmd(((CE32_INTERCOM_Handle *)handle_BLE));
							INTERCOM_STATE_END_RX_RECEIVE(((CE32_INTERCOM_Handle *)handle_BLE)); //Reset flags to no cmd detected mode
							resp=0;
						}
						else
						{
							CE32_COMMAND_Enqueue_Byte_Abort(handle);
							INTERCOM_STATE_END_RX_RECEIVE(((CE32_INTERCOM_Handle *)handle_BLE)); //Reset flags to no cmd detected mode
							resp=-2;
						}
					}
				}
			}
			else //If recording is not started
			{
				start_pkg=start_pkg<<8|data_temp;
				if(start_pkg==0xADAD)
				{
					start_pkg=0;
					INTERCOM_STATE_START_RX_RECEIVE(((CE32_INTERCOM_Handle *)handle_BLE));
					CE32_COMMAND_Enqueue_Allocate(handle,9+128);
					CE32_COMMAND_Enqueue_Byte(handle,data_temp);
					resp=0;
					Rdata16b=0; //reset 16B tail detector
					end_cnt=0;
				}
			}
		}
	}
	
	return resp;
}

int CE32_HJ580_TX_CONFIG_ISR(CE32_HJ580_Handle *handleBLE)
{
	int resp=-1;
	CE32_INTERCOM_Handle* handle_IC=(CE32_INTERCOM_Handle*) handleBLE;
	CE32_command* handle=&handleBLE->Config_CMD_TX;
	if(INTERCOM_UART_CHECK_TXEIE(handle_IC))
	{
		if(INTERCOM_CHECK_TXE(handle_IC))
		{
			if(CE32_COMMAND_GetPendingCounts(handle)>0)
			{
				uint8_t data;
				if(CE32_COMMAND_Dequeue_Byte(handle,&data)==0)
				{
					INTERCOM_TX_SEND(handle_IC,data);
					resp=0;
				}
				else
				{
					//Disable TX transmission
					INTERCOM_UART_TXEIE_DISABLE(handle_IC);
					resp=0;
				}
			}
			else
			{
				//Disable TX transmission
				INTERCOM_UART_TXEIE_DISABLE(handle_IC);
				resp=0;
			}
		}
	}
	return resp;
}



void HJ580_EnterConfigMode(CE32_HJ580_Handle *handle)
{
	PIN_RESET(HJ_CONFIG); //Exit configure mode
	handle->state|=HJ580_STATE_CONFIG; //Set configure flag
	HAL_Delay(100);
}

void HJ580_ExitConfigMode(CE32_HJ580_Handle *handle)
{
	PIN_SET(HJ_CONFIG); //Exit configure mode
	handle->state&=~HJ580_STATE_CONFIG; //Set configure flag
	HAL_Delay(100);
}

void CE32_HJ580_TX_Start(CE32_HJ580_Handle *handle)
{
	((CE32_INTERCOM_Handle*)handle)->UART->CR1|=USART_CR1_TXEIE|USART_CR1_UE|USART_CR1_TE;//Enable UART transfer
}
