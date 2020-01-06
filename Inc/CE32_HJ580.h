#include "stm32f7xx_hal.h"
#include "dataMGR.h"
#include "CE32_macro.h"
#include "main.h"

#ifndef __CE32_HJ580
#define __CE32_HJ580

//Define the following part accroding to the project setup
#define HJ_UART UART7
#define HJ_UART_IRQn UART7_IRQn
#define HJ_DMA_TX DMA1_Stream1

//#ifndef HJ_CONFIG_GPIO_Port
//	#define HJ_CONFIG_GPIO_Port GPIOA
//	#define HJ_CONFIG_Pin  GPIO_PIN_14
//#endif

//#ifndef HJ_WAKE_GPIO_Port
//	#define HJ_WAKE_GPIO_Port GPIOA
//	#define HJ_WAKE_Pin  GPIO_PIN_13
//#endif

//Define __HJ_SIMPLE if no pin is connected to role
#define __HJ_SIMPLE
#define __HJ_2WIREMODE

#define HJ_CMD_BUFSIZE 512
#define HJ_CMD_SEQ 4

#ifdef __STM32F4xx_HAL_H
	#define __NDTR_ADDR(__handle__) ((__handle__)->Instance->NDTR)
	#define HJ_UART_DR(__handle__) (__handle__->DR)
	#define UART_CHECK_TXE ((HJ_UART->SR) & (uint32_t)USART_SR_TXE)  
	#define UART_CHECK_RXNE ((HJ_UART->SR) & (uint32_t)USART_SR_RXNE)  
#endif


#if defined(__STM32L4xx_HAL_H) || defined(__STM32F3xx_HAL_H)
	#define __NDTR_ADDR(__handle__) ((__handle__)->Instance->CNDTR)
	#define HJ_UART_DR(__handle__) (__handle__->TDR)
	#define UART_CHECK_TXE (READ_REG(HJ_UART->ISR)&(uint32_t)(USART_ISR_TXE))
	#define UART_CHECK_RXNE (READ_REG(HJ_UART->ISR)&(uint32_t)(USART_ISR_RXNE))
#endif
struct CE32_HJ580_Handle
{
	uint16_t state;
	char *DeviceName;
	char *DeviceMAC;
	char *PeerMAC;
	char *BondMAC;
	char *ADVdata;
	
	uint8_t cmd_buf[HJ_CMD_SEQ][HJ_CMD_BUFSIZE];	//Incoming command buffer
	uint32_t cmd_len[HJ_CMD_SEQ];				//Incoming command length
	uint32_t cmd_idx_proc;								//Processing command idx
	uint32_t cmd_idx_rec;								//Incoming command idx
	uint32_t cmd_ptr_in;									//Incoming command ptr in Buffer
	uint32_t cmd_ptr_in_backup;					//Incoming command starting ptr in the recent commmand
	uint32_t cmd_pending;								//Count for pending commands
	
	dataMGR RX_MGR;
	dataMGR TX_MGR;
	UART_HandleTypeDef *huart;
};

#define HJ580_STATE_IDLE  		0x0001
#define HJ580_STATE_TXON  		0x0002
#define HJ580_STATE_SLEEP 		0x0004
#define HJ580_STATE_CONFIG 		0x0008
#define HJ580_STATE_CONNECTED 0x0010
#define HJ580_STATE_ERROR  		0x0020
#define HJ580_STATE_RXON			0x0040
#define HJ580_STATE_MASTER		0x0080
#define HJ580_STATE_STOP 			0x0100
#define HJ580_STATE_CMDRESP		0x0200
#define HJ580_STATE_CMD_RESP_PEND	0x0400
#define HJ580_STATE_WAITRESP  0x0800
#define HJ580_STATE_CE32_CMD  0x1000
#define HJ580_STATE_CE32_CMD_PEND  0x2000

#define UART_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->ISR) & (__FLAG__)) == (__FLAG__))
#define UART_ENABLE(__HANDLE__) ((__HANDLE__)->CR1 |=  USART_CR1_UE)
#define UART_DISABLE(__HANDLE__) ((__HANDLE__)->CR1 &= (~USART_CR1_UE))
#define DMA_ENABLE(__HANDLE__)      ((__HANDLE__)->CR |=  DMA_SxCR_EN)
#define DMA_DISABLE(__HANDLE__)     ((__HANDLE__)->CR &=  ~DMA_SxCR_EN)
#define PULL_UART_BSY while(!(UART_GET_FLAG(HJ_UART,UART_FLAG_TXE)));

int HJ_WaitCmdResp(struct CE32_HJ580_Handle *handle);
//void CE32_HJ_Init_device(UART_HandleTypeDef *huart);
int CE32_HJ_Setting(struct CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size);
int CE32_HJ_Init_Slave(struct CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size);
int CE32_HJ_Init_Master(struct CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size);
void CE32_HJ_RXISR(struct CE32_HJ580_Handle *handle);
void HJ_UartSend(UART_HandleTypeDef *huart,char * buf);
void HJ_UartSend_LimLen(UART_HandleTypeDef *huart,char * buf,int maxLen);
void CE32_UART_TRANSMIT_DMA(uint8_t *data,uint16_t size,int16_t sig1,int32_t sig2);
void HJ_BlockTransmit(char * buf,uint32_t size);

__forceinline void UART_DMA_Init(void){
	PULL_UART_BSY
	/* Configure DMA Channel source address */
	HJ_DMA_TX->PAR = (uint32_t)&HJ_UART->TDR;
	UART_DISABLE(HJ_UART);
	SET_BIT(HJ_UART->CR3, USART_CR3_DMAT); //enable UART_DMA_request
	//UART_ENABLE(HJ_UART);
}

__forceinline void UART_DMA_Enable(void){
	PULL_UART_BSY
	/* Configure DMA Channel source address */
	//SD_DMA_TX->PAR = (uint32_t)&SD_SPI->DR;
	//SD_DMA_TX->NDTR=256;
	//UART_DISABLE(HJ_UART);
	SET_BIT(HJ_UART->CR3, USART_CR3_DMAT); //enable UART_DMA_request
	DMA_ENABLE(HJ_DMA_TX);
	//UART_ENABLE(HJ_UART);
}

__forceinline void UART_DMA_Disable(void)
{
	PULL_UART_BSY
	DMA_DISABLE(HJ_DMA_TX);
	UART_DISABLE(HJ_UART);
	CLEAR_BIT(HJ_UART->CR3, USART_CR3_DMAT); //disable spi_DMA_request
	UART_ENABLE(HJ_UART);
}

__forceinline void MSG_CMD_INC(uint32_t* cmd_ptr)
{
	(*cmd_ptr)++;
	(*cmd_ptr)&=HJ_CMD_SEQ-1;
}

__forceinline void MSG_Enqueue(uint32_t* ptr,uint8_t* buf,uint8_t data)
{
	(*ptr)++;
	(*ptr)&=HJ_CMD_BUFSIZE-1;
	(*(buf+*ptr))=data;
}

#endif


