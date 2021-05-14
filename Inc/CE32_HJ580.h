#include "stm32f7xx_hal.h"
#include "dataMGR.h"
#include "CE32_macro.h"
#include "main.h"
#include "CE32_UART_INTERCOM.h"

#ifndef __CE32_HJ580
#define __CE32_HJ580

//Define the following part accroding to the project setup
#define HJ_UART UART8
#define handle_HJ_UART huart8
#define HJ_UART_IRQn UART8_IRQn
#define HJ_DMA_TX DMA2_Stream7

#ifndef HJ_CONFIG_GPIO_Port
#define HJ_CONFIG_GPIO_Port GPIOA
#define HJ_CONFIG_Pin  GPIO_PIN_14
#endif

#ifndef HJ_WAKE_GPIO_Port
#define HJ_WAKE_GPIO_Port GPIOA
#define HJ_WAKE_Pin  GPIO_PIN_13
#endif

#define __HJ_SIMPLE //if no pin is connected to role
//#define __HJ_2WIREMODE

#define HJ_CMD_BUFSIZE 1024
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

#if defined(__STM32F7xx_HAL_H)
	#define __NDTR_ADDR(__handle__) ((__handle__)->Instance->NDTR)
	#define HJ_HANDLE_RDR handle->huart->Instance->RDR
	#define HJ_UART_RDR(__handle__) (__handle__->RDR)
	#define HJ_HANDLE_TDR handle->huart->Instance->TDR
	#define HJ_UART_DR(__handle__) (__handle__->TDR)
	#define UART_CHECK_TXE(__handle__) (((__handle__)->ISR) & (uint32_t)USART_ISR_TXE)  
	#define UART_CHECK_RXNE(__handle__) (((__handle__)->ISR) & (uint32_t)USART_ISR_RXNE)  
	#define UART_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->ISR) & (__FLAG__)) == (__FLAG__))
	#define UART_ENABLE(__HANDLE__) ((__HANDLE__)->CR1 |=  USART_CR1_UE)
	#define UART_DISABLE(__HANDLE__) ((__HANDLE__)->CR1 &= (~USART_CR1_UE))
	#define DMA_ENABLE(__HANDLE__)      ((__HANDLE__)->CR |=  DMA_SxCR_EN)
	#define DMA_DISABLE(__HANDLE__)     ((__HANDLE__)->CR &=  ~DMA_SxCR_EN);while(((__HANDLE__)->CR &  DMA_SxCR_EN)!=0)
	#define PULL_UART_BSY(__HANDLE__) while(!(UART_GET_FLAG((__HANDLE__),UART_FLAG_TXE)));
#endif

#define CE32_STATE_CONNECTED 	0x0001
#define CE32_STATE_REC 				0x0002
#define CE32_STATE_PREV 			0x0004
#define CE32_STATE_STIM 			0x0008
#define CE32_STATE_DSP			 	0x0010
struct CE32_Basic_BLE_Info
{
	uint16_t state;
	int16_t RSSI;
	int16_t voltage;
	int16_t capacity;
	int16_t data_recorded;
	int16_t channel_number;
	int16_t preview_channel;
	uint16_t EXT_SIG;
	uint16_t UpdateRate;
	uint8_t MAC[6] ; 
};


#define HJ580_PARITY_NA 0
#define HJ580_PARITY_ODD 1
#define HJ580_PARITY_EVEN 2

#define HJ580_STOPBIT_1 1
#define HJ580_STOPBIT_2 2

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
#define HJ580_STATE_TXDMA  		0x4000
//Define corresponde to intan
#define HJ580_GPIO1_SIG  	0x10U			
#define HJ580_GPIO2_SIG 	0x20U
#define HJ580_GPIO3_SIG 	0x40U

typedef struct 
{
	CE32_INTERCOM_Handle COMM;
	struct CE32_Basic_BLE_Info DEV;
	uint16_t state;
	
	uint32_t baud;
	uint8_t parity;
	uint8_t stop_bit;
	
	char DeviceName[50];
	char DeviceMAC[6];
	char PeerMAC[6];
	char BondMAC[6];
	char ADVdata[50];
	
	CE32_command Config_CMD_TX,Config_CMD_RX;
	dataMGR RX_MGR;
	dataMGR TX_MGR;
}CE32_HJ580_Handle;



//#define UART_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->SR) & (__FLAG__)) == (__FLAG__))
//#define UART_ENABLE(__HANDLE__) ((__HANDLE__)->CR1 |=  USART_CR1_UE)
//#define UART_DISABLE(__HANDLE__) ((__HANDLE__)->CR1 &= (~USART_CR1_UE))
//#define DMA_ENABLE(__HANDLE__)      ((__HANDLE__)->CR |=  DMA_SxCR_EN)
//#define DMA_DISABLE(__HANDLE__)     while(((__HANDLE__)->CR &  DMA_SxCR_EN)!=0){((__HANDLE__)->CR &=  ~DMA_SxCR_EN);}
//#define PULL_UART_BSY(__HANDLE__) while(!(UART_GET_FLAG((__HANDLE__),UART_FLAG_TXE))){};

#define CE32_UART_SEND(__HANDLE__,__DATA__)  (__HANDLE__)->DR=(__DATA__);
#ifdef __HJ_SIMPLE
	#define CE32_HJ_CHECK_CONN 1
#else
	#define CE32_HJ_CHECK_CONN ((HJ_STATE_GPIO_Port->IDR & HJ_STATE_Pin) == (uint32_t)GPIO_PIN_RESET)
#endif
#define WAIT_TILL_TXE while((handle->COMM.UART->CR1&USART_CR1_TXEIE)!=0)
int HJ_WaitCmdResp(CE32_HJ580_Handle *handle);
//void CE32_HJ_Init_device(UART_HandleTypeDef *huart);
int CE32_HJ_Setting(CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size);
int CE32_HJ_Init_Slave(CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size);
int CE32_HJ_Init_Master(CE32_HJ580_Handle *handle,char* RX_buf,uint16_t RX_size,char* TX_buf, uint16_t TX_size);
void CE32_HJ_RXISR(CE32_HJ580_Handle *handle);
void HJ_UartSend(UART_HandleTypeDef *huart,char * buf);
void HJ_UartSend_LimLen(UART_HandleTypeDef *huart,char * buf,int maxLen);
void CE32_UART_TRANSMIT_PREV_DMA(uint8_t *data,uint16_t size,int16_t sig1,int32_t sig2,uint16_t* sig3);
void CE32_UART_TRANSMIT_IT(UART_HandleTypeDef* huart,uint8_t *data,uint16_t size,int16_t sig1,int32_t sig2,uint16_t* sig3);
void CE32_UART_TRANSMIT_VAR_HEADER_DMA(uint8_t *header,uint16_t sizeHeader, uint8_t *data,uint16_t size);
void HJ_BlockTransmit(char * buf,uint32_t size);
void CE32_UART_Init_unconfigState(UART_HandleTypeDef *huart);
void CE32_UART_Init_configedState(UART_HandleTypeDef *huart);
int  CE32_HJ_HandShake(CE32_HJ580_Handle *handle);
int  CE32_HJ_READPEERINFO(CE32_HJ580_Handle *handle);
int  CE32_HJ_READRSSI(CE32_HJ580_Handle *handle);
int  CE32_HJ_BondMAC(CE32_HJ580_Handle *handle);
int  CE32_HJ_DisbondMAC(CE32_HJ580_Handle *handle);
void  CE32_HJ_Upload_State(CE32_HJ580_Handle *handle);
int CE32_HJ580_RX_CONFIG_ISR(CE32_HJ580_Handle *handle_BLE);
int CE32_HJ580_RX_ISR(CE32_HJ580_Handle *handle_BLE);
int CE32_HJ580_RX_PREV_ISR(CE32_HJ580_Handle *handle_BLE);
int CE32_HJ580_TX_CONFIG_ISR(CE32_HJ580_Handle *handleBLE);
void HJ580_EnterConfigMode(CE32_HJ580_Handle *handle);
void HJ580_ExitConfigMode(CE32_HJ580_Handle *handle);
void CE32_HJ580_TX_Start(CE32_HJ580_Handle *handle);

__forceinline void UART_DMA_Init(void){
	PULL_UART_BSY(HJ_UART)
	/* Configure DMA Channel source address */
	HJ_DMA_TX->PAR = (uint32_t)&HJ_UART_DR(HJ_UART);
	UART_DISABLE(HJ_UART);
	SET_BIT(HJ_UART->CR3, USART_CR3_DMAT); //enable UART_DMA_request
	UART_ENABLE(HJ_UART);
}

__forceinline void UART_DMA_Enable(void){
	PULL_UART_BSY(HJ_UART)
	/* Configure DMA Channel source address */
	//SD_DMA_TX->PAR = (uint32_t)&SD_SPI->DR;
	//SD_DMA_TX->NDTR=256;
	UART_DISABLE(HJ_UART);
	SET_BIT(HJ_UART->CR3, USART_CR3_DMAT); //enable UART_DMA_request
	DMA_ENABLE(HJ_DMA_TX);
	UART_ENABLE(HJ_UART);
}

__forceinline void UART_DMA_Disable(void)
{
	PULL_UART_BSY(HJ_UART)
	DMA_DISABLE(HJ_DMA_TX);
	UART_DISABLE(HJ_UART);
	CLEAR_BIT(HJ_UART->CR3, USART_CR3_DMAT); //disable uart _DMA_request
	UART_ENABLE(HJ_UART);
}

__forceinline void MSG_CMD_INC(uint32_t* cmd_ptr)
{
	(*cmd_ptr)++;
	(*cmd_ptr)&=HJ_CMD_SEQ-1;
}

//__forceinline void MSG_Enqueue(uint32_t* ptr,uint8_t* buf,uint8_t data)
//{
//	(*ptr)++;
//	(*ptr)&=HJ_CMD_BUFSIZE-1;
//	(*(buf+*ptr))=data;
//}

#endif


