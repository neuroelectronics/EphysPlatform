#include "stm32f7xx_hal.h"

#ifndef dataMGR_def
#define dataMGR_def

#define MGR_STATE_TRANSBUSY 0x00000001U	//Transfer packet is on
#define MGR_STATE_TRANSCPLT 0x00000002U
#define MGR_STATE_DSPBUSY 	0x00000004U
#define MGR_STATE_LOGGING 	0x00000008U
#define MGR_STATE_TRANSON	 	0x00000010U
#define MGR_STATE_SLEEP	 		0x00000020U
#define MGR_STATE_STOP	 		0x00000040U
#define MGR_STATE_TRANSMISC 0x00000080U
#define MGR_STATE_TRANSERR  0x00000100U
#define MGR_STATE_TRANSOVR  0x00000200U
#define MGR_STATE_PREV_TRANSBUSY 0x00000400U	
#define MGR_STATE_PREV_TRANSCPLT 0x00000800U	



#define MGR_SYS_RECORDING		0x00000001U
#define MGR_SYS_PREVIEW			0x00000002U
#define MGR_SYS_USBCONN			0x00000004U
#define MGR_SYS_CL_CALIB		0x00000008U
#define MGR_SYS_CL_PROCESS	0x00000010U
#define MGR_SYS_TIMEDMODE		0x00000020U

#define MISC_SIG_TRIG				0x00000100U
#define MISC_SIG_STIM				0x00000200U
#define MISC_SIG_INPUT			0x00000400U

#define SYS_ADDR						0x0001
#define DSP_ADDR						0x0002
#define LOG_ADDR            0x0080
#define DATA_ADDR						0x0100

#define outPTR_num 2
#define ulong unsigned long
#define uchar unsigned char
#define real32_T float
#define RES_RATE						16
#define PREV_RATE						25
#define PREV_RATE_MISC			2
#define PREV_NUMCH					1
#define MISC_NUMCH					8
#define PREV_TRANS_BYTES		128
#define LOG_INTERVAL				0x20000
#define MISC_INTERVAL				64

//BUF_SIZE must be 512B aligned,to compatiable with DMA process,and complain with LFP_size and mask rule(64x & 64x-1 =mask),has to be prod of 2
#define BUF_SIZE 						0x2000
//#define BUF_MASK 						(BUF_SIZE-1)
#define LFP_SIZE 						0x100
#define LFP_MASK 						(LFP_SIZE/64-1)
#define MISC_SIZE 					0x100
#define MISC_MASK 					(MISC_SIZE-1)
#define BUF_MASK MISC_MASK
#define PREV_SIZE						0x100
#define PREV_MASK						(PREV_SIZE-1)
//#define DATASIZE 515*50
#define DATASIZE MGR->dataSize

//#define REPORT_MAX_BUF

#define SYS_SD					0x00000001U
#define SYS_BLE		 			0x00000002U
#define SYS_WIFI 				0x00000004U
#define SYS_USB	 				0x00000008U
#define SYS_ACC	 				0x00000010U
#define SYS_INS	 				0x00000020U
#define SYS_LOC	 				0x00000040U
#define SYS_MIC				 	0x00000080U
#define SYS_MOTOR0  		0x00000100U
#define SYS_MOTOR1  		0x00000200U
#define SYS_MOTOR2			0x00000001U
#define SYS_MOTOR3			0x00000002U
#define SYS_MOTOR4			0x00000004U
#define SYS_MOTOR5			0x00000008U
#define SYS_MOTOR6			0x00000010U
#define SYS_MOTOR7			0x00000020U

typedef struct{
	unsigned long moduleConnected;
}CE32_sysState;

typedef struct{
	unsigned long logState;
	unsigned long sysState;
	unsigned long dataSize;
	unsigned long inPTR;
	unsigned long outPTR[outPTR_num];
	unsigned long bufferUsed[outPTR_num];
	unsigned long bufferMAX[outPTR_num];
	char* dataPtr;		//pointer to data buffer	
	__IO uint32_t* DMA_RX_NDTR;
	__IO uint32_t* DMA_TX_NDTR;
}dataMGR;

typedef struct{
	ulong fs;					//Sampling Rate 
	ulong AUX_mode;				//AUX pin mode, 0 = unused , 1=Timer-controlled, 2=Closed-loop
	ulong Nch;					//Channel number(based on one chip), eg. max(RHD2164)=32
	unsigned short convCmd[32];			//convert commands			//64B
	ulong dispCH;					//Select channel for HighSpeed visualization
	void (*func1)(void);		//Function handle for filter1
	void (*func2)(void);		//Function handle for filter2	
	ulong cmd_ch;						//Convert command ptr	(current SPI out)
	ulong rec_ch;						//Record sample ptr		(current SPI in)
	ulong stim_mode;
  ulong cl_mode;
	ulong stim_interval;
  ulong pulse_width;
  ulong stim_delay;
	ulong stim_RndDelay;
  ulong trigger_trainStart;
  ulong trigger_trainDuration;
  float trigger_gain;
	ulong SD_capacity;
	ulong LED_pulse_CNT;
	ulong preview_channel_bank;
	ulong system_status;
	ulong unassigned[90];		//unassigned
}CE32_systemParam;

typedef struct{
	ulong Nrec;					//Number of records
	ulong log[127];				//Log storage locations. Format as [Start1,End1,Start2,End2....] keep size as 512B 
}CE32_systemLog;

typedef struct{
	uchar chOrd[128];			//Channel order for calculation;
	ulong formula;				//Formula type					; 132B
	ulong func1;		//Function handle1				; 136B
	ulong func2;		//Function handle2				; 140B
	ulong MAOrd;							//Channel number for DSP	; 144B
	float filter1[46];			//Filter1 Param
	float filter2[46];			//Filter2 Param
	
}CE32_dspParam;

void dataMGR_init(dataMGR* MGR,char* dataPtr,unsigned long dataSize);
void dataMGR_init_DMA(dataMGR* MGR,char* dataPtr,unsigned long dataSize,__IO uint32_t* DMA_RX_NDTR,__IO uint32_t* DMA_TX_NDTR);
void dataMGR_init_515B(dataMGR* MGR,char* dataPtr,unsigned long dataSize);
void dataMGR_IncNrec(CE32_systemLog* sysLog);
void CE32_init(CE32_systemParam* sys);

void syslog_init(CE32_systemLog* log,unsigned int* filePTR);
	
__forceinline void CE32_convertPtrInc(CE32_systemParam* sys){
	sys->cmd_ch++;
	sys->cmd_ch&=0x1F;
}

__forceinline void dataMGR_enQueue_byte(dataMGR* MGR,char data){
	MGR->dataPtr[MGR->inPTR]=data; //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%potential bug
	MGR->inPTR++;
	if(MGR->inPTR>=DATASIZE){
		MGR->inPTR-=DATASIZE;
	}
	for(int i=0;i<outPTR_num;i++){
		MGR->bufferUsed[i]++;
		#ifdef REPORT_MAX_BUF
		if(MGR->bufferUsed[i]>MGR->bufferMAX[i])	MGR->bufferMAX[i]=MGR->bufferUsed[i];
		#endif
	}
}

__forceinline void dataMGR_enQueue_515B_halfword(dataMGR* MGR,short data){ 
	*(short*)(&MGR->dataPtr[MGR->inPTR])=data;
	MGR->inPTR+=2;

	if((MGR->inPTR%515)==513){ //if ptr>512 move to 1
		MGR->inPTR+=3;
	}
	if(MGR->inPTR>=DATASIZE){
			MGR->inPTR-=DATASIZE;
		}
	for(int i=0;i<outPTR_num;i++){
		MGR->bufferUsed[i]+=2;
		#ifdef REPORT_MAX_BUF
		if(MGR->bufferUsed[i]>MGR->bufferMAX[i])	MGR->bufferMAX[i]=MGR->bufferUsed[i];
		#endif
	}
}

__forceinline void dataMGR_enQueue_halfword(dataMGR* MGR,short data){
	if(MGR->inPTR<DATASIZE-1){
		*(short*)(MGR->dataPtr+MGR->inPTR)=data;
		MGR->inPTR+=2;
		if(MGR->inPTR>=DATASIZE){
			MGR->inPTR-=DATASIZE;
		}
	}	
	else{
		MGR->dataPtr[MGR->inPTR]=0xff&data;
		MGR->inPTR++;
		if(MGR->inPTR>=DATASIZE){
			MGR->inPTR-=DATASIZE;
		}
		MGR->dataPtr[MGR->inPTR]=(0xff00&data)>>8;
		MGR->inPTR++;
		if(MGR->inPTR>=DATASIZE){
			MGR->inPTR-=DATASIZE;
		}
	}
	for(int i=0;i<outPTR_num;i++){
		MGR->bufferUsed[i]+=2;
		#ifdef REPORT_MAX_BUF
		if(MGR->bufferUsed[i]>MGR->bufferMAX[i])	MGR->bufferMAX[i]=MGR->bufferUsed[i];
		#endif
	}
}

__forceinline void dataMGR_enQueue_halfword_aligned(dataMGR* MGR,short data){	//fast enqueue function without boundary check
	*(short*)(MGR->dataPtr+MGR->inPTR)=data;
	MGR->inPTR+=2;
	MGR->inPTR&=BUF_MASK;
	for(int i=0;i<outPTR_num;i++){
		MGR->bufferUsed[i]+=2;
		#ifdef REPORT_MAX_BUF
		if(MGR->bufferUsed[i]>MGR->bufferMAX[i])	MGR->bufferMAX[i]=MGR->bufferUsed[i];
		#endif
	}
}

__forceinline void dataMGR_enQueue_halfword_DMA_1B(dataMGR* MGR){	//fast enqueue function without boundary check
	MGR->inPTR=BUF_SIZE-*MGR->DMA_RX_NDTR;  //DMA is in byte mode
	for(int i=0;i<outPTR_num;i++){
		MGR->bufferUsed[i]+=2;
		#ifdef REPORT_MAX_BUF
		if(MGR->bufferUsed[i]>MGR->bufferMAX[i])	MGR->bufferMAX[i]=MGR->bufferUsed[i];
		#endif
	}
}

__forceinline void dataMGR_enQueue_halfword_DMA_2B(dataMGR* MGR){	//fast enqueue function without boundary check
	MGR->inPTR=BUF_SIZE-*MGR->DMA_RX_NDTR*2;	//DMA is in halfword mode
	for(int i=0;i<outPTR_num;i++){
		MGR->bufferUsed[i]+=2;
		#ifdef REPORT_MAX_BUF
		if(MGR->bufferUsed[i]>MGR->bufferMAX[i])	MGR->bufferMAX[i]=MGR->bufferUsed[i];
		#endif
	}
}

__forceinline void dataMGR_enQueue_Nhalfword_DMA_2B(dataMGR* MGR,uint32_t Nbytes){	//fast enqueue function without boundary check
	MGR->inPTR=BUF_SIZE-*MGR->DMA_RX_NDTR*2;	//DMA is in halfword mode
	for(int i=0;i<outPTR_num;i++){
		MGR->bufferUsed[i]+=Nbytes;
		#ifdef REPORT_MAX_BUF
		if(MGR->bufferUsed[i]>MGR->bufferMAX[i])	MGR->bufferMAX[i]=MGR->bufferUsed[i];
		#endif
	}
}

__forceinline void dataMGR_enQueue_Nbytes(dataMGR* MGR,uint32_t Nbytes){	//fast enqueue function without boundary check
	MGR->inPTR+=Nbytes;
	if(MGR->inPTR>=DATASIZE){
		MGR->inPTR-=DATASIZE;
	}
	for(int i=0;i<outPTR_num;i++){
		MGR->bufferUsed[i]+=Nbytes;
		#ifdef REPORT_MAX_BUF
		if(MGR->bufferUsed[i]>MGR->bufferMAX[i])	MGR->bufferMAX[i]=MGR->bufferUsed[i];
		#endif
	}
}

__forceinline void dataMGR_enQueue_word(dataMGR* MGR,long data){
	if(MGR->inPTR<DATASIZE-3){
		*(long*)(MGR->dataPtr+MGR->inPTR)=data;
		MGR->inPTR+=4;
		if(MGR->inPTR>=DATASIZE){
			MGR->inPTR-=DATASIZE;
		}
	}
	else{
		MGR->dataPtr[MGR->inPTR]=0xff&data;
		MGR->inPTR++;
		if(MGR->inPTR>=DATASIZE){
			MGR->inPTR-=DATASIZE;
		}
		MGR->dataPtr[MGR->inPTR]=(0xff00&data)>>8;
		MGR->inPTR++;
		if(MGR->inPTR>=DATASIZE){
			MGR->inPTR-=DATASIZE;
		}
		MGR->dataPtr[MGR->inPTR]=0xff0000&data>>16;
		MGR->inPTR++;
		if(MGR->inPTR>=DATASIZE){
			MGR->inPTR-=DATASIZE;
		}
		MGR->dataPtr[MGR->inPTR]=(0xff000000&data)>>24;
		MGR->inPTR++;
		if(MGR->inPTR>=DATASIZE){
			MGR->inPTR-=DATASIZE;
		}
	}
	for(int i=0;i<outPTR_num;i++){
		MGR->bufferUsed[i]+=4;
	}
}

__forceinline char dataMGR_deQueue_byte(dataMGR* MGR,unsigned short outPTR_idx){
	char output=MGR->dataPtr[MGR->outPTR[outPTR_idx]];
	MGR->outPTR[outPTR_idx]++;
	if(MGR->outPTR[outPTR_idx]>=DATASIZE){
		MGR->outPTR[outPTR_idx]-=DATASIZE;
	}
	MGR->bufferUsed[outPTR_idx]--;
	return output;
}

__forceinline void dataMGR_deQueue(dataMGR* MGR,unsigned long numBytes,unsigned short outPTR_idx){
	MGR->outPTR[outPTR_idx]+=numBytes;
	if(MGR->outPTR[outPTR_idx]>=DATASIZE){
		MGR->outPTR[outPTR_idx]-=DATASIZE;
	}
	MGR->bufferUsed[outPTR_idx]-=numBytes;
}

__forceinline float CE32_arbitarCal(CE32_dspParam* param, short* dataPacket){
	switch(param->formula){
		case 0:
			return dataPacket[param->chOrd[0]];
		case 1:
			return dataPacket[param->chOrd[0]]-dataPacket[param->chOrd[1]];
		case 2:
			return 2*dataPacket[param->chOrd[0]]-dataPacket[param->chOrd[1]]-dataPacket[param->chOrd[2]];
	}
	return 0;
}
#endif
