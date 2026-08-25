#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "UART.h"
#include "STP.h"

uint8_t STP_RxPacket[200],STP_RxFlag;
STP_MeasurePoint STP_Measure[12];

uint8_t STP_MaxConfidence=0,STP_MaxConfidenceIndex=0;
float STP_Distance=65535;

void STP_Init(void)
{
	UART2_Init();
}

void STP_DataProcess(void)
{
	STP_Measure[0].distance=STP_RxPacket[0]|((uint16_t)STP_RxPacket[1]<<8);
	STP_Measure[0].confidence=STP_RxPacket[8];
	STP_MaxConfidence=STP_Measure[0].confidence;
	STP_MaxConfidenceIndex=0;
	
	for(uint8_t i=1;i<12;i++)
	{
		STP_Measure[i].distance=STP_RxPacket[i*15+0]|((uint16_t)STP_RxPacket[i*15+1]<<8);
		STP_Measure[i].confidence=STP_RxPacket[i*15+8];
		
		if(STP_Measure[i].confidence>STP_MaxConfidence)
		{
			STP_MaxConfidence=STP_Measure[i].confidence;
			STP_MaxConfidenceIndex=i;
		}
	}
	
	STP_Distance=STP_Measure[STP_MaxConfidenceIndex].distance;
}

void USART1_IRQHandler(void)
{
	#define STP_DataLength		185//有效数据位数
	
	static uint16_t RxState=0;//定义静态变量用于接收模式的选择
	static uint16_t pRxState=0;//定义静态变量用于充当计数器
	static uint8_t CS=0;
	
	uint8_t STP_RxData;//裁判系统接收数据
		
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)//查询接收中断标志位
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除接收中断标志位
		
		STP_RxData=USART_ReceiveData(USART1);//将数据存入缓存区
		
		if(RxState==0){if(STP_RxData==0xAA)RxState=1;}
		else if(RxState==1){if(STP_RxData==0xAA)RxState=2;else RxState=0;}
		else if(RxState==2){if(STP_RxData==0xAA)RxState=3;else RxState=0;}
		else if(RxState==3){if(STP_RxData==0xAA)RxState=4;else RxState=0;}
		else if(RxState==4){if(STP_RxData==0x00)RxState=5;else RxState=0;}
		else if(RxState==5){if(STP_RxData==0x02)RxState=6;else RxState=0;}
		else if(RxState==6){if(STP_RxData==0x00)RxState=7;else RxState=0;}
		else if(RxState==7){if(STP_RxData==0x00)RxState=8;else RxState=0;}
		else if(RxState==8){if(STP_RxData==0xB8)RxState=9;else RxState=0;}
		else if(RxState==9){if(STP_RxData==0x00){RxState=10;pRxState=0;CS=0xBA;}else RxState=0;}
		else if(RxState==10)
		{
			STP_RxPacket[pRxState]=STP_RxData;//接收数据
			pRxState++;
			
			if(pRxState>=STP_DataLength)RxState=11;//转入模式2
			else CS+=STP_RxData;
		}
		else if(RxState==11)//模式2-等待包尾
		{
			if(STP_RxPacket[184]==CS)//检测包尾
			{
				STP_DataProcess();
				RxState=0;//回到模式0
				STP_RxFlag=1;//置接收完成标志位
			}
		}				
	}
}

