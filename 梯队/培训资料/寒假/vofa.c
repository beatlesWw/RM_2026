/*
 * vofa.c
 *
 *  Created on: Nov 28, 2024
 *      Author: WML
 */
#include "stm32f1xx_hal.h"
#include "vofa.h"

#define MAX_CHANNEL 10

#define BYTE0(dwTemp)       (*(char *)(dwTemp))
#define BYTE1(dwTemp)       (*((char *)(dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(dwTemp) + 3))

float* UserData[MAX_CHANNEL] = { 0 };//only transmit float
unsigned char Data_Number = 0;

 void Upper_Computer_Init(float* addr)
{
	if (Data_Number < MAX_CHANNEL)
	UserData[Data_Number++] = addr;
}

unsigned char data_to_send[4 * MAX_CHANNEL + 4] = { 0 }; //使用DMA发送数据时，由于指向的是地址，需要是全局地址，否则真正发送数据时数据已经没了!!!!!!!!!!
void Upper_Computer_Show_Wave(void)
{
	unsigned char cnt = 0;

	for (int i = 0; i < Data_Number; i++)
	{
		data_to_send[cnt++] = BYTE0(UserData[i]);
		data_to_send[cnt++] = BYTE1(UserData[i]);
		data_to_send[cnt++] = BYTE2(UserData[i]);
		data_to_send[cnt++] = BYTE3(UserData[i]);
	}

	  data_to_send[cnt++] = 0x00;
	  data_to_send[cnt++] = 0x00;
	  data_to_send[cnt++] = 0x80;
	  data_to_send[cnt++] = 0x7F;
	//-------------user change-----------------
	extern UART_HandleTypeDef huart1;
//while ((&huart1)->gState != HAL_UART_STATE_READY); //如果上一次包没有发送完成，会导致这个包被丢弃
//HAL_UART_Transmit_DMA(&huart1, data_to_send, cnt);
  
	HAL_UART_Transmit(&huart1, data_to_send, cnt,100);
		//----------------end----------------------
}



