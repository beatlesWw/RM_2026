#ifndef __DM_J4310_2EC_H
#define __DM_J4310_2EC_H

#include <stdint.h>

#define DM_J8009_PMAX				12.5f//DM_J8009电机位置最大值12.5rad
#define DM_J8009_PMIN				-12.5f//DM_J8009电机位置最小值-12.5rad
#define DM_J8009_VMAX				45.0f//DM_J8009电机速度最大值45rad/s
#define DM_J8009_VMIN				-45.0f//DM_J8009电机速度最小值-45rad/s
#define DM_J8009_TMAX				54.0f//DM_J8009电机转矩最大值54N·m
#define DM_J8009_TMIN				-54.0f//DM_J8009电机转矩最小值-54N·m

typedef enum
{
	DM_J8009_1=0x01,
	DM_J8009_2=0x02,
	DM_J8009_3=0x03,
	DM_J8009_4=0x04,
}DM_J8009_ID;//DM_J8009电机ID号枚举

typedef enum
{
	DM_J8009_Rx1=0xF1,
	DM_J8009_Rx2=0xF2,
	DM_J8009_Rx3=0xF3,
	DM_J8009_Rx4=0xF4,
}DM_J8009_RxID;//DM_J8009电机接收ID号枚举

typedef struct
{
	uint8_t Status;
	
	float Position;//位置(rad)
	float Speed;//速度(rad/s)
	float Torque;//转矩(N·m)
}DM_J8009_Motor;//DM_J8009电机状态结构体

extern DM_J8009_Motor DM_J8009_MotorStatus[];//DM_J8009电机状态数组

void DM_J8009_Init(void);
uint8_t DM_J8009_CANSend(DM_J8009_ID ID,uint8_t *Data);//DM_J8009 CAN总线发送
void DM_J8009_SetTorque(DM_J8009_ID ID,float Torque);
void DM_J8009_DataProcess(DM_J8009_ID ID,uint8_t *Data);

#endif
