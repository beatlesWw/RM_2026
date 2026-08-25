#ifndef __MF9025_H
#define __MF9025_H

#include <stdint.h>

typedef enum
{
	MF9025_1=0x141,//ID1
	MF9025_2=0x142,//ID2
}MF9025_ID;//MF9025电机ID号枚举

typedef struct
{
	uint16_t Angle;//MF9025电机机械角度(0~65536对应0~360°)
	uint8_t First_Flag;//MF9025电机首次接收标志位
	int64_t r;//MF9025电机转过圈数
	int64_t Position;//MF9025电机角度位置
	int64_t PositionCheck;
	float Speed;//MF9025电机转速(单位rad/s)
	int16_t Current;//MF9025电机实际转矩电流(-2048~2048对应-33~33A)
	uint8_t Temperature;//MF9025电机电机温度
	
	float Torque;//MF9025扭矩
	float Power;//MF9025功率
}MF9025_Motor;//MF9025电机状态结构体

extern MF9025_Motor MF9025_MotorStatus[];//MF9025电机状态数组

uint8_t MF9025_CANGetStatus(MF9025_ID ID);//CAN1总线获取MF9025电机状态
uint8_t MF9025_CANSetTorque(MF9025_ID ID,float Torque);//CAN总线设置MF9025扭矩
void MF9025_CANDataProcess(MF9025_ID ID,uint8_t *Data);//MF9025数据处理

#endif
