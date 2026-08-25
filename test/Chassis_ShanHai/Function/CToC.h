#ifndef __CTOC_H
#define __CTOC_H

#include <stdint.h>

#define CToC_MasterID1	0x019//主机ID1

#define CToC_SlaveID1	0x05//从机ID1
#define CToC_SlaveID2	0x14//从机ID2
#define CToC_SlaveID3	0x18//从机ID3

extern float Gimbal_Yaw,Gimbal_Check;
extern uint8_t Z_Flag;
extern int16_t Gimbal_Pitch;//1000倍
extern uint8_t Visual_Find_Flag;

void CToC_SlaveInit(void);//板间通讯从机初始化
uint8_t CToC_SlaveSendRefereeSystemData(void);//板间通讯从机发送裁判系统数据
void CToC_CANDataProcess(uint32_t ID,uint8_t *Data);//板间通讯数据处理

#endif
