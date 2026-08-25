#ifndef __CAN_H
#define __CAN_H

#include <stdint.h>

typedef enum
{
	CAN_GM6020=0,//GM6020
	CAN_RoboMasterC,//C板
	CAN_MF9025,//MF9025
	CAN_DM_J8009,//DM_J8009
	CAN_Ultra_CAP,
}CAN_MotorModel;//CAN总线设备分类枚举

extern uint8_t CAN_CAN1DeviceNumber;//CAN1总线上设备数量
extern uint8_t CAN_CAN2DeviceNumber;//CAN2总线上设备数量
extern uint8_t CAN_DeviceNumber;//CAN总线上设备数量
extern uint32_t CAN_CAN1IDList[][4];//CAN1总线上设备ID列表
extern uint32_t CAN_CAN2IDList[][4];//CAN2总线上设备ID列表
extern uint8_t CAN_IDSelect;//CAN总线上ID列表选择位

void CAN_CANInit(void);//CAN总线初始化

#endif
