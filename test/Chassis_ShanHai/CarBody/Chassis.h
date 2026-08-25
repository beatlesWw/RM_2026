#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <stdint.h>


#define RollThreshold		(30.0f/180.0f*3.1415926f)


extern float Tl,Tpl,T1l,T2l,Tr,Tpr,T1r,T2r,Fl,Fr;
extern uint8_t Chassis_Model;
extern float TargetX,TargetdX;
extern float Chassis_PowerLimit,Chassis_EstimatedPower;
extern float PowerControl_dx;
extern float Accel;
extern float TargetL0;

void Chassis_Init(void);
void Chassis_MotorControl(float T_l,float T1_l,float T2_l,float T_r,float T1_r,float T2_r);
void Chassis_ModelControl(void);
void Chassis_Reset(void);
void Chassis_Control(void);

#endif
