#ifndef __LOCOMOTION_CONTROLLER_H
#define __LOCOMOTION_CONTROLLER_H

#include "PID.h"

extern PID_PositionInitTypedef Yaw_ControllerPositionPID;

void Locomotion_Controller_Init(void);//综合运动控制初始化
void Locomotion_Controller_Yaw_Control(float target_Yaw,float *LeftWheel_DeltaT,float *RightWheel_DeltaT,float w_Limit);
void Locomotion_Controller_Yaw_Control2(float target_Yaw,float *LeftWheel_DeltaT,float *RightWheel_DeltaT,float w_Limit);
void Locomotion_Controller_LegCoordination_Control(float *LeftLeg_DeltaTp,float *RightLeg_DeltaTp);//双腿协调
void Locomotion_Controller_Roll_Control(float Roll_Target,float *LeftLeg_DeltaL0,float *RightLeg_DeltaL0,float *LeftLeg_DeltaF,float *RightLeg_DeltaF);//Roll补偿

#endif
