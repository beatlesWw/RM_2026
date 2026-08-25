#ifndef __LEG_CONTROLLER_H
#define __LEG_CONTROLLER_H

#include "PID.h"
#include "Observer.h"

extern PID_PositionInitTypedef Leg_Controller_LeftLegControlPID,Leg_Controller_RightLegControlPID;

void Leg_Controller_VMC(Observer_LegStatus Leg,float F,float Tp,float *T1,float *T2);//腿长控制VMC
void Leg_Controller_LegControlInit(void);//腿长控制初始化
void Leg_Controller_LegControl(float *LeftLeg_DeltaF,float *RightLeg_DeltaF);//腿长控制
void Leg_Controller_InverseKinematicsSolution(float L_0,float phi_0,float *phi_1,float *phi_4);//腿长控制逆运动学解算
void Leg_Controller_LengthLQR(Observer_LegStatus Leg,float target_phi1,float target_phi4,float *T1,float *T2);

#endif
