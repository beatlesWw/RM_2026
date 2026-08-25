#ifndef __GIMBAL_H
#define __GIMBAL_H

#define Gimbal_YawMotor										GM6020_1//Yaw轴电机
#define Gimbal_PitchMotor									DM_J4310_2//Pitch轴电机
#define Gimbal_L_FrictionWheel								M3508_1//左摩擦轮
#define Gimbal_R_FrictionWheel								M3508_2//右摩擦轮

extern uint8_t UI_Flag;

void Gimbal_Init(void);//云台初始化
void Gimbal_CleanPID(void);//云台PID清理
void Gimbal_MoveControl(void);//云台运动控制

#endif
