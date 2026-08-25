#ifndef __OBSERVER_H
#define __OBSERVER_H

#include "MotionEstimation.h"

typedef struct
{
	float x;		//轮子位移x(m)
	float Angle;	//轮子角度(rad)
	float Speed;	//轮子速度(rad/s)
	float T;		//轮子转矩T(N·m)
}Observer_WheelStatus;

typedef struct
{
	Observer_WheelStatus Wheel;	//轮子状态
	
	float phi_1;				//五连杆phi1(rad)
	float phi_4;				//五连杆phi4(rad)
	float dphi_1;				//五连杆dphi1(rad/s)
	float dphi_4;				//五连杆dphi4(rad/s)
	
	float L_0;					//摆杆长度L0(m)
	float last_dL_0;			//上一次摆杆长度last_L0(m)
	float dL_0;					//摆杆长度变化率dL0(m/s)
	float ddL_0;				//摆杆长度二阶变化率ddL0(m/s^2)
	
	float phi_0;				//摆杆角度phi0(rad)
	float dphi_0;				//摆杆角速度dphi0(rad/s)
	
	float T1;					//髋关节1的转矩T1(N·m)
	float T2;					//髋关节2的转矩T2(N·m)
	
	float theta;				//摆杆摆角theta(rad)
	float last_dtheta;			//上一次摆杆摆角last_theta(rad)
	float dtheta;				//摆杆摆角角速度dtheta(rad/s)
	float ddtheta;				//摆杆摆角角加速度ddtheta(rad/s^2)
	
	float F;					//摆杆推力F(N)
	float Tp;					//摆杆扭矩Tp(N·m)
	float FN;					//支持力FN(N)
	
	float J_11;					//VMC雅可比矩阵J元素
	float J_12;					//VMC雅可比矩阵J元素
	float J_21;					//VMC雅可比矩阵J元素
	float J_22;					//VMC雅可比矩阵J元素
	
	float T_11;					//VMC逆解矩阵T元素
	float T_12;					//VMC逆解矩阵T元素
	float T_21;					//VMC逆解矩阵T元素
	float T_22;					//VMC逆解矩阵T元素
}Observer_LegStatus;

typedef struct
{
	float Yaw;									//偏航角(rad)
	float GM6020_Yaw;
	float Yaw_Theta;
	float Pitch;								//俯仰角(rad)
	float Roll;									//翻滚角(rad)
	
	float dYaw;									//偏航角速度(rad/s)
	float dPitch;								//俯仰角速度(rad/s)
	float dRoll;								//翻滚角速度(rad/s)
	
	float a_xE;									//世界坐标系x轴加速度(m/s^2)
	float a_yE;									//世界坐标系y轴加速度(m/s^2)
	float a_zE;									//世界坐标系z轴加速度(m/s^2)

	float a_xb;									//机体坐标系x轴加速度(m/s^2)
	float a_yb;									//机体坐标系y轴加速度(m/s^2)
	float a_zb;									//机体坐标系z轴加速度(m/s^2)
	
	float x;									//位移x(m)
	float dx;									//速度dx(m/s)
	float x_nofilter;							//未运动估计的位移(m)
	float dx_nofilter;							//未运动估计的速度(m)
	
	MotionEstimation_Balance MotionEstimation;	//运动估计结构体
}Observer_BodyStatus;

typedef struct
{
	float dt;						//闭环周期
	
	float P;
	
	Observer_LegStatus LeftLeg;		//左腿
	Observer_LegStatus RightLeg;	//右腿
	
	Observer_BodyStatus Body;		//机体
}Observer_Balance;

extern Observer_Balance Observer_BalanceStatus;

void Observer_Init(void);//观测器初始化
void Observer_DataGet(void);//观测器数据处理
void Observer_GetWheelStatus(void);//轮子状态获取

#endif
