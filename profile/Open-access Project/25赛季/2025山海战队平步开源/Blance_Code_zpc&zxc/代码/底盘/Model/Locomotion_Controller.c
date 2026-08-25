#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "PID.h"
#include "Observer.h"
#include "Data.h"
#include "RefereeSystem.h"
#include "Parameter.h"
#include "GM6020.h"

PID_PositionInitTypedef Yaw_ControllerPositionPID,Yaw_ControllerSpeedPID,LegCoordination_ControllerPID,Pitch_ControllerPID,X_ControllerPID,Roll_L0ControllerPID,Roll_FControllerPID;

/*
 *函数简介:综合运动控制初始化
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Locomotion_Controller_Init(void)
{
////	PID_PositionStructureInit(&Yaw_ControllerPositionPID,0);
////	PID_PositionSetParameter(&Yaw_ControllerPositionPID,1,0,0);
////	PID_PositionSetEkRange(&Yaw_ControllerPositionPID,-0.05f,0.05f);
////	PID_PositionSetOUTRange(&Yaw_ControllerPositionPID,-3,3);
////	PID_PositionStructureInit(&Yaw_ControllerSpeedPID,0);
////	PID_PositionSetParameter(&Yaw_ControllerSpeedPID,10,0,0);
////	PID_PositionSetEkRange(&Yaw_ControllerSpeedPID,-0.1f,0.1f);
////	PID_PositionSetOUTRange(&Yaw_ControllerSpeedPID,-5,5);
//	PID_PositionStructureInit(&Yaw_ControllerPositionPID,0);
//	PID_PositionSetParameter(&Yaw_ControllerPositionPID,8,0,1);
//	PID_PositionSetEkRange(&Yaw_ControllerPositionPID,-0.01f,0.01f);
//	PID_PositionSetOUTRange(&Yaw_ControllerPositionPID,-10,10);

	//双腿协调
	PID_PositionStructureInit(&LegCoordination_ControllerPID,0);
	PID_PositionSetParameter(&LegCoordination_ControllerPID,100,0,0);
	//PID_PositionSetEkRange(&LegCoordination_ControllerPID,-0.05f,0.05f);
	PID_PositionSetOUTRange(&LegCoordination_ControllerPID,-10,10);

	PID_PositionStructureInit(&Pitch_ControllerPID,0);
	PID_PositionSetParameter(&Pitch_ControllerPID,5,0,0);
	PID_PositionSetEkRange(&Pitch_ControllerPID,-0.1f,0.1f);
	PID_PositionSetOUTRange(&Pitch_ControllerPID,-2,2);

	PID_PositionStructureInit(&X_ControllerPID,0);
	PID_PositionSetParameter(&X_ControllerPID,2,0,0);
	PID_PositionSetEkRange(&X_ControllerPID,-0.1f,0.1f);
	PID_PositionSetOUTRange(&X_ControllerPID,-2,2);

	PID_PositionStructureInit(&Roll_L0ControllerPID,0);
	PID_PositionSetParameter(&Roll_L0ControllerPID,0.5f,0,0);
	PID_PositionSetEkRange(&Roll_L0ControllerPID,-0.01f,0.01f);
	PID_PositionSetOUTRange(&Roll_L0ControllerPID,-0.3,0.3);

	PID_PositionStructureInit(&Roll_FControllerPID,0);
	PID_PositionSetParameter(&Roll_FControllerPID,500,0,25);
	PID_PositionSetEkRange(&Roll_FControllerPID,-0.01f,0.01f);
	PID_PositionSetOUTRange(&Roll_FControllerPID,-50,50);
}

/*
 *函数简介:Yaw控制
 *参数说明:左轮ΔT
 *参数说明:右轮ΔT
 *返回类型:无
 *备注:无
 */
void Locomotion_Controller_Yaw_Control(float target_Yaw,float *LeftWheel_DeltaT,float *RightWheel_DeltaT,float w_Limit)
{
	#define Yaw_Control_LQR_K1		4.4721f
	#define Yaw_Control_LQR_K2		6.3246f//31.6228    6.9680
	
	float w=Yaw_Control_LQR_K1*(Observer_BalanceStatus.Body.Yaw-target_Yaw);
	w=Data_Clipping(w,-w_Limit,w_Limit);
	float Tau=Yaw_Control_LQR_K2*(Observer_BalanceStatus.Body.dYaw+w);
	
	(*LeftWheel_DeltaT)=-Tau/2.0f;
	(*RightWheel_DeltaT)=Tau/2.0f;
}

/*
 *函数简介:Yaw控制
 *参数说明:左轮ΔT
 *参数说明:右轮ΔT
 *返回类型:无
 *备注:无
 */
void Locomotion_Controller_Yaw_Control2(float target_Yaw,float *LeftWheel_DeltaT,float *RightWheel_DeltaT,float w_Limit)
{ 
	float w=Yaw_Control_LQR_K1*(-(GM6020_MotorStatus[0].Angle-Yaw_GM6020PositionValue)/8192.0f*2.0f*PI-target_Yaw);
	w=Data_Clipping(w,-w_Limit,w_Limit);
	float Tau=Yaw_Control_LQR_K2*(Observer_BalanceStatus.Body.dYaw+w);
	
	(*LeftWheel_DeltaT)=-Tau/2.0f;
	(*RightWheel_DeltaT)=Tau/2.0f;
//	int16_t T=w_Limit*Observer_BalanceStatus.dt/(2.0f*PI)*8192;

//	if((int16_t)GM6020_MotorStatus[0].Angle-Now_Value>T)Now_Value+=T;
//	else if((int16_t)GM6020_MotorStatus[0].Angle-Now_Value<-T)Now_Value-=T;
//	else Now_Value=GM6020_MotorStatus[0].Angle;
//	
//	float Tau=Yaw_Control_LQR_K1*(-(Now_Value-Yaw_GM6020PositionValue)/8192.0f*2.0f*PI)+Yaw_Control_LQR_K2*Observer_BalanceStatus.Body.dYaw;
//	
//	(*LeftWheel_DeltaT)=-Tau/2.0f;
//	(*RightWheel_DeltaT)=Tau/2.0f;
}

/*
 *函数简介:双腿协调
 *参数说明:左腿ΔTp
 *参数说明:右腿ΔTp
 *返回类型:无
 *备注:无
 */
void Locomotion_Controller_LegCoordination_Control(float *LeftLeg_DeltaTp,float *RightLeg_DeltaTp)
{
	//PID_PositionCalc(&LegCoordination_ControllerPID,Observer_BalanceStatus.RightLeg.theta-Observer_BalanceStatus.LeftLeg.theta);
	PID_PositionCalcGivenDot(&LegCoordination_ControllerPID,Observer_BalanceStatus.RightLeg.theta-Observer_BalanceStatus.LeftLeg.theta,Observer_BalanceStatus.RightLeg.dtheta-Observer_BalanceStatus.LeftLeg.dtheta);
	
	(*LeftLeg_DeltaTp)=LegCoordination_ControllerPID.OUT;
	(*RightLeg_DeltaTp)=-LegCoordination_ControllerPID.OUT;
}

/*
 *函数简介:Pitch控制
 *参数说明:左轮ΔT
 *参数说明:右轮ΔT
 *返回类型:无
 *备注:无
 */
/*
void Locomotion_Controller_Pitch_Control(float *LeftWheel_DeltaT,float *RightWheel_DeltaT)
{
	PID_PositionCalc(&Pitch_ControllerPID,Observer_BalanceStatus.Body.Pitch);
	
	(*LeftWheel_DeltaT)=-Pitch_ControllerPID.OUT;
	(*RightWheel_DeltaT)=-Pitch_ControllerPID.OUT;
}
*/

/*
 *函数简介:位移控制
 *参数说明:左轮ΔT
 *参数说明:右轮ΔT
 *返回类型:无
 *备注:无
 */
 /*
void Locomotion_Controller_X_Control(float *LeftWheel_DeltaT,float *RightWheel_DeltaT)
{
	PID_PositionCalc(&X_ControllerPID,Observer_BalanceStatus.Body.x);
	
	(*LeftWheel_DeltaT)=X_ControllerPID.OUT;
	(*RightWheel_DeltaT)=X_ControllerPID.OUT;	
}
*/

/*
 *函数简介:Roll补偿
 *参数说明:左腿ΔL0
 *参数说明:右腿ΔL0
 *参数说明:左腿ΔF
 *参数说明:右腿ΔF
 *返回类型:无
 *备注:无
 */
void Locomotion_Controller_Roll_Control(float Roll_Target,float *LeftLeg_DeltaL0,float *RightLeg_DeltaL0,float *LeftLeg_DeltaF,float *RightLeg_DeltaF)
{
	/*
//	PID_PositionCalc(&Roll_L0ControllerPID,Observer_BalanceStatus.Body.Roll);
//	PID_PositionCalc(&Roll_FControllerPID,Observer_BalanceStatus.Body.Roll);
	PID_PositionCalcGivenDot(&Roll_L0ControllerPID,Observer_BalanceStatus.Body.Roll,Observer_BalanceStatus.Body.dRoll);
	PID_PositionCalcGivenDot(&Roll_FControllerPID,Observer_BalanceStatus.Body.Roll,Observer_BalanceStatus.Body.dRoll);
	
	(*LeftLeg_DeltaL0)=Roll_L0ControllerPID.OUT;
	(*RightLeg_DeltaL0)=-Roll_L0ControllerPID.OUT;	
	(*LeftLeg_DeltaF)=Roll_FControllerPID.OUT;
	(*RightLeg_DeltaF)=-Roll_FControllerPID.OUT;
	*/

	float Delta_L0=Observer_BalanceStatus.RightLeg.L_0-Observer_BalanceStatus.LeftLeg.L_0;
	float A=Delta_L0*arm_cos_f32(Observer_BalanceStatus.Body.Roll-Roll_Target)+2.0f*Rl*arm_sin_f32(Observer_BalanceStatus.Body.Roll-Roll_Target);
	float B=-Delta_L0*arm_sin_f32(Observer_BalanceStatus.Body.Roll-Roll_Target)+2.0f*Rl*arm_cos_f32(Observer_BalanceStatus.Body.Roll-Roll_Target);
	float tan_delta,L0d_r,L0d_l;
	if(B==0){L0d_r=L0d_l=0;}
	else
	{
		tan_delta=A/B;

		L0d_r=Rl*tan_delta;
		L0d_l=-Rl*tan_delta;
	}
	(*LeftLeg_DeltaL0)=L0d_l;
	(*RightLeg_DeltaL0)=L0d_r;
	
	Roll_FControllerPID.Need_Value=Roll_Target;
	PID_PositionCalcGivenDot(&Roll_FControllerPID,Observer_BalanceStatus.Body.Roll,Observer_BalanceStatus.Body.dRoll);
	(*LeftLeg_DeltaF)=Roll_FControllerPID.OUT;
	(*RightLeg_DeltaF)=-Roll_FControllerPID.OUT;
}
