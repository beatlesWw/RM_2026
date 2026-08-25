#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "arm_math.h"
#include "Delay.h"
#include "MF9025.h"
#include "DM_J8009.h"
#include "Observer.h"
#include "Leg_Controller.h"
#include "Parameter.h"
#include "LQR.h"
#include "Locomotion_Controller.h"
#include "Remote.h"
#include "Warming.h"
#include "Data.h"
#include "RefereeSystem.h"
#include "Ultra_CAP.h"
#include "GM6020.h"
#include "STP.h"
#include "Chassis.h"


//#define SingleChassis		0//注释后Yaw跟云台同步
//#define PowerControlFlag	0//注释后不进行功率控制
#define Pingqi				0


uint8_t Chassis_FirstFlag=1;
uint8_t Chassis_FirstFlag2=1;
uint8_t Chassis_YawFlag=0;
uint8_t Chassis_JumpFlag=0;
uint16_t Chassis_JumpCount=0;
uint8_t Chassis_Model=0;
float TargetX,TargetdX,Target_Yaw,TargetL0,TargetRoll,w_Limit=2.5f,YawTrack_Target;

uint8_t Chassis_GyroScopeFlag;//底盘小陀螺标志位
float Chassis_Power;//底盘功率(软件计算值)
float Chassis_PowerLimit,Chassis_EstimatedPower;


void Chassis_Reset(void);

void Chassis_Init(void)
{
	DM_J8009_Init();
	Leg_Controller_LegControlInit();
	Locomotion_Controller_Init();
}

void Chassis_WheelControl(float T_l,float T_r)
{
//	float P_l=fabs(Data_Sqrt3*T_l*Observer_BalanceStatus.LeftLeg.Wheel.Speed);
//	float P_r=fabs(Data_Sqrt3*T_r*Observer_BalanceStatus.RightLeg.Wheel.Speed);
//	Chassis_EstimatedPower=P_l+P_r;
	
	MF9025_CANSetTorque(Chassis_Wheel_L,-T_l);//由于负反馈，在这里力矩取了一次反
	MF9025_CANSetTorque(Chassis_Wheel_R,T_r);
}

void Chassis_MotorControl(float T_l,float T1_l,float T2_l,float T_r,float T1_r,float T2_r)
{
	uint8_t Start[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	static uint16_t EnableCount=0;
	EnableCount++;
	if(EnableCount==200 && DM_J8009_MotorStatus[DM_J8009_1-0x01].Status==0)DM_J8009_CANSend(DM_J8009_1,Start);
	else if(EnableCount==400 && DM_J8009_MotorStatus[DM_J8009_2-0x01].Status==0)DM_J8009_CANSend(DM_J8009_2,Start);
	else if(EnableCount==600 && DM_J8009_MotorStatus[DM_J8009_3-0x01].Status==0)DM_J8009_CANSend(DM_J8009_3,Start);
	else if(EnableCount==800 && DM_J8009_MotorStatus[DM_J8009_4-0x01].Status==0)DM_J8009_CANSend(DM_J8009_4,Start);
	else if(EnableCount==1000)EnableCount=0;
	
	Chassis_WheelControl(T_l,T_r);
	Delay_us(400);
	DM_J8009_SetTorque(Chassis_Joint1_L,-T1_l);
	DM_J8009_SetTorque(Chassis_Joint2_L,-T2_l);
	Delay_us(200);
	DM_J8009_SetTorque(Chassis_Joint1_R,T1_r);
	DM_J8009_SetTorque(Chassis_Joint2_R,T2_r);
}

void Chassis_MotorControl2(float T1_l,float T2_l,float T1_r,float T2_r)
{
	uint8_t Start[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	static uint16_t EnableCount2=0;
	EnableCount2++;
	if(EnableCount2==100)DM_J8009_CANSend(DM_J8009_1,Start);
	else if(EnableCount2==200)DM_J8009_CANSend(DM_J8009_2,Start);
	else if(EnableCount2==300)DM_J8009_CANSend(DM_J8009_3,Start);
	else if(EnableCount2==400)DM_J8009_CANSend(DM_J8009_4,Start);
	else if(EnableCount2==500)EnableCount2=0;

	float Tl,Tr;
	Warming_Brake(&Tl,&Tr);
	
	Chassis_WheelControl(Tl,Tr);
	Delay_us(400);
	DM_J8009_SetTorque(Chassis_Joint1_L,-T1_l);
	DM_J8009_SetTorque(Chassis_Joint2_L,-T2_l);
	Delay_us(200);
	DM_J8009_SetTorque(Chassis_Joint1_R,T1_r);
	DM_J8009_SetTorque(Chassis_Joint2_R,T2_r);
}

void Chassis_StandFromGround(void)
{
	#ifdef Pingqi
		float Tl=0,T1l=0,T2l=0,Tr=0,T1r=0,T2r=0;
	#else
		//float Tl=0,Tpl=0,T1l=0,T2l=0,Tr=0,Tpr=0,T1r=0,T2r=0;
	#endif
	static uint16_t Shoutui_Count=0;
	static uint16_t Pingheng_Count=0;
	
	/*====================LQR建模====================*/
	#ifndef Pingqi
		LQR_Clc(&Tl,&Tpl,&Tr,&Tpr,0,0);
	#endif
	
	/*====================关节力矩====================*/
	float L0=0.1f;
	float L0_l=L0,L0_r=L0;
	float phi1l,phi4l,phi1r,phi4r;
	Leg_Controller_InverseKinematicsSolution(L0_l,PI/2.0f,&phi1l,&phi4l);
	Leg_Controller_InverseKinematicsSolution(L0_r,PI/2.0f,&phi1r,&phi4r);
	Leg_Controller_LengthLQR(Observer_BalanceStatus.LeftLeg,phi1l,phi4l,&T1l,&T2l);
	Leg_Controller_LengthLQR(Observer_BalanceStatus.RightLeg,phi1r,phi4r,&T1r,&T2r);

	/*====================轮向力矩====================*/
	//偏航角控制->T
	float Locomotion_Controller_LeftWheelDeltaT1=0,Locomotion_Controller_RightWheelDeltaT1=0;
	if(Chassis_YawFlag==0)Locomotion_Controller_Yaw_Control(Target_Yaw,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
	else Locomotion_Controller_Yaw_Control2(YawTrack_Target,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);

	//T=LQR_T+偏航角控制T
	Tl=Tl+Locomotion_Controller_LeftWheelDeltaT1;
	Tr=Tr+Locomotion_Controller_RightWheelDeltaT1;
	
	/*====================收腿检测====================*/
	#define Shoutui_AngleThreshold		0.18f
	#define Shoutui_SpeedThreshold		0.04f
	if(fabs(phi1l-Observer_BalanceStatus.LeftLeg.phi_1)<Shoutui_AngleThreshold && fabs(phi4l-Observer_BalanceStatus.LeftLeg.phi_4)<Shoutui_AngleThreshold \
	   && fabs(phi1r-Observer_BalanceStatus.RightLeg.phi_1)<Shoutui_AngleThreshold && fabs(phi4r-Observer_BalanceStatus.RightLeg.phi_4)<Shoutui_AngleThreshold \
	   && fabs(Observer_BalanceStatus.LeftLeg.dphi_1)<Shoutui_SpeedThreshold && fabs(Observer_BalanceStatus.LeftLeg.dphi_4)<Shoutui_SpeedThreshold \
	   && fabs(Observer_BalanceStatus.RightLeg.dphi_1)<Shoutui_SpeedThreshold && fabs(Observer_BalanceStatus.RightLeg.dphi_4)<Shoutui_SpeedThreshold)
		Shoutui_Count++;
	else
		Shoutui_Count=0;
	
	if(Shoutui_Count>200)
	{
		Shoutui_Count=0;
		Chassis_FirstFlag2=0;
	}
	
	/*====================电机控制====================*/
	#define Stand_x			-0.0f
	if(Chassis_FirstFlag2==1)
	{
		Observer_BalanceStatus.Body.MotionEstimation.x_nofilter=Stand_x;
		Observer_BalanceStatus.Body.MotionEstimation.x=Stand_x;
		Observer_BalanceStatus.Body.x_nofilter=Stand_x;
		Observer_BalanceStatus.Body.x=Stand_x;
		
		Chassis_MotorControl2(T1l,T2l,T1r,T2r);
	}
	else
		Chassis_MotorControl(Tl,T1l,T2l,Tr,T1r,T2r);
	//Chassis_MotorControl(0,0,0,0,0,0);
	
	/*====================超时检测====================*/
	static uint16_t TimeOUT_Count=0;
	TimeOUT_Count++;
	if(TimeOUT_Count>5000)
	{
		TimeOUT_Count=0;
		Chassis_Reset();
	}
	
	/*====================平衡检测====================*/
	#ifdef Pingqi
		#define Pingheng_PitchThreshold		(15.0f/180.0f*PI)
	#else
		#define Pingheng_PitchThreshold		(5.0f/180.0f*PI)
	#endif
	
	if(Observer_BalanceStatus.Body.Pitch<Pingheng_PitchThreshold && Observer_BalanceStatus.Body.Pitch>-Pingheng_PitchThreshold)
		Pingheng_Count++;
	else
		Pingheng_Count=0;
	
	if(Pingheng_Count>=500)
	{
		#ifndef SingleChassis
			Chassis_YawFlag=1;
			
			#define Pingheng_YawThreshold		(5.0f/180.0f*PI)
			if(Observer_BalanceStatus.Body.GM6020_Yaw<Pingheng_YawThreshold && Observer_BalanceStatus.Body.GM6020_Yaw>-Pingheng_YawThreshold)
				Pingheng_Count++;
			else
				Pingheng_Count=0;
			
			if(Pingheng_Count>=1000)
			{
				Pingheng_Count=0;
				TimeOUT_Count=0;
				Chassis_FirstFlag=0;
				if(Remote_Status==1)Chassis_Model=Remote_RxData.Remote_5;
				else Chassis_Model=3;
				
				TargetL0=0.2f;
			
				Observer_BalanceStatus.Body.MotionEstimation.x_nofilter=Blance_X;
				Observer_BalanceStatus.Body.MotionEstimation.x=Blance_X;
				Observer_BalanceStatus.Body.x_nofilter=Blance_X;
				Observer_BalanceStatus.Body.x=Blance_X;
			}
		#else //单底盘没有Yaw同步环节
			Pingheng_Count=0;
			TimeOUT_Count=0;
			Chassis_FirstFlag=0;
			Chassis_Model=Remote_RxData.Remote_5;
			if(Chassis_Model==2)TargetL0=0.15f;
			else TargetL0=0.2f;
		
			Observer_BalanceStatus.Body.MotionEstimation.x_nofilter=Blance_X;
			Observer_BalanceStatus.Body.MotionEstimation.x=Blance_X;
			Observer_BalanceStatus.Body.x_nofilter=Blance_X;
			Observer_BalanceStatus.Body.x=Blance_X;
		#endif
	}
}

	float Tl=0,Tpl=0,T1l=0,T2l=0,Tr=0,Tpr=0,T1r=0,T2r=0,Fl=0,Fr=0;
void Chassis_ModelTwo(void)
{
	Tl=Tpl=T1l=T2l=Tr=Tpr=T1r=T2r=Fl=Fr=0;
	/*====================LQR建模====================*/
	LQR_Clc(&Tl,&Tpl,&Tr,&Tpr,TargetX,0);
	
	/*====================关节力矩====================*/
	//翻滚角补偿->L0 F
	float Leg_Controller_LeftLegDeltaL0=0,Leg_Controller_RightLegDeltaL0=0;
	float Leg_Controller_LeftLegDeltaF1=0,Leg_Controller_RightLegDeltaF1=0;
	Locomotion_Controller_Roll_Control(TargetRoll,&Leg_Controller_LeftLegDeltaL0,&Leg_Controller_RightLegDeltaL0,&Leg_Controller_LeftLegDeltaF1,&Leg_Controller_RightLegDeltaF1);
	
	//腿长控制->F
	float Leg_Controller_LeftLegDeltaF2=0,Leg_Controller_RightLegDeltaF2=0;
	Leg_Controller_LeftLegControlPID.Need_Value=TargetL0+Leg_Controller_LeftLegDeltaL0;
	if(Leg_Controller_LeftLegControlPID.Need_Value>0.36f)Leg_Controller_LeftLegControlPID.Need_Value=0.36f;
	else if(Leg_Controller_LeftLegControlPID.Need_Value<0.12f)Leg_Controller_LeftLegControlPID.Need_Value=0.12f;
	if(Observer_BalanceStatus.LeftLeg.FN<FN_Threshold)Leg_Controller_LeftLegControlPID.Need_Value=0.25f;
	
	Leg_Controller_RightLegControlPID.Need_Value=TargetL0+Leg_Controller_RightLegDeltaL0;
	if(Leg_Controller_RightLegControlPID.Need_Value>0.36f)Leg_Controller_RightLegControlPID.Need_Value=0.36f;
	else if(Leg_Controller_RightLegControlPID.Need_Value<0.12f)Leg_Controller_RightLegControlPID.Need_Value=0.12f;
	if(Observer_BalanceStatus.RightLeg.FN<=FN_Threshold)Leg_Controller_RightLegControlPID.Need_Value=0.25f;
	Leg_Controller_LegControl(&Leg_Controller_LeftLegDeltaF2,&Leg_Controller_RightLegDeltaF2);

	//双腿协调->Tp
	float Locomotion_Controller_LeftLegDeltaTp=0,Locomotion_Controller_RightLegDeltaTp=0;
	Locomotion_Controller_LegCoordination_Control(&Locomotion_Controller_LeftLegDeltaTp,&Locomotion_Controller_RightLegDeltaTp);
	
	//F=F0/cos(theta)+腿长控制F+翻滚角补偿F
	//float Fl=0.5f*M*g/arm_cos_f32(Observer_BalanceStatus.LeftLeg.theta)+Leg_Controller_LeftLegDeltaF1+Leg_Controller_LeftLegDeltaF2;
	//float Fr=0.5f*M*g/arm_cos_f32(Observer_BalanceStatus.RightLeg.theta)+Leg_Controller_RightLegDeltaF1+Leg_Controller_RightLegDeltaF2;
	Fl=110.0f/arm_cos_f32(Observer_BalanceStatus.LeftLeg.theta)+Leg_Controller_LeftLegDeltaF1+Leg_Controller_LeftLegDeltaF2;
	Fr=110.0f/arm_cos_f32(Observer_BalanceStatus.LeftLeg.theta)+Leg_Controller_RightLegDeltaF1+Leg_Controller_RightLegDeltaF2;
	//Tp=LQR_Tp+双腿协调Tp
	if(Observer_BalanceStatus.LeftLeg.FN>=FN_Threshold)Tpl=Tpl+Locomotion_Controller_LeftLegDeltaTp;
	else Fl=-10+Leg_Controller_LeftLegDeltaF2;
	if(Observer_BalanceStatus.RightLeg.FN>=FN_Threshold)Tpr=Tpr+Locomotion_Controller_RightLegDeltaTp;
	else Fr=-10+Leg_Controller_RightLegDeltaF2;
	Leg_Controller_VMC(Observer_BalanceStatus.LeftLeg,Fl,Tpl,&T1l,&T2l);
	Leg_Controller_VMC(Observer_BalanceStatus.RightLeg,Fr,Tpr,&T1r,&T2r);
		
	/*====================轮向力矩====================*/
	//偏航角控制->T
	float Locomotion_Controller_LeftWheelDeltaT1=0,Locomotion_Controller_RightWheelDeltaT1=0;
	#ifndef SingleChassis
		if(Chassis_GyroScopeFlag==0)
		{
			if(Remote_RxData.Remote_Mouse_KeyR==1)
				Locomotion_Controller_Yaw_Control(Target_Yaw,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
			else
			{
				Locomotion_Controller_Yaw_Control2(YawTrack_Target,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
				Target_Yaw=Observer_BalanceStatus.Body.Yaw;
			}
		}
		else
		{
			Locomotion_Controller_Yaw_Control(Target_Yaw,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
		}
	#else
		Locomotion_Controller_Yaw_Control(Target_Yaw,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
	#endif
		
	
	
	//无用
	float Locomotion_Controller_LeftWheelDeltaT2=0,Locomotion_Controller_RightWheelDeltaT2=0;
	//Locomotion_Controller_Pitch_Control(&Locomotion_Controller_LeftWheelDeltaT2,&Locomotion_Controller_RightWheelDeltaT2);
	float Locomotion_Controller_LeftWheelDeltaT3=0,Locomotion_Controller_RightWheelDeltaT3=0;
	//Locomotion_Controller_X_Control(&Locomotion_Controller_LeftWheelDeltaT3,&Locomotion_Controller_RightWheelDeltaT3);
	
	//T=LQR_T+偏航角控制T
	if(Observer_BalanceStatus.LeftLeg.FN>=FN_Threshold)Tl=Tl+Locomotion_Controller_LeftWheelDeltaT1+Locomotion_Controller_LeftWheelDeltaT2+Locomotion_Controller_LeftWheelDeltaT3;
	if(Observer_BalanceStatus.RightLeg.FN>=FN_Threshold)Tr=Tr+Locomotion_Controller_RightWheelDeltaT1+Locomotion_Controller_RightWheelDeltaT2+Locomotion_Controller_RightWheelDeltaT3;
	
	/*====================电机控制====================*/
	Chassis_MotorControl(Tl,T1l,T2l,Tr,T1r,T2r);
	//Chassis_MotorControl(Tl,0,0,Tr,0,0);
	//Chassis_MotorControl(0,0,0,0,0,0);
	//Chassis_MotorControl(0,T1l,T2l,0,T1r,T2r);
	
	/*====================模型切换====================*/
	if(Remote_Status==1)Chassis_Model=Remote_RxData.Remote_5;
	
	
//	if(fabs(Observer_BalanceStatus.Body.Pitch)>45.0f/180.0f*PI || fabs(TargetX-Observer_BalanceStatus.Body.x)>20)
//		Chassis_Reset();
}

void Chassis_ModelJump(void)
{
	#define shoutuiTime			150
	#define shangtuiTime		120
	#define suotuiTime			100
	#define luodiTime			40
	
	float Tl=0,Tpl=0,T1l=0,T2l=0,Tr=0,Tpr=0,T1r=0,T2r=0,Fl=0,Fr=0;
	float JumpL0=0.2f;
	float JumpF0=110.0f;
	static uint8_t StartJumpFlag=0;
	
	if(STP_Distance<800)StartJumpFlag=1;
	
	if(StartJumpFlag==1)
	{
		Chassis_JumpCount++;
		
		if(Chassis_JumpCount<shoutuiTime)
		{
			PID_PositionSetParameter(&Leg_Controller_LeftLegControlPID,3000,0,300);
			PID_PositionSetParameter(&Leg_Controller_RightLegControlPID,3000,0,300);
			JumpL0=0.1f;
			JumpF0=110.0f;
		}
		else if(Chassis_JumpCount<shoutuiTime+shangtuiTime)
		{
			JumpL0=0.36f;
			JumpF0=200.0f;
		}
		else if(Chassis_JumpCount<shoutuiTime+shangtuiTime+suotuiTime)
		{
			JumpL0=0.1f;
			JumpF0=-25.0f;
		}
		else if(Chassis_JumpCount<shoutuiTime+shangtuiTime+suotuiTime+luodiTime)
		{
			JumpL0=0.2f;
			JumpF0=110.0f;
		}
		else
		{
			PID_PositionSetParameter(&Leg_Controller_LeftLegControlPID,800,0,300);
			PID_PositionSetParameter(&Leg_Controller_RightLegControlPID,800,0,300);
			Chassis_JumpFlag=0;
			Chassis_JumpCount=0;
			StartJumpFlag=0;
		}
	}
	
	
	
	
	/*====================LQR建模====================*/
	LQR_Clc(&Tl,&Tpl,&Tr,&Tpr,TargetX,0);
	
	/*====================关节力矩====================*/
	//翻滚角补偿->L0 F
	float Leg_Controller_LeftLegDeltaL0=0,Leg_Controller_RightLegDeltaL0=0;
	float Leg_Controller_LeftLegDeltaF1=0,Leg_Controller_RightLegDeltaF1=0;
	Locomotion_Controller_Roll_Control(TargetRoll,&Leg_Controller_LeftLegDeltaL0,&Leg_Controller_RightLegDeltaL0,&Leg_Controller_LeftLegDeltaF1,&Leg_Controller_RightLegDeltaF1);
	
	//腿长控制->F
	float Leg_Controller_LeftLegDeltaF2=0,Leg_Controller_RightLegDeltaF2=0;
	Leg_Controller_LeftLegControlPID.Need_Value=JumpL0+Leg_Controller_LeftLegDeltaL0;
	if(Leg_Controller_LeftLegControlPID.Need_Value>0.36f)Leg_Controller_LeftLegControlPID.Need_Value=0.36f;
	else if(Leg_Controller_LeftLegControlPID.Need_Value<0.10f)Leg_Controller_LeftLegControlPID.Need_Value=0.10f;
	Leg_Controller_RightLegControlPID.Need_Value=JumpL0+Leg_Controller_RightLegDeltaL0;
	if(Leg_Controller_RightLegControlPID.Need_Value>0.36f)Leg_Controller_RightLegControlPID.Need_Value=0.36f;
	else if(Leg_Controller_RightLegControlPID.Need_Value<0.10f)Leg_Controller_RightLegControlPID.Need_Value=0.10f;
	Leg_Controller_LegControl(&Leg_Controller_LeftLegDeltaF2,&Leg_Controller_RightLegDeltaF2);
	
	//双腿协调->Tp
	float Locomotion_Controller_LeftLegDeltaTp=0,Locomotion_Controller_RightLegDeltaTp=0;
	Locomotion_Controller_LegCoordination_Control(&Locomotion_Controller_LeftLegDeltaTp,&Locomotion_Controller_RightLegDeltaTp);

	//F=F0/cos(theta)+腿长控制F+翻滚角补偿F
	Fl=JumpF0/arm_cos_f32(Observer_BalanceStatus.LeftLeg.theta)+Leg_Controller_LeftLegDeltaF1+Leg_Controller_LeftLegDeltaF2;
	Fr=JumpF0/arm_cos_f32(Observer_BalanceStatus.RightLeg.theta)+Leg_Controller_RightLegDeltaF1+Leg_Controller_RightLegDeltaF2;
	//Tp=LQR_Tp+双腿协调Tp
	Tpl=Tpl+Locomotion_Controller_LeftLegDeltaTp;
	Tpr=Tpr+Locomotion_Controller_RightLegDeltaTp;
	Leg_Controller_VMC(Observer_BalanceStatus.LeftLeg,Fl,Tpl,&T1l,&T2l);
	Leg_Controller_VMC(Observer_BalanceStatus.RightLeg,Fr,Tpr,&T1r,&T2r);

	/*====================轮向力矩====================*/
	//偏航角控制->T
	float Locomotion_Controller_LeftWheelDeltaT1=0,Locomotion_Controller_RightWheelDeltaT1=0;
	#ifndef SingleChassis
		if(Chassis_GyroScopeFlag==0)
		{
			if(Remote_RxData.Remote_Mouse_KeyR==1)
				Locomotion_Controller_Yaw_Control(Target_Yaw,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
			else
			{
				Locomotion_Controller_Yaw_Control2(YawTrack_Target,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
				Target_Yaw=Observer_BalanceStatus.Body.Yaw;
			}
		}
		else
		{
			Locomotion_Controller_Yaw_Control(Target_Yaw,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
		}
	#else
		Locomotion_Controller_Yaw_Control(Target_Yaw,&Locomotion_Controller_LeftWheelDeltaT1,&Locomotion_Controller_RightWheelDeltaT1,w_Limit);
	#endif

	
	//T=LQR_T+偏航角控制T
	Tl=Tl+Locomotion_Controller_LeftWheelDeltaT1;
	Tr=Tr+Locomotion_Controller_RightWheelDeltaT1;
	
	Chassis_MotorControl(Tl,T1l,T2l,Tr,T1r,T2r);
}

void Chassis_ModelControl(void)
{
	if(Chassis_FirstFlag==1)
		Chassis_StandFromGround();
	else
	{
			if(Chassis_JumpFlag==0)
				Chassis_ModelTwo();
			else if(Chassis_JumpFlag==1)
				Chassis_ModelJump();
	}
}

float Accel=0;

		float PowerControl_dx,PowerControl_dy;
		float PowerControl_dw;
void Chassis_PowerControl(float *V,float *W)
{
	#define Chassis_PowerControlGainCoefficientInitialValue		0.1f//功率控制增益系数初始值
	#define Chassis_PowerControl_T								5.0f//功率控制周期(T=Mecanum_PowerControl_T*2ms)
	#define Chassis_PowerControl_UseBuffer						30.0f//功率控制消耗的缓冲能量
	#define Chassis_PowerControl_PowerMax						4.0f//功率控制功率上限与裁判系统功率上限比值上限
	#define Chassis_PowerControl_UltraCAPPower					100.0f//使用超电增加功率
	
	
	#define LiberatePower	0
	
	/*==========三轴速度获取==========*/
	float vy=(Remote_RxData.Remote_R_UD-1024)/660.0f;
	float w=(Remote_RxData.Remote_L_RL-1024)/660.0f*2.5f;
	
	float sigma=fabs(vy);//获取xy轴速度归一化系数
		
	/*==========功率上限处理==========*/
	/*由缓冲能量得到功率控制功率上限*/
	float Mecanum_PowerRef=1.0f/(60.0f-Chassis_PowerControl_UseBuffer)*RefereeSystem_Buffer;//功率增益
	if(Mecanum_PowerRef>Chassis_PowerControl_PowerMax)Mecanum_PowerRef=Chassis_PowerControl_PowerMax;
	Chassis_PowerLimit=Mecanum_PowerRef*RefereeSystem_Ref;
	
	/*由运动状态约束功率控制功率上限*/
	float Scale=sigma;
	if(Scale<1 && Scale>0)Chassis_PowerLimit*=Scale;//平移约束
	if(Chassis_GyroScopeFlag==1)Chassis_PowerLimit=Mecanum_PowerRef*RefereeSystem_Ref;//小陀螺约束
	
	#ifndef LiberatePower
		if(Chassis_PowerLimit>150.0f)Chassis_PowerLimit=150.0f;//限幅约束
	#endif
	
	/*由超电约束功率控制功率上限*/
//	if(Remote_RxData.Remote_KeyPush_Shift==1)//开启超电
//	{
//		if(Chassis_PowerLimit>0.0f)
//		{
//			if(Chassis_PowerLimit<RefereeSystem_Ref)Chassis_PowerLimit+=Chassis_PowerControl_UltraCAPPower;
//			else Chassis_PowerLimit=RefereeSystem_Ref+Chassis_PowerControl_UltraCAPPower;
//		}
//		Ultra_CAP_SetPower(RefereeSystem_Ref);
//	}
//	else
//		Ultra_CAP_SetPower(Chassis_PowerLimit);
	

	
	Chassis_PowerLimit=150;
	Chassis_PowerLimit=RefereeSystem_Ref;
	//Ultra_CAP_SetPower(RefereeSystem_Ref-5);
	
	float v_K;
	if(fabs(w)>0.1f)v_K=0.18f;
	else v_K=0.21f;
	
	if(vy>0.04f)
		(*V)=v_K*sqrtf(Chassis_PowerLimit);//为了留给转向足够的功率
	else if(vy<-0.04f)
		(*V)=-v_K*sqrtf(Chassis_PowerLimit);
	else (*V)=0;
	
	#ifndef SingleChassis
//		if(Remote_RxData.Remote_RS==2 || Remote_RxData.Remote_RS==1)Chassis_GyroScopeFlag=1;
//		else Chassis_GyroScopeFlag=0;
//	
//		if(Chassis_GyroScopeFlag==1)
//		{
//			(*V)=0;
//			
//			if(Remote_RxData.Remote_RS==2)(*W)=0.6f*sqrtf(Chassis_PowerLimit);
//			else if(Remote_RxData.Remote_RS==1)(*W)=-0.6f*sqrtf(Chassis_PowerLimit);
//		}
//		else (*W)=0;

		
		#define w_K		0.75f
		#define w_K2	0.5f
			static uint8_t Chassis_GyroScopeCloseFlag=0,Chassis_GyroScopeCloseFlag1=0;
	
			if(Remote_RxData.Remote_RS==2 || Remote_RxData.Remote_KeyPush_Ctrl==1)
			{
				PowerControl_dx=0;(*V)=0;
				PowerControl_dw=w_K*sqrtf(Chassis_PowerLimit);
				Chassis_GyroScopeFlag=1;
				Chassis_GyroScopeCloseFlag=1;
				
				w_Limit=PowerControl_dw;
			}
			else if(Chassis_GyroScopeCloseFlag==1)
			{
				PowerControl_dx=0;(*V)=0;
				float A=w_K*sqrtf(Chassis_PowerLimit)/2.0f;
				if(A>4.0f)A=4.0f;
				
				if(PowerControl_dw>A)
				{
					PowerControl_dw-=0.05f;
					w_Limit=PowerControl_dw;
				}
				else
				{
					PowerControl_dw=A;
					w_Limit=PowerControl_dw;
					
					if(GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-250 && Yaw_GM6020PositionValue-500<GM6020_MotorStatus[0].Angle)
						Chassis_GyroScopeCloseFlag=0;
				}
			}
			else if(Remote_RxData.Remote_RS==1)
			{
				PowerControl_dx=0;(*V)=0;
				PowerControl_dw=-w_K*sqrtf(Chassis_PowerLimit);
				Chassis_GyroScopeFlag=1;
				Chassis_GyroScopeCloseFlag1=1;
				
				w_Limit=-PowerControl_dw;
			}
			else if(Chassis_GyroScopeCloseFlag1==1)
			{
				PowerControl_dx=0;
				float A=w_K*sqrtf(Chassis_PowerLimit)/2.0f;
				if(A>4.0f)A=4.0f;
				
				if(PowerControl_dw<-A)
				{
					PowerControl_dw+=0.05f;
					w_Limit=-PowerControl_dw;
				}
				else
				{
					PowerControl_dw=-A;
					w_Limit=-PowerControl_dw;
					
					if(GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+250 && Yaw_GM6020PositionValue+500>GM6020_MotorStatus[0].Angle)
						Chassis_GyroScopeCloseFlag1=0;
				}
			}
			else
			{
				Chassis_GyroScopeFlag=0;
				Chassis_GyroScopeCloseFlag=0;Chassis_GyroScopeCloseFlag1=0;
			
				PowerControl_dw=0;
				
				float w_power=Chassis_PowerLimit-(fabs(Observer_BalanceStatus.Body.dx)/0.21f)*(fabs(Observer_BalanceStatus.Body.dx)/0.21f);
				if(w_power<0)w_power=0;
				float Power_wLimit=w_K2*sqrt(w_power);
				
				if(fabs(Observer_BalanceStatus.Body.dx)!=0)
				{
					w_Limit=3.0f/fabs(Observer_BalanceStatus.Body.dx);
					if(w_Limit>Power_wLimit)w_Limit=Power_wLimit;
				}
				else w_Limit=Power_wLimit;
			}	
	#else
		if(w>0.1f)
			(*W)=0.6f*sqrtf(Chassis_PowerLimit);
		else if(w<-0.1f)
			(*W)=-0.6f*sqrtf(Chassis_PowerLimit);
		else (*W)=0;
	#endif
}

void Chassis_Reset(void)
{
	TargetX=0;
	Observer_BalanceStatus.Body.MotionEstimation.x_nofilter=0;
	Observer_BalanceStatus.Body.MotionEstimation.x=0;
	Observer_BalanceStatus.Body.x_nofilter=0;
	Observer_BalanceStatus.Body.x=0;
	
	//Yaw_ControllerPositionPID.Need_Value=Observer_BalanceStatus.Body.Yaw;
	Target_Yaw=Observer_BalanceStatus.Body.Yaw;
	
	Chassis_FirstFlag=1;
	Chassis_FirstFlag2=1;
	Chassis_YawFlag=0;
	Chassis_Model=0;
	
	Chassis_PowerLimit=RefereeSystem_Ref;
	Accel=0;
	
	w_Limit=2.5f;
	
	TargetRoll=0;
	YawTrack_Target=0;
}

void Chassis_Control(void)
{
	if(Chassis_FirstFlag==0)
	{
		/*====================变腿长====================*/
		if(Remote_RxData.Remote_LS==3 && Chassis_Model==3)
		{
			if(Remote_RxData.Remote_ThumbWheel>1024+50)TargetL0+=0.3f*Observer_BalanceStatus.dt;
			else if(Remote_RxData.Remote_ThumbWheel<1024-50)TargetL0-=0.3f*Observer_BalanceStatus.dt;
		}
		if(Remote_RxData.Remote_Key_E==1)TargetL0+=0.3f*Observer_BalanceStatus.dt;
		else if(Remote_RxData.Remote_Key_X==1)TargetL0-=0.3f*Observer_BalanceStatus.dt;
		TargetL0=Data_Clipping(TargetL0,0.15f,0.36f);
		
		/*====================Roll控制====================*/
		if(RefereeSystem_Status==0)
		{
			if(Remote_RxData.Remote_LS==3 && Chassis_Model==1)
			{
				if(Remote_RxData.Remote_ThumbWheel>1024+50)TargetRoll+=0.6f*Observer_BalanceStatus.dt;
				else if(Remote_RxData.Remote_ThumbWheel<1024-50)TargetRoll-=0.6f*Observer_BalanceStatus.dt;
			}
			else TargetRoll=0;
		}
		else if(Remote_RxData.Remote_KeyPush_Ctrl==0 && Remote_RxData.Remote_KeyPush_B==0)
		{
			if(YawTrack_Target==0)
			{
				if(Remote_RxData.Remote_Key_A==1)TargetRoll-=0.6f*Observer_BalanceStatus.dt;
				else if(Remote_RxData.Remote_Key_D==1)TargetRoll+=0.6f*Observer_BalanceStatus.dt;
			}
			else if(YawTrack_Target==PI/2.0f)
			{
				if(Remote_RxData.Remote_Key_S==1)TargetRoll-=0.6f*Observer_BalanceStatus.dt;
				else if(Remote_RxData.Remote_Key_W==1)TargetRoll+=0.6f*Observer_BalanceStatus.dt;
			}
		}
		TargetRoll=Data_Clipping(TargetRoll,-RollThreshold,RollThreshold);
		
		if(Remote_RxData.Remote_Key_C==1){TargetL0=0.2f;TargetRoll=0;}
		
		/*====================功率控制====================*/
		#ifdef PowerControlFlag
			if(Remote_Status==1)
			{
				Chassis_PowerControl(&PowerControl_dx,&w_Limit);
			
				if(Chassis_Model==2)PowerControl_dx=Data_Clipping(PowerControl_dx,-2.5,2.5f);
				else PowerControl_dx=Data_Clipping(PowerControl_dx,-2.5,2.5f);
				
				if(Remote_RxData.Remote_Mouse_KeyR==1)
				{
					PowerControl_dx=0;
					if(Chassis_GyroScopeFlag==0)PowerControl_dw=0;
				}
			}
			else
			{
				PowerControl_dx=0;
				PowerControl_dw=0;
			}
		#else
			#ifndef SingleChassis
				static uint8_t Chassis_GyroScopeCloseFlag=0,Chassis_GyroScopeCloseFlag1=0;
				#define XiaoTuoluoSpeedH	15.0f
				#define XiaoTuoluoSpeedL	10.0f
				#define GuoDuSpeed			4.0f
				#define XiaoTuoluoAccel		15.0f
		
				if(Remote_RxData.Remote_RS==2 || Remote_RxData.Remote_RKnob>1600 || Remote_RxData.Remote_KeyPush_Ctrl==1 || Remote_RxData.Remote_KeyPush_B==1)
				{
					PowerControl_dx=0;
					if(Remote_RxData.Remote_LKnob>1600)
					{
						if(PowerControl_dw<XiaoTuoluoSpeedH)PowerControl_dw+=XiaoTuoluoAccel*Observer_BalanceStatus.dt;
						if(PowerControl_dw>XiaoTuoluoSpeedH)PowerControl_dw=XiaoTuoluoSpeedH;
					}
					else
					{
						if(PowerControl_dw<XiaoTuoluoSpeedL)PowerControl_dw+=XiaoTuoluoAccel*Observer_BalanceStatus.dt;
						if(PowerControl_dw>XiaoTuoluoSpeedL)PowerControl_dw=XiaoTuoluoSpeedL;
					}
					//PowerControl_dw=10.0f;
					Chassis_GyroScopeFlag=1;
					Chassis_GyroScopeCloseFlag=1;
					
					w_Limit=PowerControl_dw;
				}
				else if(Chassis_GyroScopeCloseFlag==1)
				{
					PowerControl_dx=0;
					if(PowerControl_dw>GuoDuSpeed)
					{
						PowerControl_dw-=XiaoTuoluoAccel*Observer_BalanceStatus.dt;
						w_Limit=PowerControl_dw;
					}
					else
					{
						PowerControl_dw=GuoDuSpeed;
						w_Limit=PowerControl_dw;
						
						if(GM6020_MotorStatus[0].Angle<Yaw_GM6020PositionValue-100 && Yaw_GM6020PositionValue-500<GM6020_MotorStatus[0].Angle)
							Chassis_GyroScopeCloseFlag=0;
					}
				}
				else if(Remote_RxData.Remote_RS==1)
				{
					PowerControl_dx=0;
					PowerControl_dw=-5.0f;
					Chassis_GyroScopeFlag=1;
					Chassis_GyroScopeCloseFlag1=1;
					
					w_Limit=-PowerControl_dw;
				}
				else if(Chassis_GyroScopeCloseFlag1==1)
				{
					PowerControl_dx=0;
					if(PowerControl_dw<-4.0f)
					{
						PowerControl_dw+=0.05f;
						w_Limit=-PowerControl_dw;
					}
					else
					{
						PowerControl_dw=-4.0f;
						w_Limit=-PowerControl_dw;
						
						if(GM6020_MotorStatus[0].Angle>Yaw_GM6020PositionValue+100 && Yaw_GM6020PositionValue+500>GM6020_MotorStatus[0].Angle)
							Chassis_GyroScopeCloseFlag1=0;
					}
				}
				else
				{
					Chassis_GyroScopeFlag=0;
					Chassis_GyroScopeCloseFlag=0;
				
					PowerControl_dw=0;
					if(fabs(Observer_BalanceStatus.Body.dx)!=0)
					{
						w_Limit=3.0f/fabs(Observer_BalanceStatus.Body.dx);
						if(w_Limit>4.0f)w_Limit=4.0f;
					}
					else w_Limit=4.0f;
				}
				
				if(Remote_RxData.Remote_LKnob>1600)
				{
					PowerControl_dx=(Remote_RxData.Remote_R_UD-1024)/660.0f*2.5f;
					PowerControl_dy=(Remote_RxData.Remote_R_RL-1024)/660.0f*2.5f;
				}
				else if(Remote_RxData.Remote_LKnob<-3200)
				{
					PowerControl_dx=(Remote_RxData.Remote_R_UD-1024)/660.0f*1.5f;
					PowerControl_dy=(Remote_RxData.Remote_R_RL-1024)/660.0f*1.5f;
				}
				else
				{
					PowerControl_dx=(Remote_RxData.Remote_R_UD-1024)/660.0f*2.3f;
					PowerControl_dy=(Remote_RxData.Remote_R_RL-1024)/660.0f*2.3f;
				}
			#else
				if(Chassis_Model==2)PowerControl_dx=(Remote_RxData.Remote_R_UD-1024)/660.0f*2.3f;
				else PowerControl_dx=(Remote_RxData.Remote_R_UD-1024)/660.0f*2.3f;
			
				PowerControl_dw=(Remote_RxData.Remote_L_RL-1024)/660.0f*2.5f;
				w_Limit=2.5f;
			#endif
		#endif
				
		Ultra_CAP_SetPower(RefereeSystem_Ref,RefereeSystem_Buffer,ENABLE);
		
		/*====================移动&转向====================*/
		//if(Remote_RxData.Remote_R_UD<1000 || Remote_RxData.Remote_R_UD>1050)
		{
			TargetdX=PowerControl_dx*arm_cos_f32(Observer_BalanceStatus.Body.Yaw_Theta-PowerControl_dw*(-0.024f*fabs(PowerControl_dw)+0.62f))-PowerControl_dy*arm_sin_f32(Observer_BalanceStatus.Body.Yaw_Theta-PowerControl_dw*(-0.024f*fabs(PowerControl_dw)+0.62f));
			TargetX+=TargetdX*Observer_BalanceStatus.dt;
		}
		//if(Remote_RxData.Remote_L_RL<1000 || Remote_RxData.Remote_L_RL>1050)
			//Yaw_ControllerPositionPID.Need_Value-=((Remote_RxData.Remote_L_RL-1024)/660.0f*0.005f);
			Target_Yaw-=PowerControl_dw*Observer_BalanceStatus.dt;
		
		/*====================跳跃键====================*/
		static uint8_t Last_RemoteLS=3;
		if(Last_RemoteLS==2 && Remote_RxData.Remote_LS==3)
		{
			Chassis_JumpFlag=1;
			Chassis_JumpCount=0;
		}
		Last_RemoteLS=Remote_RxData.Remote_LS;

		/*====================横车====================*/
		static uint8_t Last_Remote_5=3,Last_Remote_R=0;
		if((Last_Remote_5==2 && Remote_RxData.Remote_5==3) || (Last_Remote_R==0 && Remote_RxData.Remote_Key_R==1))
		{
			if(YawTrack_Target==PI/2.0f)YawTrack_Target=0;
			else if(YawTrack_Target==0)YawTrack_Target=PI/2.0f;
		}
		Last_Remote_5=Remote_RxData.Remote_5;
		Last_Remote_R=Remote_RxData.Remote_Key_R;
	}
	
	/*====================控制====================*/
	Chassis_ModelControl();
}
