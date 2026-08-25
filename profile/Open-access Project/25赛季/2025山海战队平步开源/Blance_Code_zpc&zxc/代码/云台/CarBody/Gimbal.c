#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Parameter.h"
#include "PID.h"
#include "Remote.h"
#include "AttitudeAlgorithms.h"
#include "M3508.h"
#include "M2006.h"
#include "GM6020.h"
#include "Laser.h"
#include "RefereeSystem.h"
#include "Visual.h"
#include "DM_J4310.h"
#include "Gimbal.h"
#include "arm_math.h"
#include "Delay.h"
#include "Visual.h"
#include "CAN.h"
#include "ins_task.h"
#include "LinkCheck.h"

uint8_t Gimbal_FrictionWheelFlag;//云台小陀螺标志位,云台开摩擦轮标志位

PID_PositionInitTypedef Gimbal_YawAnglePositionPID,Gimbal_YawAngleSpeedPID;//Yaw轴GM6020电机PID
PID_PositionInitTypedef Gimbal_PitchAnglePositionPID,Gimbal_PitchAngleSpeedPID;//Pitch轴GM6020电机PID
PID_PositionInitTypedef Gimbal_L_FrictionWheelPID,Gimbal_R_FrictionWheelPID;//摩擦轮转速PID
PID_PositionInitTypedef Gimbal_RammerSpinSpeedPID;//拨弹盘旋转PID

float Pitch_TargetTheta=0;
float Yaw_TargetTheta=0;
float FiringMechanismL_TargetSpeed=0,FiringMechanismR_TargetSpeed=0;
float Rammer_TargetTheta=0;
float Rammer_TargetSpeed=0;

/*
 *函数简介:云台初始化
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Gimbal_Init(void)
{
//	PID_PositionStructureInit(&Gimbal_YawAnglePositionPID,0);//Yaw轴陀螺仪闭环
//	PID_PositionSetParameter(&Gimbal_YawAnglePositionPID,10,0,20);
//	PID_PositionSetEkRange(&Gimbal_YawAnglePositionPID,-1,1);
//	PID_PositionSetOUTRange(&Gimbal_YawAnglePositionPID,-200,200);
//	PID_PositionStructureInit(&Gimbal_YawAngleSpeedPID,200);
//	PID_PositionSetParameter(&Gimbal_YawAngleSpeedPID,200,0,10);
//	PID_PositionSetEkRange(&Gimbal_YawAngleSpeedPID,-5,5);
//	PID_PositionSetOUTRange(&Gimbal_YawAngleSpeedPID,-30000,30000);
//	
//	PID_PositionStructureInit(&Gimbal_PitchAnglePositionPID,0);//Pitch轴陀螺仪闭环
//	PID_PositionSetParameter(&Gimbal_PitchAnglePositionPID,6,0,0);
//	PID_PositionSetEkRange(&Gimbal_PitchAnglePositionPID,-1,1);
//	PID_PositionSetOUTRange(&Gimbal_PitchAnglePositionPID,-150,150);
//	PID_PositionStructureInit(&Gimbal_PitchAngleSpeedPID,150);
//	PID_PositionSetParameter(&Gimbal_PitchAngleSpeedPID,50,1,0);
//	PID_PositionSetEkRange(&Gimbal_PitchAngleSpeedPID,-5,5);
//	PID_PositionSetOUTRange(&Gimbal_PitchAngleSpeedPID,-30000,30000);
//	
//	PID_PositionStructureInit(&Gimbal_L_FrictionWheelPID,0);//左摩擦轮
//	PID_PositionSetParameter(&Gimbal_L_FrictionWheelPID,16,0,30);
//	PID_PositionSetEkRange(&Gimbal_L_FrictionWheelPID,-5,5);
//	PID_PositionSetOUTRange(&Gimbal_L_FrictionWheelPID,-15000,15000);
//	PID_PositionStructureInit(&Gimbal_R_FrictionWheelPID,0);//右摩擦轮
//	PID_PositionSetParameter(&Gimbal_R_FrictionWheelPID,16,0,30);
//	PID_PositionSetEkRange(&Gimbal_R_FrictionWheelPID,-5,5);
//	PID_PositionSetOUTRange(&Gimbal_R_FrictionWheelPID,-15000,15000);

//	PID_PositionStructureInit(&Gimbal_RammerSpinSpeedPID,-Gimbal_RammerSpeed);//拨弹盘
//	PID_PositionSetParameter(&Gimbal_RammerSpinSpeedPID,16,0,0);
//	PID_PositionSetEkRange(&Gimbal_RammerSpinSpeedPID,-20,20);
//	PID_PositionSetOUTRange(&Gimbal_RammerSpinSpeedPID,-30000,30000);
	
	DM_J4310_Init();
	Laser_Init();
}

/*
 *函数简介:云台PID清理
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Gimbal_CleanPID(void)
{
//	PID_PositionClean(&Gimbal_YawAnglePositionPID);
//	PID_PositionClean(&Gimbal_YawAngleSpeedPID);
//	PID_PositionClean(&Gimbal_PitchAnglePositionPID);
//	PID_PositionClean(&Gimbal_PitchAngleSpeedPID);
//	PID_PositionClean(&Gimbal_L_FrictionWheelPID);
//	PID_PositionClean(&Gimbal_R_FrictionWheelPID);
//	PID_PositionClean(&Gimbal_RammerSpinSpeedPID);
}

uint8_t Fire_Flag=0;
uint8_t UI_Flag=0;

/*
 *函数简介:云台Pitch轴控制
 *参数说明:无
 *返回类型:无
 *备注:根据拨杆或鼠标获得俯仰角度,映射比例在上方宏定义Gimbal_LeverSpeedMapRate更改
 *备注:俯仰限幅由结构决定,参数由Parameter.h文件中的Pitch_GM6020PositionLowerLinit和Pitch_GM6020PositionUpperLinit决定
 *备注:俯仰轴GM6020报文标识符和M2006高位ID一致,故均在拨弹盘控制函数中统一发送控制报文
 *备注:在此函数中进行了视觉自瞄处理,由于视觉组摆烂,并没有开发出自瞄,也没用进行过联调,故自瞄部分没有拆出去独立函数
 */
void Gimbal_PitchControl(void)
{
	#define LQR_K1		22.3607f
	#define LQR_K2		0.7635f//22.3607    0.7635
//	#define LQR_K1		21.4955f
//	#define LQR_K2		1.2101f//21.4955    1.2101

	static uint16_t Count=0;
	static uint8_t PitchFlag=0;
	static float Sentry_t=0;
	
	if(PitchFlag==1)PitchFlag=0;
	else if(Remote_StartFlag==2)//遥控器刚建立连接时,复位Pitch轴角度并清理Pitch轴位置环积分项
	{
		Count=0;
		Pitch_TargetTheta=0.0f;//AttitudeAlgorithms_RadRoll;
	}
	
	if(Remote_RxData.Remote_RKnob>1600 || Remote_RxData.Remote_KeyPush_B==1)
	{
		static uint8_t PCount=0;
		
		if(Visual_ReceiveFlag==1 && (Visual_Pitch!=0 || Visual_Yaw!=0))//自瞄,补偿角度
		{
			UI_Flag=1;
			Fire_Flag=1;
			Pitch_TargetTheta=Visual_Pitch;
			PCount=0;
		}
		else
		{
			PCount++;
			
			if(PCount>3)
			{
				Pitch_TargetTheta=0.5f*(SentryModel_PitchUpperLinit+Pitch_GM6020AngleLowerLinit)+0.5f*(SentryModel_PitchUpperLinit-Pitch_GM6020AngleLowerLinit)*arm_sin_f32(Sentry_t);
				//Pitch_TargetTheta=Pitch_GM6020AngleLowerLinit*arm_sin_f32(Sentry_t);
				Sentry_t+=SentryModel_PitchSpeed;
			}
		}
	}
	else
	{
		Sentry_t=0;
		
		if(Remote_Status==0){PitchFlag=1;}
		else if(CAN_CAN1IDList[0][3]==1)
		{
			if(((Remote_RxData.Remote_L_UD>1050 && RefereeSystem_Status==0) || (1024+Remote_RxData.Remote_Mouse_DU*3)<1024) && DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Position>Pitch_GM6020AngleUpperLinit)
			{
				if(PC_Pitch==0)Pitch_TargetTheta-=Gimbal_LeverSpeedMapRate*0.0439453125f*((Remote_RxData.Remote_L_UD-1024)/660.0f)/180.0f*3.1415926f;
				else Pitch_TargetTheta+=Gimbal_LeverSpeedMapRate*0.0439453125f*(PC_Pitch*PC_Mouse_DUSensitivity/660.0f*2)/180.0f*3.1415926f;
			}
			else if(((Remote_RxData.Remote_L_UD<1000 && RefereeSystem_Status==0) || (1024+Remote_RxData.Remote_Mouse_DU*3)>1024) && DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Position<Pitch_GM6020AngleLowerLinit)
			{
				if(PC_Pitch==0)Pitch_TargetTheta+=Gimbal_LeverSpeedMapRate*0.0439453125f*((1024-Remote_RxData.Remote_L_UD)/660.0f)/180.0f*3.1415926f;
				else Pitch_TargetTheta+=Gimbal_LeverSpeedMapRate*0.0439453125f*(PC_Pitch*PC_Mouse_DUSensitivity/660.0f*2)/180.0f*3.1415926f;
			}
			
			#define Pitch_Duzhuan_theta		(10.0f/180.0f*PI)
			#define Pitch_Duzhuan_speed		0.5f
			#define Pitch_Duzhuan_Count		500
			static uint16_t Duzhuan_Count=0;
			
			if(fabs(Pitch_TargetTheta-AttitudeAlgorithms_RadPitch)>Pitch_Duzhuan_theta && \
			   fabs(DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Speed)<Pitch_Duzhuan_speed)
			{
				Duzhuan_Count++;
				if(Duzhuan_Count>Pitch_Duzhuan_Count)
				{
					Pitch_TargetTheta=AttitudeAlgorithms_RadPitch;
					Duzhuan_Count=0;
				}
			}
			else Duzhuan_Count=0;
		
			if(Remote_RxData.Remote_Mouse_KeyR==1 && Visual_ReceiveFlag==1 && (Visual_Pitch!=0 || Visual_Yaw!=0))//自瞄,补偿角度
			{
				Visual_ReceiveFlag=0;
				UI_Flag=1;
				Fire_Flag=2;
				Pitch_TargetTheta=Visual_Pitch;
				if(Pitch_TargetTheta<Pitch_GM6020AngleUpperLinit)Pitch_TargetTheta=Pitch_GM6020AngleUpperLinit;
				if(Pitch_TargetTheta>Pitch_GM6020AngleLowerLinit)Pitch_TargetTheta=Pitch_GM6020AngleLowerLinit;
				//Gimbal_YawAnglePositionPID.Need_Value=Visual_Yaw;
				Yaw_TargetTheta=Visual_Yaw/180.0f*3.1415926f;
			}
		}
	}
	
//	if(Pitch_TargetTheta<Pitch_GM6020AngleUpperLinit)Pitch_TargetTheta=Pitch_GM6020AngleUpperLinit;
//	if(Pitch_TargetTheta>Pitch_GM6020AngleLowerLinit)Pitch_TargetTheta=Pitch_GM6020AngleLowerLinit;
	
	//float tau=LQR_K1*(Pitch_TargetTheta-DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Position)-LQR_K2*DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Speed;
	float tau=LQR_K1*(Pitch_TargetTheta-AttitudeAlgorithms_RadPitch)-LQR_K2*INS.Gyro[0];
	
	tau-=0.6f*arm_cos_f32(DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Position);//重力前馈
	if(tau>5.0f)tau=5.0f;
	if(tau<-5.0f)tau=-5.0f;
	
	if(CAN_CAN1IDList[0][3]==0)tau=0;
	//if(Count>5)DM_J4310_Set(Gimbal_PitchMotor,0);
	if(Count>250)DM_J4310_Set(Gimbal_PitchMotor,tau);
	else
	{
		Count++;
		DM_J4310_Set(Gimbal_PitchMotor,0);
	}
	
	uint8_t Start[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	static uint16_t EnableCount=0;
	EnableCount++;
	if(EnableCount==100 && DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Status==0x00)DM_J4310_CANSend(Gimbal_PitchMotor,Start);
	else if(EnableCount==200)EnableCount=0;
}

/*
 *函数简介:云台Yaw轴控制
 *参数说明:无
 *返回类型:无
 *备注:根据拨杆或鼠标获得偏航角度,映射比例在上方宏定义Gimbal_LeverSpeedMapRate和Gimbal_YawPitchSpeedRate更改
 *备注:由于云台一直根据陀螺仪角度闭环,不需要考虑小陀螺问题
 */
void Gimbal_YawControl(void)
{
	#define LQR_K3		22.3607f
	#define LQR_K4		2.5232f//22.3607    2.3187
	
	static uint8_t YawFlag=0;
	
	if(YawFlag==1)YawFlag=0;
	else if(Remote_StartFlag==2)//遥控器刚建立连接时,复位Pitch轴角度并清理Pitch轴位置环积分项
	{
		Yaw_TargetTheta=AttitudeAlgorithms_RadYaw;
		Rammer_TargetTheta=M2006_MotorStatus[6].ShaftPosition/180.0f*PI;
		Rammer_TargetSpeed=0;
	}
	
	if(Remote_RxData.Remote_RKnob>1600 || Remote_RxData.Remote_KeyPush_B==1)
	{
		static uint8_t YCount=0;
		
		if(Visual_ReceiveFlag==1 && (Visual_Pitch!=0 || Visual_Yaw!=0))//自瞄,补偿角度
		{
			Visual_ReceiveFlag=0;
			Yaw_TargetTheta=Visual_Yaw/180.0f*3.1415926f;
			YCount=0;
		}
		else
		{
			YCount++;
			
			if(YCount>3)
			{
				Yaw_TargetTheta-=SentryModel_YawSpeed;
			}
		}
	}
	else
	{
		if(Remote_Status==0){YawFlag=1;}
		else if(CAN_CAN2IDList[0][3]==1)
		{
			if((Remote_RxData.Remote_L_RL>1050 && RefereeSystem_Status==0) || 1024+PC_Spin*PC_Mouse_RLSensitivity>1024)//根据摇杆改变偏航
			{
				if(PC_Spin==0)Yaw_TargetTheta-=Gimbal_LeverSpeedMapRate*Gimbal_YawPitchSpeedRate*0.0439453125f*((Remote_RxData.Remote_L_RL-1024)/660.0f)/180.0f*3.1415926f;
				else Yaw_TargetTheta-=Gimbal_LeverSpeedMapRate*Gimbal_YawPitchSpeedRate*0.0439453125f*(PC_Spin*PC_Mouse_RLSensitivity/660.0f*2)/180.0f*3.1415926f;
			}
			else if((Remote_RxData.Remote_L_RL<1000 && RefereeSystem_Status==0) || 1024+PC_Spin*PC_Mouse_RLSensitivity<1024)
			{
				if(PC_Spin==0)Yaw_TargetTheta+=Gimbal_LeverSpeedMapRate*Gimbal_YawPitchSpeedRate*0.0439453125f*((1024-Remote_RxData.Remote_L_RL)/660.0f)/180.0f*3.1415926f;
				else Yaw_TargetTheta-=Gimbal_LeverSpeedMapRate*Gimbal_YawPitchSpeedRate*0.0439453125f*(PC_Spin*PC_Mouse_RLSensitivity/660.0f*2)/180.0f*3.1415926f;
			}
				
			static uint8_t F_LastStatus=0;
			if(F_LastStatus==1 && Remote_RxData.Remote_Key_F==0)Yaw_TargetTheta-=PI;
			F_LastStatus=Remote_RxData.Remote_Key_F;
		}
	}
	
	float tau=LQR_K3*(Yaw_TargetTheta-AttitudeAlgorithms_RadYaw)-LQR_K4*AttitudeAlgorithms_RaddYaw_E;
	float Current=tau/0.741f/3.0f*16384.0f;
	if(Current>16384)Current=16384;
	if(Current<-16384)Current=-16384;
	if(CAN_CAN2IDList[0][3]==0)Current=0;
	GM6020_CAN2SetLIDCurrent(Current,0,0,0);
}

uint8_t Gimbal_Q_FlagStatus=0;//0-空闲
uint8_t Status2_Count=0;
float Gimbal_Q=0;
/*
 *函数简介:摩擦轮控制
 *参数说明:无
 *返回类型:无
 *备注:遥控左拨动开关向上拨(Remote_LS=1)开摩擦轮,摩擦轮打开的同时会打开激光
 */
void Gimbal_FiringMechanismControl(void)
{
	static uint8_t Gimbal_FiringMechanismStart=0;
	
	#define LQR_K5		0.0707f
	
	if(Remote_RxData.Remote_RKnob>1600 || Remote_RxData.Remote_KeyPush_B==1)PC_FrictionWheel=1;
	
	if(Remote_Status==0){}
	else
	{
		if(((Remote_RxData.Remote_LS==1 && RefereeSystem_Status==0) || PC_FrictionWheel==1) && RefereeSystem_ShooterStatus==1 && CAN_CAN1IDList[2][3]==1 && CAN_CAN1IDList[3][3]==1)//摩擦轮开
		{
			FiringMechanismL_TargetSpeed=Gimbal_FrictionWheelSpeed;FiringMechanismR_TargetSpeed=-Gimbal_FrictionWheelSpeed;
			Laser_ON();//开激光
			Gimbal_FrictionWheelFlag=1;
		}
		else//摩擦轮关
		{
			FiringMechanismL_TargetSpeed=FiringMechanismR_TargetSpeed=0;
			Laser_OFF();//关激光
			Gimbal_FrictionWheelFlag=0;
		}
	}
	
	float Speed_Ek_L=FiringMechanismL_TargetSpeed-M3508_MotorStatus[Gimbal_L_FrictionWheel-0x201].RotorSpeed/60.0f*2.0f*PI;
	float tauL=LQR_K5*Speed_Ek_L;
	float Speed_Ek_R=FiringMechanismR_TargetSpeed-M3508_MotorStatus[Gimbal_R_FrictionWheel-0x201].RotorSpeed/60.0f*2.0f*PI;
	float tauR=LQR_K5*Speed_Ek_R;
	float CurrentL=tauL/0.3f/20.0f*16384.0f;
	if(CurrentL>16384)CurrentL=16384;
	if(CurrentL<-16384)CurrentL=-16384;
	float CurrentR=tauR/0.3f/20.0f*16384.0f;
	if(CurrentR>16384)CurrentR=16384;
	if(CurrentR<-16384)CurrentR=-16384;
	if(CAN_CAN1IDList[2][3]==0 || CAN_CAN1IDList[3][3]==0)CurrentL=CurrentR=0;
	else M3508_CANSetLIDCurrent(CurrentL,CurrentR,0,0);
	
	
	
	#define Status0_SpeedEkThreshold		(200.0f/60.0f*2.0f*PI)	//rpm->rad/s
	#define Status1_CurrentThreshold		(1.4f)					//A
	#define Status2_TimeThreshold			(15.0f/2.0f)			//ms->count
		
	if(Gimbal_Q_FlagStatus==0)//空闲
	{
		if(Gimbal_FiringMechanismStart==1 && \
		   fabs(Speed_Ek_L)<Status0_SpeedEkThreshold && \
		   fabs(Speed_Ek_R)<Status0_SpeedEkThreshold)
			Gimbal_Q_FlagStatus=1;
	}
	else if(Gimbal_Q_FlagStatus==1)//等待电流尖峰
	{
		if(fabs(M3508_MotorStatus[Gimbal_L_FrictionWheel-0x201].Current)>Status1_CurrentThreshold && \
		   fabs(M3508_MotorStatus[Gimbal_R_FrictionWheel-0x201].Current)>Status1_CurrentThreshold)
			Gimbal_Q_FlagStatus=2;
		
		if(Gimbal_FiringMechanismStart==0)Gimbal_Q_FlagStatus=0;
		
		Status2_Count=0;
	}
	else if(Gimbal_Q_FlagStatus==2)//时间判断
	{
		Status2_Count++;
		
		if(fabs(M3508_MotorStatus[Gimbal_L_FrictionWheel-0x201].Current)<Status1_CurrentThreshold || \
		   fabs(M3508_MotorStatus[Gimbal_R_FrictionWheel-0x201].Current)<Status1_CurrentThreshold)
			Gimbal_Q_FlagStatus=1;
		
		if(Status2_Count>=Status2_TimeThreshold)
		{
			Gimbal_Q+=10;
			
			Gimbal_Q_FlagStatus=1;
		}
	}
	
	Gimbal_Q-=RefereeSystem_Cool*0.002f;
	if(Gimbal_Q<0)Gimbal_Q=0;
	
	if(RefereeSystem_Q_Flag==1)
	{
		RefereeSystem_Q_Flag=0;
		Gimbal_Q=RefereeSystem_Q;
	}
	Gimbal_Q=0;
}

float Danfa_TargetTheta;
/*
 *函数简介:拨弹盘控制
 *参数说明:无
 *返回类型:无
 *备注:俯仰轴GM6020报文标识符和M2006高位ID一致,故均在拨弹盘控制函数中统一发送控制报文
 */
void Gimbal_Rammer(void)
{
	#define LQR_K7		10.0000f
	
	static uint16_t Q_Count=0;
	#define Q_Count_T		250
	#define Q_Limit_Retain	40
	
	if(Remote_RxData.Remote_Mouse_KeyR==1)Remote_RxData.Remote_KeyPush_G=0;
	static uint8_t Last_PushG=0;
	if(Last_PushG==0 && Remote_RxData.Remote_KeyPush_G==1)Danfa_TargetTheta=M2006_MotorStatus[6].ShaftPosition/180.0f*3.1415926f;
	Last_PushG=Remote_RxData.Remote_KeyPush_G;
		
	if(Remote_RxData.Remote_KeyPush_G==0)
	{
		if(Remote_Status==0)Rammer_TargetSpeed=0;
		else
		{
			if(Gimbal_FrictionWheelFlag==1 && CAN_CAN1IDList[1][3]==1)
			{
				if(Fire_Flag>0)
				{
					Fire_Flag--;
					
					if(Visual_Fire==1)
					{
						if(Gimbal_Q>RefereeSystem_QLimit-Q_Limit_Retain)Q_Count=Q_Count_T;
						
						if(Q_Count==0)Rammer_TargetSpeed=-Gimbal_RammerSpeed;
						else{Q_Count--;Rammer_TargetSpeed=0;}
					}
					else
						Rammer_TargetSpeed=0;
				}
				else
				{
					if((Remote_RxData.Remote_ThumbWheel<1000 && RefereeSystem_Status==0) || PC_Fire==1)
					{
						if(Gimbal_Q>RefereeSystem_QLimit-Q_Limit_Retain)Q_Count=Q_Count_T;
						
						if(Q_Count==0)Rammer_TargetSpeed=-Gimbal_RammerSpeed;
						else{Q_Count--;Rammer_TargetSpeed=0;}
					}
					else
						Rammer_TargetSpeed=0;
	//	//			else if((Remote_RxData.Remote_ThumbWheel>1050 && RefereeSystem_Status==0) || PC_Ejection==1)
	//	//				Rammer_TargetTheta+=2*PI/Gimbal_RammerCount;
				}
			}
		}
		
		float tau=LQR_K7*(Rammer_TargetSpeed-M2006_MotorStatus[6].ShaftSpeed/60.0f*2.0f*PI);
		float Current=(tau/36.0f)/0.18f/10.0f*10000.0f;
		if(Current>10000)Current=10000;
		if(Current<-10000)Current=-10000;
		if(Gimbal_FrictionWheelFlag==0 || CAN_CAN1IDList[1][3]==0)Current=0;
		M2006_CANSetHIDCurrent(0,0,Current,0);
	}
	else
	{
		#define Danfa_K7		22.3607f
		#define Danfa_K8		22.3607f
		
		static uint8_t LastZuojian=0;
		if(LastZuojian==0 && Remote_RxData.Remote_Mouse_KeyL==1)Danfa_TargetTheta-=45.0f/180.0f*3.1415926f;
		LastZuojian=Remote_RxData.Remote_Mouse_KeyL;
		
		float danfa_v=Danfa_K7*(Danfa_TargetTheta-M2006_MotorStatus[6].ShaftPosition/180.0f*3.1415926f);
		if(danfa_v>10)danfa_v=10;
		if(danfa_v<-10)danfa_v=-10;
		
		float danfa_tau=Danfa_K8*(danfa_v-M2006_MotorStatus[6].ShaftSpeed/60.0f*2.0f*3.1415926f);
		float Current=(danfa_tau/36.0f)/0.18f/10.0f*10000.0f;
		if(Current>10000)Current=10000;
		if(Current<-10000)Current=-10000;
		
		if(Gimbal_FrictionWheelFlag==0){Current=0;}
		
		M2006_CANSetHIDCurrent(0,0,Current,0);
	}
} 

/*
 *函数简介:云台运动控制
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Gimbal_MoveControl(void)
{
	Gimbal_PitchControl();//云台Pitch轴控制
	
	Delay_us(200);
	
	Gimbal_YawControl();//云台Yaw轴控制
	
	Delay_us(200);
	
	Gimbal_FiringMechanismControl();//摩擦轮控制
	Gimbal_Rammer();//拨弹盘控制
}
