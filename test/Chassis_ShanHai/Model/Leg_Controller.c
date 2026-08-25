#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "arm_math.h"
#include "Leg_Controller.h"
#include "Parameter.h"

PID_PositionInitTypedef Leg_Controller_LeftLegControlPID,Leg_Controller_RightLegControlPID;

/*
 *函数简介:腿长控制VMC
 *参数说明:腿部状态结构体
 *参数说明:摆杆推力F
 *参数说明:摆杆扭矩Tp
 *参数说明:髋关节1转矩T1
 *参数说明:髋关节2转矩T2
 *返回类型:无
 *备注:无
 */
void Leg_Controller_VMC(Observer_LegStatus Leg,float F,float Tp,float *T1,float *T2)
{	
	(*T1)=(Leg.J_11)*F+(Leg.J_21)*Tp;
	(*T2)=(Leg.J_12)*F+(Leg.J_22)*Tp;
}

/*
 *函数简介:腿长控制初始化
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Leg_Controller_LegControlInit(void)
{
	PID_PositionStructureInit(&Leg_Controller_LeftLegControlPID,0.20f);
	PID_PositionSetParameter(&Leg_Controller_LeftLegControlPID,800,0,300);
	PID_PositionSetEkRange(&Leg_Controller_LeftLegControlPID,-0.01f,0.01f);
	//PID_PositionSetEkSumRange(&Leg_Controller_LeftLegControlPID,-0.05f,0.05f);
	PID_PositionSetOUTRange(&Leg_Controller_LeftLegControlPID,-100,100);

	PID_PositionStructureInit(&Leg_Controller_RightLegControlPID,0.20f);
	PID_PositionSetParameter(&Leg_Controller_RightLegControlPID,800,0,300);
	PID_PositionSetEkRange(&Leg_Controller_RightLegControlPID,-0.01f,0.01f);
	//PID_PositionSetEkSumRange(&Leg_Controller_RightLegControlPID,-0.05f,0.05f);
	PID_PositionSetOUTRange(&Leg_Controller_RightLegControlPID,-100,100);
}	

/*
 *函数简介:腿长控制
 *参数说明:左腿ΔF
 *参数说明:右腿ΔF
 *返回类型:无
 *备注:无
 */
void Leg_Controller_LegControl(float *LeftLeg_DeltaF,float *RightLeg_DeltaF)
{
//	PID_PositionCalc(&Leg_Controller_LeftLegControlPID,Observer_BalanceStatus.LeftLeg.L_0);
//	PID_PositionCalc(&Leg_Controller_RightLegControlPID,Observer_BalanceStatus.RightLeg.L_0);
	PID_PositionCalcGivenDot(&Leg_Controller_LeftLegControlPID,Observer_BalanceStatus.LeftLeg.L_0,Observer_BalanceStatus.LeftLeg.dL_0);
	PID_PositionCalcGivenDot(&Leg_Controller_RightLegControlPID,Observer_BalanceStatus.RightLeg.L_0,Observer_BalanceStatus.RightLeg.dL_0);
	
	(*LeftLeg_DeltaF)=Leg_Controller_LeftLegControlPID.OUT;
	(*RightLeg_DeltaF)=Leg_Controller_RightLegControlPID.OUT;
}

/*
 *函数简介:腿长控制逆运动学解算
 *参数说明:摆杆长度L0
 *参数说明:摆杆角度phi0
 *参数说明:五连杆phi1
 *参数说明:五连杆phi4
 *返回类型:无
 *备注:无
 */
void Leg_Controller_InverseKinematicsSolution(float L_0,float phi_0,float *phi_1,float *phi_4)
{
	float XC=l_5/2.0f+L_0*cosf(phi_0);
	float YC=L_0*sinf(phi_0);
	
	float A=l_1+XC;
	float B=l_1*l_1-XC*XC;
	float C=l_2*l_2-YC*YC;
	float D=l_2*l_2+YC*YC;
	
	float phi1=2.0f*atan2f(2.0f*l_1*YC+sqrtf(2.0f*l_1*l_1*D+2.0f*XC*XC*C-B*B-C*C),A*A-C);
	if(phi1>2.0f*PI)phi1-=2.0f*PI;
	if(phi1<0)phi1+=2.0f*PI;
	
	A=l_3+l_4;
	B=l_5-XC;
	C=l_3-l_4;
	D=XC+l_4-l_5;
	
	float E=(A*A-B*B-YC*YC);
	float F=(B*B-C*C+YC*YC);
	float phi4=2.0f*atan2f(2.0f*l_4*YC-sqrtf(E*F),D*D+YC*YC-l_3*l_3);
	if(phi4>PI)phi4-=2.0f*PI;
	if(phi4<-PI)phi4+=2.0f*PI;

	(*phi_1)=phi1;
	(*phi_4)=phi4;
}

/*
 *函数简介:腿长控制LQR控制腿长
 *参数说明:腿部状态结构体
 *参数说明:目标五连杆phi1
 *参数说明:目标五连杆phi4
 *参数说明:髋关节1转矩T1
 *参数说明:髋关节2转矩T2
 *返回类型:无
 *备注:无
 */
void Leg_Controller_LengthLQR(Observer_LegStatus Leg,float target_phi1,float target_phi4,float *T1,float *T2)
{
	#define Leg_Controller_LQR_FeedForward	5.4f//3.8f
	#define Leg_Controller_LQR_K1			31.6228f
	#define Leg_Controller_LQR_K2			1.2142f//31.6228    1.2142
	
	(*T1)=-Leg_Controller_LQR_FeedForward-Leg_Controller_LQR_K1*(Leg.phi_1-target_phi1)-Leg_Controller_LQR_K2*Leg.dphi_1;
	(*T2)=Leg_Controller_LQR_FeedForward-Leg_Controller_LQR_K1*(Leg.phi_4-target_phi4)-Leg_Controller_LQR_K2*Leg.dphi_4;
}
