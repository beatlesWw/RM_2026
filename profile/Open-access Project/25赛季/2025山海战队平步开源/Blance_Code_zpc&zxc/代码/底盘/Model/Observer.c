#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "arm_math.h"
#include "Parameter.h"
#include "DM_J8009.h"
#include "MF9025.h"
#include "Observer.h"
#include "AttitudeAlgorithms.h"
#include "GM6020.h"

Observer_Balance Observer_BalanceStatus;

/*
 *函数简介:观测器初始化
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Observer_Init(void)
{
	Observer_BalanceStatus.dt=0.002f;
	MotionEstimation_Init();
}

/*
 *函数简介:轮子状态获取
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Observer_GetWheelStatus(void)
{
	MF9025_CANGetStatus(Chassis_Wheel_L);
	MF9025_CANGetStatus(Chassis_Wheel_R);
}

/*
 *函数简介:腿长正运动学解算
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Observer_LegForwardKinematicsSolution(Observer_LegStatus *Leg)
{
	//获取L0和phi0
	float phi_1=Leg->phi_1;
	float phi_4=Leg->phi_4;
	float dphi_1=Leg->dphi_1;
	float dphi_4=Leg->dphi_4;
	
	float XB=l_1*arm_cos_f32(phi_1);
	float YB =l_1*arm_sin_f32(phi_1);
	float XD=l_5+l_4*arm_cos_f32(phi_4);
	float YD=l_4*arm_sin_f32(phi_4);
	
	float lBD_2=(XD-XB)*(XD-XB)+(YD-YB)*(YD-YB);
	
	float A0=2*l_2*(XD-XB);
	float B0=2*l_2*(YD-YB);
	float C0=l_2*l_2+lBD_2-l_3*l_3;
	float phi2=2*atan2f((B0+sqrt(A0*A0+B0*B0-C0*C0)),A0+C0);
	float phi3=atan2f(YB-YD+l_2*arm_sin_f32(phi2),XB-XD+l_2*arm_cos_f32(phi2));
	
	float XC=l_1*arm_cos_f32(phi_1)+l_2*arm_cos_f32(phi2);
	float YC=l_1*arm_sin_f32(phi_1)+l_2*arm_sin_f32(phi2);
	
	(Leg->L_0)=sqrt((XC-l_5/2.0f)*(XC-l_5/2.0f)+YC*YC);
	(Leg->phi_0)=atan2f(YC,(XC-l_5/2.0f));
	
	//获取VMC雅可比矩阵元素
	float sigma1=arm_sin_f32(phi3-phi2);
	float sigma2=arm_sin_f32(phi3-phi_4);
	float sigma3=arm_sin_f32(phi_1-phi2);
	float sigma4=arm_sin_f32(Leg->phi_0-phi3);
	float sigma5=arm_cos_f32(Leg->phi_0-phi3);
	float sigma6=arm_sin_f32(Leg->phi_0-phi2);
	float sigma7=arm_cos_f32(Leg->phi_0-phi2);
	
	(Leg->J_11)=(l_1*sigma4*sigma3)/sigma1;
	(Leg->J_12)=(l_4*sigma6*sigma2)/sigma1;
	(Leg->J_21)=(l_1*sigma5*sigma3)/((Leg->L_0)*sigma1);
	(Leg->J_22)=(l_4*sigma7*sigma2)/((Leg->L_0)*sigma1);
	
	//获取VMC逆解矩阵元素
	float sigma8=l_4*sigma2;
	float sigma9=l_1*sigma3;
	(Leg->T_11)=-sigma7/sigma9;
	(Leg->T_12)=sigma5/sigma8;
	(Leg->T_21)=(Leg->L_0)*sigma6/sigma9;
	(Leg->T_22)=-(Leg->L_0)*sigma4/sigma8;
	
	//获取dL0 ddL0 dphi0
	float sigma10=l_1*dphi_1;
	float sigma11=l_5/2.0f-XC;
	float sigma12=sigma10*arm_sin_f32(phi_1-phi3)+sigma8*dphi_4;
	float sigma13=sigma12/sigma1;
	float sigma14=sigma10*arm_cos_f32(phi_1)+sigma13*arm_cos_f32(phi2);
	float sigma15=sigma10*arm_sin_f32(phi_1)+sigma13*arm_sin_f32(phi2);
	(Leg->last_dL_0)=(Leg->dL_0);
	(Leg->dL_0)=(YC*sigma14+sigma11*sigma15)/(Leg->L_0);
	(Leg->ddL_0)=0.19f*(Leg->dL_0-Leg->last_dL_0)/Observer_BalanceStatus.dt+0.81f*(Leg->ddL_0);//一阶低通滤波
	(Leg->dphi_0)=-(sigma14*sigma11-YC*sigma15)/(YC*YC+sigma11*sigma11);
}

/*
 *函数简介:支持力解算
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Observer_GetFN(Observer_LegStatus *Leg)
{
	//z_ddot_w=z_ddot_M-L_ddot_0*cos(theta)+2*L_dot_0*theta_dot*sin(theta)+L_0*theta_ddot*sin(theta)+L_0*theta_dot^2*cos(theta)
	//F_N=P+m_w*g+m_w*z_ddot_w
	float COS=arm_cos_f32(Leg->theta);
	float SIN=arm_sin_f32(Leg->theta);
	
	(Leg->F)=(Leg->T_11)*(Leg->T1)+(Leg->T_12)*(Leg->T2);
	(Leg->Tp)=(Leg->T_21)*(Leg->T1)+(Leg->T_22)*(Leg->T2);
	float P=(Leg->F)*COS+(Leg->Tp)*SIN/(Leg->L_0);
	
	float ddz_w=Observer_BalanceStatus.Body.a_zE-(Leg->ddL_0)*COS+2.0f*(Leg->dL_0)*(Leg->dtheta)*SIN+(Leg->L_0)*(Leg->ddtheta)*SIN+(Leg->L_0)*(Leg->dtheta)*(Leg->dtheta)*COS;

	(Leg->FN)=P+m_w*g+m_w*ddz_w;
}

/*
 *函数简介:观测器数据处理
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Observer_DataGet(void)
{
	//机体
	Observer_BalanceStatus.Body.Yaw=AttitudeAlgorithms_IMU.Yaw;
	Observer_BalanceStatus.Body.GM6020_Yaw=-(GM6020_MotorStatus[0].Angle-Yaw_GM6020PositionValue)/8192.0f*2.0f*PI;
	
	int16_t Raw_Theta=Yaw_GM6020PositionValue-GM6020_MotorStatus[0].Angle;//获取底盘云台相对角度原始数据
	if(Raw_Theta<0)Raw_Theta+=8192;
	//Observer_BalanceStatus.Yaw_Theta=Raw_Theta/8192.0f*2.0f*3.141592653589793238462643383279f;
	Observer_BalanceStatus.Body.Yaw_Theta=Raw_Theta*0.000766990393942820614859043794746f;//获取底盘云台相对角度

	Observer_BalanceStatus.Body.Pitch=-AttitudeAlgorithms_IMU.Roll;
	Observer_BalanceStatus.Body.Roll=AttitudeAlgorithms_IMU.Pitch;
	
	Observer_BalanceStatus.Body.dYaw=AttitudeAlgorithms_IMU.dyaw;
	Observer_BalanceStatus.Body.dPitch=-AttitudeAlgorithms_IMU.droll;
	Observer_BalanceStatus.Body.dRoll=AttitudeAlgorithms_IMU.dpitch;
	
//	Observer_BalanceStatus.Body.a_xE=AttitudeAlgorithms_IMU.ayE;
	Observer_BalanceStatus.Body.a_yE=AttitudeAlgorithms_IMU.axE;
	Observer_BalanceStatus.Body.a_zE=AttitudeAlgorithms_IMU.azE;
	
//	Observer_BalanceStatus.Body.a_xb=AttitudeAlgorithms_IMU.ayb;
//	Observer_BalanceStatus.Body.a_yb=AttitudeAlgorithms_IMU.axb;
	Observer_BalanceStatus.Body.a_zb=AttitudeAlgorithms_IMU.azb;

	//左腿
	Observer_BalanceStatus.LeftLeg.Wheel.Angle=MF9025_MotorStatus[Chassis_Wheel_L-0x141].Position/65536.0f*2.0f*PI;
	Observer_BalanceStatus.LeftLeg.Wheel.x=Observer_BalanceStatus.LeftLeg.Wheel.Angle*Wheel_R;
	Observer_BalanceStatus.LeftLeg.Wheel.Speed=MF9025_MotorStatus[Chassis_Wheel_L-0x141].Speed;
	Observer_BalanceStatus.LeftLeg.Wheel.T=MF9025_MotorStatus[Chassis_Wheel_L-0x141].Torque;
	
	Observer_BalanceStatus.LeftLeg.phi_1=L0MAX_phi1-DM_J8009_MotorStatus[Chassis_Joint1_L-0x01].Position;
	Observer_BalanceStatus.LeftLeg.phi_4=L0MAX_phi4-DM_J8009_MotorStatus[Chassis_Joint2_L-0x01].Position;
	Observer_BalanceStatus.LeftLeg.dphi_1=-DM_J8009_MotorStatus[Chassis_Joint1_L-0x01].Speed;
	Observer_BalanceStatus.LeftLeg.dphi_4=-DM_J8009_MotorStatus[Chassis_Joint2_L-0x01].Speed;
	Observer_LegForwardKinematicsSolution(&(Observer_BalanceStatus.LeftLeg));
	
	Observer_BalanceStatus.LeftLeg.T1=-DM_J8009_MotorStatus[Chassis_Joint1_L-0x01].Torque;
	Observer_BalanceStatus.LeftLeg.T2=-DM_J8009_MotorStatus[Chassis_Joint2_L-0x01].Torque;
	
	Observer_BalanceStatus.LeftLeg.theta=PI/2.0f-(Observer_BalanceStatus.LeftLeg.phi_0+Observer_BalanceStatus.Body.Pitch);
	Observer_BalanceStatus.LeftLeg.last_dtheta=Observer_BalanceStatus.LeftLeg.dtheta;
	Observer_BalanceStatus.LeftLeg.dtheta=-(Observer_BalanceStatus.LeftLeg.dphi_0+Observer_BalanceStatus.Body.dPitch);
	Observer_BalanceStatus.LeftLeg.ddtheta=0.19f*(Observer_BalanceStatus.LeftLeg.dtheta-Observer_BalanceStatus.LeftLeg.last_dtheta)/Observer_BalanceStatus.dt+0.81f*Observer_BalanceStatus.LeftLeg.ddtheta;//一阶低通滤波
	
	Observer_GetFN(&Observer_BalanceStatus.LeftLeg);
	
	//右腿
	Observer_BalanceStatus.RightLeg.Wheel.Angle=-MF9025_MotorStatus[Chassis_Wheel_R-0x141].Position/65536.0f*2.0f*PI;
	Observer_BalanceStatus.RightLeg.Wheel.x=Observer_BalanceStatus.RightLeg.Wheel.Angle*Wheel_R;
	Observer_BalanceStatus.RightLeg.Wheel.Speed=-MF9025_MotorStatus[Chassis_Wheel_R-0x141].Speed;
	Observer_BalanceStatus.RightLeg.Wheel.T=-MF9025_MotorStatus[Chassis_Wheel_R-0x141].Torque;

	Observer_BalanceStatus.RightLeg.phi_1=L0MAX_phi1+DM_J8009_MotorStatus[Chassis_Joint1_R-0x01].Position;
	Observer_BalanceStatus.RightLeg.phi_4=L0MAX_phi4+DM_J8009_MotorStatus[Chassis_Joint2_R-0x01].Position;
	Observer_BalanceStatus.RightLeg.dphi_1=DM_J8009_MotorStatus[Chassis_Joint1_R-0x01].Speed;
	Observer_BalanceStatus.RightLeg.dphi_4=DM_J8009_MotorStatus[Chassis_Joint2_R-0x01].Speed;
	Observer_LegForwardKinematicsSolution(&(Observer_BalanceStatus.RightLeg));

	Observer_BalanceStatus.RightLeg.T1=DM_J8009_MotorStatus[Chassis_Joint1_R-0x01].Torque;
	Observer_BalanceStatus.RightLeg.T2=DM_J8009_MotorStatus[Chassis_Joint2_R-0x01].Torque;

	Observer_BalanceStatus.RightLeg.theta=PI/2.0f-(Observer_BalanceStatus.RightLeg.phi_0+Observer_BalanceStatus.Body.Pitch);
	Observer_BalanceStatus.RightLeg.last_dtheta=Observer_BalanceStatus.RightLeg.dtheta;
	Observer_BalanceStatus.RightLeg.dtheta=-(Observer_BalanceStatus.RightLeg.dphi_0+Observer_BalanceStatus.Body.dPitch);
	Observer_BalanceStatus.RightLeg.ddtheta=0.19f*(Observer_BalanceStatus.RightLeg.dtheta-Observer_BalanceStatus.RightLeg.last_dtheta)/Observer_BalanceStatus.dt+0.81f*Observer_BalanceStatus.RightLeg.ddtheta;//一阶低通滤波

	Observer_GetFN(&Observer_BalanceStatus.RightLeg);

	//机体
	MotionEstimation_Update();
	
	//功率
	Observer_BalanceStatus.P=(MF9025_MotorStatus[Chassis_Wheel_L-0x141].Power+MF9025_MotorStatus[Chassis_Wheel_R-0x141].Power)/4.0f;
}
