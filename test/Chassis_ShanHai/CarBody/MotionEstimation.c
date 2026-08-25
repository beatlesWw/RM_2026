#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "MotionEstimation.h"
#include "Observer.h"
#include "Parameter.h"

//状态向量x=[v a],量测向量z=[v a]
float MotionEstimation_P[4]={	1.0f,		0.0f,
								0.0f,		1.0f,		};

float MotionEstimation_F[4]={	1.0f,		0.002f,
								0.0f,		1.0f,		};

float MotionEstimation_H[4]={	1.0f,		0.0f,
								0.0f,		1.0f,		};

float MotionEstimation_Q[4]={	0.1f,		0.0f,
								0.0f,		0.1f,		};

float MotionEstimation_R[4]={	100.0f,		0.0f,
								0.0f,		1000000.0f,	};

/*
 *函数简介:运动估计初始化
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void MotionEstimation_Init(void)
{
	Kalman_Filter_Init(&(Observer_BalanceStatus.Body.MotionEstimation.Motion_Estimation),2,0,2);
	memcpy(Observer_BalanceStatus.Body.MotionEstimation.Motion_Estimation.P_data,MotionEstimation_P,sizeof(MotionEstimation_P));
	memcpy(Observer_BalanceStatus.Body.MotionEstimation.Motion_Estimation.F_data,MotionEstimation_F,sizeof(MotionEstimation_F));
	memcpy(Observer_BalanceStatus.Body.MotionEstimation.Motion_Estimation.H_data,MotionEstimation_H,sizeof(MotionEstimation_H));
	memcpy(Observer_BalanceStatus.Body.MotionEstimation.Motion_Estimation.Q_data,MotionEstimation_Q,sizeof(MotionEstimation_Q));
	memcpy(Observer_BalanceStatus.Body.MotionEstimation.Motion_Estimation.R_data,MotionEstimation_R,sizeof(MotionEstimation_R));
}

/*
 *函数简介:运动估计量测更新
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void MotionEstimation_MeasureUpdate(void)
{
	//w=w_eb+phi_dot_bc+w_ecd
	//v=wR+L_0*theta_dot*cos(theta)+L_dot_0*sin(theta)
	float wL=Observer_BalanceStatus.LeftLeg.Wheel.Speed-Observer_BalanceStatus.Body.dPitch-Observer_BalanceStatus.LeftLeg.dphi_0;
	Observer_BalanceStatus.Body.MotionEstimation.vl_measure=wL*Wheel_R+Observer_BalanceStatus.LeftLeg.dL_0*arm_sin_f32(Observer_BalanceStatus.LeftLeg.theta)+Observer_BalanceStatus.LeftLeg.L_0*Observer_BalanceStatus.LeftLeg.dtheta*arm_cos_f32(Observer_BalanceStatus.LeftLeg.theta);
	
	float wR=Observer_BalanceStatus.RightLeg.Wheel.Speed-Observer_BalanceStatus.Body.dPitch-Observer_BalanceStatus.RightLeg.dphi_0;
	Observer_BalanceStatus.Body.MotionEstimation.vr_measure=wR*Wheel_R+Observer_BalanceStatus.RightLeg.dL_0*arm_sin_f32(Observer_BalanceStatus.RightLeg.theta)+Observer_BalanceStatus.RightLeg.L_0*Observer_BalanceStatus.RightLeg.dtheta*arm_cos_f32(Observer_BalanceStatus.RightLeg.theta);

	Observer_BalanceStatus.Body.MotionEstimation.v_measure=(Observer_BalanceStatus.Body.MotionEstimation.vl_measure+Observer_BalanceStatus.Body.MotionEstimation.vr_measure)/2.0f;
	Observer_BalanceStatus.Body.MotionEstimation.a_measure=Observer_BalanceStatus.Body.a_yE;
//	
//	if(Observer_BalanceStatus.LeftLeg.FN<FN_Threshold && Observer_BalanceStatus.RightLeg.FN<FN_Threshold)
//		Observer_BalanceStatus.Body.MotionEstimation.v_measure=0;
}

/*
 *函数简介:运动估计更新
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void MotionEstimation_Update(void)
{
	MotionEstimation_MeasureUpdate();
	
	Observer_BalanceStatus.Body.MotionEstimation.v_nofilter=0.5f*(Observer_BalanceStatus.LeftLeg.Wheel.Speed+Observer_BalanceStatus.RightLeg.Wheel.Speed)*Wheel_R;
	Observer_BalanceStatus.Body.MotionEstimation.x_nofilter+=Observer_BalanceStatus.Body.MotionEstimation.v_nofilter*Observer_BalanceStatus.dt;
		
	Observer_BalanceStatus.Body.MotionEstimation.Motion_Estimation.MeasuredVector[0]=Observer_BalanceStatus.Body.MotionEstimation.v_measure;
	Observer_BalanceStatus.Body.MotionEstimation.Motion_Estimation.MeasuredVector[1]=Observer_BalanceStatus.Body.MotionEstimation.a_measure;
	
    Kalman_Filter_Update(&(Observer_BalanceStatus.Body.MotionEstimation.Motion_Estimation));
	
	Observer_BalanceStatus.Body.MotionEstimation.v=Observer_BalanceStatus.Body.MotionEstimation.Motion_Estimation.FilteredValue[0];
	Observer_BalanceStatus.Body.MotionEstimation.x+=Observer_BalanceStatus.Body.MotionEstimation.v*Observer_BalanceStatus.dt;
	
	Observer_BalanceStatus.Body.x_nofilter=Observer_BalanceStatus.Body.MotionEstimation.x_nofilter;
	Observer_BalanceStatus.Body.dx_nofilter=Observer_BalanceStatus.Body.MotionEstimation.v_nofilter;
	Observer_BalanceStatus.Body.x=Observer_BalanceStatus.Body.MotionEstimation.x;
	Observer_BalanceStatus.Body.dx=Observer_BalanceStatus.Body.MotionEstimation.v;
}
