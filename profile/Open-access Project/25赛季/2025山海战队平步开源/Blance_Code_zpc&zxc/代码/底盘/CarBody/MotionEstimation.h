#ifndef __MOTIONESTIMATION_H
#define __MOTIONESTIMATION_H

#include "stdint.h"
#include "kalman_filter.h"

typedef struct
{
	float x;//卡尔曼滤波后的位移x
	float v;//卡尔曼滤波后的速度v
	float x_nofilter;//卡尔曼滤波前的位移x
	float v_nofilter;//卡尔曼滤波前的速度v
	
	float vl_measure,vr_measure,v_measure;//左轮速度测量值,右轮速度测量值,速度测量值
	float a_measure;//加速度测量值
		
	KalmanFilter_t Motion_Estimation;//卡尔曼滤波器
}MotionEstimation_Balance;

void MotionEstimation_Init(void);//运动估计更新
void MotionEstimation_MeasureUpdate(void);//运动估计量测更新
void MotionEstimation_Update(void);//运动估计初始化

#endif
