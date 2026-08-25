#ifndef __ATTITUDEALGORITHMS_H
#define __ATTITUDEALGORITHMS_H

typedef struct
{
	float dt;			//INS更新周期
	float AccelLPF;		//加速度计低通滤波器系数
	
	float Yaw;			//偏航角(弧度制)
	float Pitch;		//俯仰角(弧度制)
	float Roll;			//翻滚角(弧度制)
	
	float Yaw_deg;		//偏航角(角度制)
	float Pitch_deg;	//俯仰角(角度制)
	float Roll_deg;		//翻滚角(角度制)
	
	float axb;			//机体x轴加速度
	float ayb;			//机体y轴加速度
	float azb;			//机体z轴加速度
	float axE;			//世界x轴加速度
	float ayE;			//世界y轴加速度
	float azE;			//世界z轴加速度
	
	float dyaw;			//偏航角速度(弧度制)
	float dpitch;		//俯仰角速度(弧度制)
	float droll;		//翻滚角速度(弧度制)
}AttitudeAlgorithms_IMU_Struct;//IMU结构体

extern AttitudeAlgorithms_IMU_Struct AttitudeAlgorithms_IMU;//IMU结构体

void AttitudeAlgorithms_Init(void);//姿态解算初始化

#endif
