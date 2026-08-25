#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "AttitudeAlgorithms.h"
#include "BMI088.h"
#include "ins_task.h"
#include "Data.h"

AttitudeAlgorithms_IMU_Struct AttitudeAlgorithms_IMU;//IMU结构体

/*
 *函数简介:姿态解算初始化
 *参数说明:无
 *返回类型:无
 *备注:定时器定时1ms,更新四元数并解算角度
 */
void AttitudeAlgorithms_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);//开启时钟
	
	TIM_InternalClockConfig(TIM11);//选择时基单元的时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//配置时基单元
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_TimeBaseInitStructure.TIM_Period=500-1;//配置自动重装值ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=336-1;//配置分频值PSC,默认频率1000Hz
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM11,&TIM_TimeBaseInitStructure);//初始化TIM2
	
	TIM_ClearFlag(TIM11,TIM_FLAG_Update);//清除配置时基单元产生的中断标志位
	
	TIM_ITConfig(TIM11,TIM_IT_Update,ENABLE);//使能更新中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组
	
	NVIC_InitTypeDef NVIC_InitStructure;//配置NVIC（配置参数）
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_TRG_COM_TIM11_IRQn;//选择中断通道为TIM11
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//TIM2的抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//TIM2的响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	
	
	BMI088_Init();//初始化BMI088
	INS_Init();//INS初始化
	AttitudeAlgorithms_IMU.dt=0.001f;//INS更新周期1kHz
	AttitudeAlgorithms_IMU.AccelLPF=0.0085f;//加速度计低通滤波系数

	TIM_Cmd(TIM11,ENABLE);//启动定时器
}

/*
 *函数简介:TIM11定时器更新中断函数
 *参数说明:无
 *返回类型:无
 *备注:定时1ms更新四元数并且解算角度
 */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	static float AttitudeAlgorithms_LastDegYaw,AttitudeAlgorithms_ThisDegYaw;//上一次角度制偏航角,本次角度制偏航角
	static int64_t AttitudeAlgorithms_YawR=0;//角度制偏航角圈数
	static uint8_t AttitudeAlgorithms_YawFirstFlag=1;//第一次接收数据标志位
	
	if(TIM_GetITStatus(TIM11,TIM_IT_Update)==SET)//检测TIM2更新
	{
		TIM_ClearITPendingBit(TIM11,TIM_IT_Update);//清除标志位
		float AccelFilter_A=AttitudeAlgorithms_IMU.AccelLPF/(AttitudeAlgorithms_IMU.AccelLPF+AttitudeAlgorithms_IMU.dt);//加速度计低通滤波器系数A
		float AccelFilter_B=AttitudeAlgorithms_IMU.dt/(AttitudeAlgorithms_IMU.AccelLPF+AttitudeAlgorithms_IMU.dt);//加速度计低通滤波器系数B
		
		/*====================机体坐标系下的加速度====================*/
//		AttitudeAlgorithms_IMU.axb=AttitudeAlgorithms_IMU.axb*AccelFilter_A+BMI088_Accel[0]*AccelFilter_B;//机体加速度低通滤波
//		AttitudeAlgorithms_IMU.ayb=AttitudeAlgorithms_IMU.ayb*AccelFilter_A+BMI088_Accel[1]*AccelFilter_B;
//		AttitudeAlgorithms_IMU.azb=AttitudeAlgorithms_IMU.azb*AccelFilter_A+BMI088_Accel[2]*AccelFilter_B;
		AttitudeAlgorithms_IMU.axb=INS.MotionAccel_b[0];//机体加速度
		AttitudeAlgorithms_IMU.ayb=INS.MotionAccel_b[1];
		AttitudeAlgorithms_IMU.azb=INS.MotionAccel_b[2];
		
		/*====================姿态解算====================*/
		INS_Task();//INS更新
		AttitudeAlgorithms_LastDegYaw=AttitudeAlgorithms_ThisDegYaw;//上一次偏航角
		AttitudeAlgorithms_ThisDegYaw=INS.Yaw;//此次偏航角
		AttitudeAlgorithms_IMU.Pitch_deg=INS.Pitch;//俯仰角
		AttitudeAlgorithms_IMU.Roll_deg=INS.Roll;//翻滚角	
		if(AttitudeAlgorithms_YawFirstFlag==0)//获取带圈数的偏航角
		{
			if(AttitudeAlgorithms_ThisDegYaw-AttitudeAlgorithms_LastDegYaw>180.0f)AttitudeAlgorithms_YawR--;
			else if(AttitudeAlgorithms_LastDegYaw-AttitudeAlgorithms_ThisDegYaw>180.0f)AttitudeAlgorithms_YawR++;
		}
		else AttitudeAlgorithms_YawFirstFlag=0;
		AttitudeAlgorithms_IMU.Yaw_deg=AttitudeAlgorithms_YawR*360.0f+AttitudeAlgorithms_ThisDegYaw;
		
		AttitudeAlgorithms_IMU.Yaw=AttitudeAlgorithms_IMU.Yaw_deg*Data_Deg2Rad;//转为弧度制
		AttitudeAlgorithms_IMU.Pitch=AttitudeAlgorithms_IMU.Pitch_deg*Data_Deg2Rad;//转为弧度制
		AttitudeAlgorithms_IMU.Roll=AttitudeAlgorithms_IMU.Roll_deg*Data_Deg2Rad;//转为弧度制
		
		/*====================姿态角速度====================*/
		float dyawB=INS.Gyro[2];
		float dpitchB=INS.Gyro[0];
		float drollB=INS.Gyro[1];
//		AttitudeAlgorithms_IMU.dyaw=arm_cos_f32(AttitudeAlgorithms_IMU.Roll)/arm_cos_f32(AttitudeAlgorithms_IMU.Pitch)*dyawB+arm_sin_f32(AttitudeAlgorithms_IMU.Roll)/arm_cos_f32(AttitudeAlgorithms_IMU.Pitch)*dpitchB;//坐标变换
//		AttitudeAlgorithms_IMU.dpitch=-arm_sin_f32(AttitudeAlgorithms_IMU.Roll)*dyawB+arm_cos_f32(AttitudeAlgorithms_IMU.Roll)*dpitchB;
//		AttitudeAlgorithms_IMU.droll=arm_cos_f32(AttitudeAlgorithms_IMU.Roll)*tanf(AttitudeAlgorithms_IMU.Pitch)*dyawB+arm_sin_f32(AttitudeAlgorithms_IMU.Roll)*tanf(AttitudeAlgorithms_IMU.Pitch)*dpitchB+drollB;
		AttitudeAlgorithms_IMU.dyaw=dyawB;//坐标变换
		AttitudeAlgorithms_IMU.dpitch=dpitchB;
		AttitudeAlgorithms_IMU.droll=drollB;
		
		/*====================世界坐标系下的加速度====================*/
//		float Accel_E[3];
//		BodyFrameToEarthFrame(BMI088_Accel,Accel_E,INS.q);//坐标变换
//		AttitudeAlgorithms_IMU.axE=AttitudeAlgorithms_IMU.axE*AccelFilter_A+Accel_E[0]*AccelFilter_B;//世界加速度低通滤波
//		AttitudeAlgorithms_IMU.ayE=AttitudeAlgorithms_IMU.ayE*AccelFilter_A+Accel_E[1]*AccelFilter_B;
//		AttitudeAlgorithms_IMU.azE=AttitudeAlgorithms_IMU.azE*AccelFilter_A+Accel_E[2]*AccelFilter_B;
		AttitudeAlgorithms_IMU.axE=arm_cos_f32(AttitudeAlgorithms_IMU.Pitch)*INS.MotionAccel_b[0]+
								   arm_sin_f32(AttitudeAlgorithms_IMU.Roll)*arm_sin_f32(AttitudeAlgorithms_IMU.Pitch)*INS.MotionAccel_b[1]+
								   arm_cos_f32(AttitudeAlgorithms_IMU.Roll)*arm_sin_f32(AttitudeAlgorithms_IMU.Pitch)*INS.MotionAccel_b[2];//世界加速度低通滤波
		AttitudeAlgorithms_IMU.ayE=arm_cos_f32(AttitudeAlgorithms_IMU.Roll)*INS.MotionAccel_b[1]
								   -arm_sin_f32(AttitudeAlgorithms_IMU.Roll)*INS.MotionAccel_b[2];
		AttitudeAlgorithms_IMU.azE=-arm_sin_f32(AttitudeAlgorithms_IMU.Pitch)*INS.MotionAccel_b[0]+
								   arm_sin_f32(AttitudeAlgorithms_IMU.Roll)*arm_cos_f32(AttitudeAlgorithms_IMU.Pitch)*INS.MotionAccel_b[1]+
								   arm_cos_f32(AttitudeAlgorithms_IMU.Roll)*arm_cos_f32(AttitudeAlgorithms_IMU.Pitch)*INS.MotionAccel_b[2];
	}
}
