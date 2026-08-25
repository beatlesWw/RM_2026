#ifndef __PARAMETER_H
#define __PARAMETER_H

/*=============================================模型参数=============================================*/
#define g													9.80665f

#define Wheel_R												0.0775f//0.155f/2.0f
#define m_w													1.674f
#define Rl													0.2315f//0.463f/2.0f

#define L0MAX_phi1											1.9198621771937625346160598453375f//110°
#define L0MAX_phi4											1.221730476396030703846583537942f//70°
#define l_1													0.15f
#define l_2													0.27f
#define l_3													0.27f
#define l_4													0.15f
#define l_5													0.15f
//#define m_p													1.202f


#define Blance_X											0.00f;
#define FN_Threshold										20.0f

//#define M													8.494f
//不装云台14.246kg，装云台17.9kg，机体12.148kg，云台3.654kg

/*=============================================结构参数=============================================*/
#define Chassis_Wheel_L										MF9025_1
#define Chassis_Joint1_L									DM_J8009_2
#define Chassis_Joint2_L									DM_J8009_3
#define Chassis_Wheel_R										MF9025_2
#define Chassis_Joint1_R									DM_J8009_1
#define Chassis_Joint2_R									DM_J8009_4

#define Yaw_GM6020PositionValue								3421//Yaw轴编码器值

/*=============================================麦轮参数=============================================*/
//#define Mecanum_WheelRadius									7.0f//麦轮半径(单位cm)

//#define Mecanum_rx											18.75f//底盘中心到轮子中心的距离的x轴分量(单位cm)
//#define Mecanum_ry											18.0f//底盘中心到轮子中心的距离的y轴分量(单位cm)

//#define Mecanum_LeverSpeedMapRate							(1.2f/660.0f)//拨杆速度映射比例
//#define Mecanum_GyroScopeAngularVelocity					5.0f//小陀螺角速度
//#define Mecanum_NormalSpeedRate								1.5f//底盘正常速度和超功率速度的比值

#endif
