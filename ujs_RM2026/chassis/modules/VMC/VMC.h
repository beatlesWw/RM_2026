#ifndef __VMC_CALC_H
#define __VMC_CALC_H

#include "main.h"
#include "ins_task.h"
#include  "arm_math.h"

#define pi 3.1415926535f

//右腿
#define LEGR_PID_KP  2700.0f
// #define LEGR_PID_KP  2000.0f
#define LEGR_PID_KI  0.0f//不积分
// #define LEGR_PID_KD  3600.0f
#define LEGR_PID_KD  15000.0f
#define LEGR_PID_MAX_OUT  500.0f //500N
#define LEGR_PID_MAX_IOUT 0.0f

//左腿
#define LEGL_PID_KP  2700.0f
// #define LEGL_PID_KP  2000.0f
#define LEGL_PID_KI  0.0f//不积分
// #define LEGL_PID_KD  3600.0f
#define LEGL_PID_KD  15000.0f
#define LEGL_PID_MAX_OUT  500.0f //500N
#define LEGL_PID_MAX_IOUT 0.0f
typedef struct
{
	//五连杆
	/*左右两腿的公共参数，固定不变*/
	//腿长定义   单位为m
	float l5;
	float l1;
	float l2;
	float l3;
	float l4;
	
	float XB,YB;//B点的坐标
	float XD,YD;//D点的坐标
	
	float XC,YC;//C点的直角坐标
	float L0,phi0;//C点的极坐标
	float alpha;
	float d_alpha;	
	
	float lBD;//BD两点的距离
	
	float d_phi0;//现在C点角度phi0的变换率
	float last_phi0;//上一次C点角度，用于计算角度phi0的变换率d_phi0
	
	float A0,B0,C0;//中间变量
	float phi2,phi3;
	float phi1,phi4;
	
	float j11,j12,j21,j22;//笛卡尔空间力到关节空间的力的雅可比矩阵系数
	float torque_set[2];
	float wheel_set;

	float F0;
	float Tp;
	float F0_test;
	
	float theta;
	float theta_prev;//上一拍theta，用于跳变检测
	float theta_accum;//连续累积theta（跳变补偿后），供debug观测
	float d_theta;//theta的一阶导数
	float last_d_theta;
	float dd_theta;//theta的二阶导数
	float d_theta_lpf;
	float dd_theta_lpf;
	
	float d_L0;//L0的一阶导数
	float dd_L0;//L0的二阶导数
	float last_L0;
	float last_d_L0;
	float d_L0_lpf;
	float dd_L0_lpf;
	
	float FN;//支持力
	float Fv;
	float z_w_ddot;
	float z_m_ddot;
	float z_m_ddot_lpf;
	float z_w_ddot_lpf;
	float Gimbal_YAW;

	uint8_t first_flag;
	uint8_t leg_flag;//腿长完成标志
	uint8_t leg_tumble;
	
} vmc_leg_t;
	
	
	
	
typedef struct
{
    float legr_dphi0;   // 右腿外环输出（目标phi0速度，作为内环给定）
    float legl_dphi0;   // 左腿外环输出（目标phi0速度，作为内环给定）
    float Tp_r;         // 右腿内环输出（最终髋关节力矩）
    float Tp_l;         // 左腿内环输出（最终髋关节力矩）
    float Tp_dphi0;     // 同步外环输出（phi0差值控制，作为同步内环给定）
    float Tp_sync;      // 同步内环输出（双腿同步修正力矩）
	float pitch_continuous;  // 连续化的pitch角度
    float last_pitch_raw;    // 上次原始pitch值
} Tumble_t;

extern void VMC_init(vmc_leg_t *vmc);//给杆长赋值	
	
//计算theta和d_theta给lqr用，同时也计算腿长L0 
extern void VMC_calc_right(vmc_leg_t *vmc,attitude_t *ins,float dt);

//计算theta和d_theta给lqr用，同时也计算腿长L0
extern void VMC_calc_left(vmc_leg_t *vmc,attitude_t *ins,float dt);
extern void VMC_calc_splitter(vmc_leg_t *vmc);//计算期望的关节输出力矩

extern uint8_t Ground_detectionR(vmc_leg_t *vmc,attitude_t *ins);//右腿离地检测
extern uint8_t Ground_detectionL(vmc_leg_t *vmc,attitude_t *ins);//左腿离地检测

extern float LQR_K_calc(float *coe,float len);

#endif

