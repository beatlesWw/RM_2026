#ifndef __VMC_CALC_H
#define __VMC_CALC_H

#include "main.h"
#include "INS_task.h"
#include  "arm_math.h"

#define pi 3.1415926535f
//#define LEG_PID_KP  350.0f
//#define LEG_PID_KI  0.0f//������
//#define LEG_PID_KD  3000.0f
//#define LEG_PID_KP  300.0f
//#define LEG_PID_KI  0.0f//������
//#define LEG_PID_KD  50.0f

#define LEG_PID_KP  200.0f
#define LEG_PID_KI  0.0f//������
#define LEG_PID_KD  500.0f
#define LEG_PID_MAX_OUT  90.0f //90ţ
#define LEG_PID_MAX_IOUT 0.0f


#define WHEEL_PID_KP  18000.0f
#define WHEEL_PID_KI  10.0f//������
#define WHEEL_PID_KD  0.0f
#define WHEEL_PID_MAX_OUT  16000.0f //90ţ
#define WHEEL_PID_MAX_IOUT 2000.0f





typedef struct
{
	//������
	/*�������ȵĹ����������̶�����*/
	//�ȳ�����   ��λΪm
	float l5;
	float l1;
	float l2;
	float l3;
	float l4;
	
	float XB,YB;//B�������
	float XD,YD;//D�������
	
	float XC,YC;//C���ֱ������
	float L0,phi0;//C��ļ�����
	float alpha;
	float d_alpha;	
	
	float	lBD;//BD����ľ���
	
	float d_phi0;//����C��Ƕ�phi0�ı任��
	float last_phi0;//��һ��C��Ƕȣ����ڼ���Ƕ�phi0�ı任��d_phi0
	
	float A0,B0,C0;//�м����
	float phi2,phi3;
	float phi1,phi4;
	
	float j11,j12,j21,j22;//�ѿ����ռ������ؽڿռ�������ſɱȾ���ϵ��
	float torque_set[2];

	float F0;
	float Tp;
	float F02;
	
	float theta;
	float d_theta;//theta��һ�׵���
	float last_d_theta;
	float dd_theta;//theta�Ķ��׵���
	
	float d_L0;//L0��һ�׵���
	float dd_L0;//L0�Ķ��׵���
	float last_L0;
	float last_d_L0;
	
	float FN;//֧����
	
	uint8_t first_flag;
	uint8_t leg_flag;//�ȳ���ɱ�־
	
} vmc_leg_t;
	
extern void VMC_init(vmc_leg_t *vmc);
extern void VMC_calc_right(vmc_leg_t *vmc,INS_t *ins,float dt);
extern void VMC_calc_left(vmc_leg_t *vmc,INS_t *ins,float dt);
extern void VMC_calc_2(vmc_leg_t *vmc);
extern uint8_t ground_detectionR(vmc_leg_t *vmc,INS_t *ins);
extern uint8_t ground_detectionL(vmc_leg_t *vmc,INS_t *ins);
extern float LQR_K_calc(float *coe,float len);

#endif

