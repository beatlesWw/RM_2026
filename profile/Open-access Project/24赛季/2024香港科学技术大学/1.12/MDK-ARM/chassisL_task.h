#ifndef __CHASSISL_TASK_H
#define __CHASSISL_TASK_H



#include "main.h"
#include "dm_8009_drv.h"
#include "chassisR_task.h"
//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//�������ֵת���ɽǶ�ֵ
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif
//reducation of 3508 motor
//m3508����ļ��ٱ�
#define M3508_MOTOR_REDUCATION 15.764705882f

//m3508 rpm change to chassis speed   576.096  3.14 *07425 = 0.233145
//m3508ת��ת��(rpm)ת���ɵ����ٶ�(m/s)�ı�����c=pi*r/(30*k)��kΪ������ٱ�
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.0004998609952f

//m3508 rpm change to motor angular velocity
//m3508ת��ת��(rpm)ת��Ϊ�������ٶ�(rad/s)�ı���
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.00664267f

//m3508 current change to motor torque
//m3508ת�ص���(-16384~16384)תΪ�ɵ�����ת��(N.m)�ı���
//c=20/16384*0.3��   
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f
#define X_err 0.0f

////�����ֵ��PID
//#define TRIGGER_ANGLE_PID_KP        500.0f
//#define TRIGGER_ANGLE_PID_KI        0.03f
//#define TRIGGER_ANGLE_PID_KD        0.0f










extern void ChassisL_task(void);

extern void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc,pid_type_def *legl);
extern void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,pid_type_def *leg);
extern  fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
#endif


