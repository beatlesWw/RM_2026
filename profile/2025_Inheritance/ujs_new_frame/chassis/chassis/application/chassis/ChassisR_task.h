#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dmmotor.h"
#include "controller.h"
#include "VMC.h"
#include "ins_task.h"
#include "remote_control.h"
#include "can.h"
#include "user_lib.h"
#include "robot_def.h"
// //m3508电机的减速比
// #define M3508_MOTOR_REDUCATION 15.764705882f
// //m3508 rpm change to chassis speed   576.096  3.14 *07425 = 0.233145
// //m3508转子转速(rpm)转化成底盘速度(m/s)的比例，c=pi*r/(30*k)，k为电机减速比
// #define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.0004998609952f

// //m3508 rpm change to motor angular velocity
// //m3508转子转速(rpm)转换为输出轴角速度(rad/s)的比例
// #define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.00664267f

// //m3508 current change to motor torque
// //m3508转矩电流(-16384~16384)转为成电机输出转矩(N.m)的比例
// //c=20/16384*0.3，   
// #define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f

extern void ChassisR_task(void);
extern void chassisR_feedback_update(Balance_Chassis_e *chassis,DMMotorInstance *T1,DMMotorInstance *T2,vmc_leg_t *vmc,attitude_t *ins);
extern void chassisR_control_loop(Balance_Chassis_e  *chassis,vmc_leg_t *vmcr,attitude_t *ins,float *LQR_K);

#endif


