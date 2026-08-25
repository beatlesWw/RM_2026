#ifndef __PID_CONTROLL_H
#define __PID_CONTROLL_H

#include "VMC.h"

extern Tumble_t tumble_ctrl;

void TumbleRecover_Init(void);

// 右腿串级PID + 双腿同步串级PID（在ChassisR_task中调用）
// vmcr: 右腿VMC, vmcl: 左腿VMC, vel_r: T1_r关节速度反馈
float TumbleRecover_Right(vmc_leg_t *vmcr, vmc_leg_t *vmcl, float vel_r);

// 左腿串级PID（在chassisL_task中调用）
// vmcl: 左腿VMC, vel_l: T1_l关节速度反馈
float TumbleRecover_Left(vmc_leg_t *vmcl, float vel_l);

float TumbleRecover_sync(vmc_leg_t *vmcr, vmc_leg_t *vmcl, float vel_r);
uint8_t tumble_detect(attitude_t *INS,vmc_leg_t *leg);
#endif /* __PID_CONTROLL_H */
