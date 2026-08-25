#ifndef __CHASSISL_TASK_H
#define __CHASSISL_TASK_H

/**
 * @file ChassisL_task.h
 * @author UJS Wheel-Leg Balance Infantry Team
 * @brief 左腿控制任务头文件
 * @version 1.0
 * @date 2025-12-16
 * 
 * @details 左腿控制任务负责：
 *          - 左前关节电机(T1_l)和左后关节电机(T2_l)的VMC控制
 *          - 左侧轮毂电机(W_l)的速度控制
 *          - 左腿VMC运动学解算和LQR控制
 *          - 左腿离地检测和姿态补偿
 * 
 * @note 本任务与ChassisR_task并行运行，共享chassis.c中初始化的电机实例
 *       任务优先级：osPriorityAboveNormal，堆栈大小：512字节
 */

#include "main.h"
#include "dmmotor.h"
#include "dji_motor.h"
#include "controller.h"
#include "VMC.h"
#include "ins_task.h"
#include "robot_def.h"
#include "chassis.h"

/* ==================== 左腿任务时间参数 ==================== */
#define CHASSL_TIME 1  // 左腿任务周期：1ms

/* ==================== 左腿任务函数声明 ==================== */

/**
 * @brief 左腿控制主任务
 * @note 在FreeRTOS中以1kHz频率运行，负责左腿的完整控制流程
 *       包括：反馈更新 -> VMC解算 -> LQR控制 -> 电机输出
 */
extern void ChassisL_task(void);

/**
 * @brief 左腿反馈数据更新
 * @param chassis 底盘控制结构体指针
 * @param T1 左前关节电机实例指针
 * @param T2 左后关节电机实例指针
 * @param vmc 左腿VMC结构体指针
 * @param ins IMU姿态数据指针
 * @note 更新左腿关节角度、IMU数据、轮毂速度等反馈信息
 */
extern void chassisL_feedback_update(Balance_Chassis_e *chassis, 
                                      DMMotorInstance *T1, 
                                      DMMotorInstance *T2, 
                                      vmc_leg_t *vmc, 
                                      attitude_t *ins);

/**
 * @brief 左腿控制环路计算
 * @param chassis 底盘控制结构体指针
 * @param vmcl 左腿VMC结构体指针
 * @param ins IMU姿态数据指针
 * @param LQR_K LQR控制器增益数组
 * @note 执行VMC运动学解算、LQR控制计算、力矩分配等
 */
extern void chassisL_control_loop(Balance_Chassis_e *chassis, 
                                   vmc_leg_t *vmcl, 
                                   attitude_t*ins, 
                                   float *LQR_K);

#endif // __CHASSISL_TASK_H
