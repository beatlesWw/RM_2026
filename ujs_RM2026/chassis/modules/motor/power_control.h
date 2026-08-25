/**
 * @file dji_motor.h
 * @author neozng
 * @brief DJI智能电机头文件
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  1. 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
          2. 为M2006和M3508增加开环的零位校准函数,并在初始化时调用(根据用户配置决定是否调用)

 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"
#include "dji_motor.h"


DJIMotorInstance *PowerControlInit(Motor_Init_Config_s *config);

/**
 * @brief 电机功率控制,此时电机根据电机功率模型进行控制，不是直接的pid控制
 *
 * @param motor 电机实例指针
 * @param power 功率值
 */
void PowerControl(void);

/**
 * @brief 设置电机功率限制
 *
 * @param power_limit 功率限制值
 */
void SetPowerLimit(float power_limit);

/**
 * @brief 直接发送单个DJI电机的控制指令（零延迟，用于高频控制任务）
 * @param motor DJI电机实例
 * @param current_set 电流设定值（-16384~16384）
 * @note 此函数绕过独立任务和功率限制，直接发送CAN指令，适用于轮毂电机等不需要功率限制的场景
 */
void DJIMotorSendSingle(DJIMotorInstance *motor, int16_t current_set);

#endif // !DJI_MOTOR_H
