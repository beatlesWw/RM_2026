#ifndef __LQR_GIMBAL_H
#define __LQR_GIMBAL_H

#include "stm32f4xx.h"

/*==============================================================================
 * LQR 控制器增益参数定义
 *============================================================================*/

/* Pitch轴LQR增益 (位置环K1, 速度环K2) */
#define LQR_PITCH_K1        22.3607f    // 位置误差增益
#define LQR_PITCH_K2        0.7635f     // 速度误差增益

/* Yaw轴LQR增益 (位置环K3, 速度环K4) */
#define LQR_YAW_K3          22.3607f    // 位置误差增益
#define LQR_YAW_K4          2.5232f     // 速度误差增益

/* 摩擦轮LQR增益 (速度环K5) */
#define LQR_FRICTION_K5     0.0707f     // 速度误差增益

/* 拨弹盘LQR增益 (速度环K7) */
#define LQR_RAMMER_K7       10.0000f    // 速度误差增益

/* 单发模式LQR增益 */
#define LQR_DANFA_K7        22.3607f    // 位置环增益
#define LQR_DANFA_K8        22.3607f    // 速度环增益

/*==============================================================================
 * LQR 控制器输出限幅参数
 *============================================================================*/

#define LQR_PITCH_TAU_MAX       5.0f        // Pitch轴力矩限幅 (Nm)
#define LQR_YAW_CURRENT_MAX     16384.0f    // Yaw轴电流限幅
#define LQR_FRICTION_CURRENT_MAX 16384.0f   // 摩擦轮电流限幅
#define LQR_RAMMER_CURRENT_MAX  10000.0f    // 拨弹盘电流限幅

/*==============================================================================
 * LQR 控制器结构体定义
 *============================================================================*/

/* 二阶LQR控制器结构体 (位置-速度状态反馈) */
typedef struct {
    float K1;           // 位置误差增益
    float K2;           // 速度误差增益
    float output_max;   // 输出上限
    float output_min;   // 输出下限
} LQR_SecondOrder_TypeDef;

/* 一阶LQR控制器结构体 (速度状态反馈) */
typedef struct {
    float K;            // 速度误差增益
    float output_max;   // 输出上限
    float output_min;   // 输出下限
} LQR_FirstOrder_TypeDef;

/*==============================================================================
 * 函数声明
 *============================================================================*/

/**
 * @brief  初始化二阶LQR控制器
 * @param  lqr: LQR控制器结构体指针
 * @param  k1: 位置误差增益
 * @param  k2: 速度误差增益
 * @param  out_max: 输出上限
 * @param  out_min: 输出下限
 */
void LQR_SecondOrder_Init(LQR_SecondOrder_TypeDef *lqr, float k1, float k2, float out_max, float out_min);

/**
 * @brief  初始化一阶LQR控制器
 * @param  lqr: LQR控制器结构体指针
 * @param  k: 速度误差增益
 * @param  out_max: 输出上限
 * @param  out_min: 输出下限
 */
void LQR_FirstOrder_Init(LQR_FirstOrder_TypeDef *lqr, float k, float out_max, float out_min);

/**
 * @brief  二阶LQR控制器计算 (位置-速度状态反馈)
 * @param  lqr: LQR控制器结构体指针
 * @param  target_pos: 目标位置 (rad)
 * @param  actual_pos: 实际位置 (rad)
 * @param  actual_vel: 实际速度 (rad/s)
 * @retval 控制输出 (力矩或电流)
 */
float LQR_SecondOrder_Calc(LQR_SecondOrder_TypeDef *lqr, float target_pos, float actual_pos, float actual_vel);

/**
 * @brief  一阶LQR控制器计算 (速度状态反馈)
 * @param  lqr: LQR控制器结构体指针
 * @param  target_vel: 目标速度 (rad/s)
 * @param  actual_vel: 实际速度 (rad/s)
 * @retval 控制输出 (力矩)
 */
float LQR_FirstOrder_Calc(LQR_FirstOrder_TypeDef *lqr, float target_vel, float actual_vel);

/**
 * @brief  Pitch轴LQR力矩计算 (含重力前馈补偿)
 * @param  target_theta: 目标角度 (rad)
 * @param  actual_theta: 实际角度 (rad) - 来自陀螺仪
 * @param  actual_omega: 实际角速度 (rad/s) - 来自陀螺仪
 * @param  motor_pos: 电机位置 (rad) - 用于重力前馈计算
 * @retval 控制力矩 (Nm)
 */
float LQR_Pitch_CalcTorque(float target_theta, float actual_theta, float actual_omega, float motor_pos);

/**
 * @brief  Yaw轴LQR电流计算
 * @param  target_theta: 目标角度 (rad)
 * @param  actual_theta: 实际角度 (rad) - 来自陀螺仪
 * @param  actual_omega: 实际角速度 (rad/s) - 来自陀螺仪
 * @retval 控制电流
 */
float LQR_Yaw_CalcCurrent(float target_theta, float actual_theta, float actual_omega);

/**
 * @brief  摩擦轮LQR电流计算
 * @param  target_speed: 目标速度 (rad/s)
 * @param  actual_rpm: 实际转速 (rpm)
 * @retval 控制电流
 */
float LQR_Friction_CalcCurrent(float target_speed, float actual_rpm);

/**
 * @brief  拨弹盘速度环LQR电流计算
 * @param  target_speed: 目标速度 (rad/s)
 * @param  actual_shaft_speed: 实际输出轴转速 (rpm)
 * @retval 控制电流
 */
float LQR_Rammer_SpeedCalcCurrent(float target_speed, float actual_shaft_speed);

/**
 * @brief  拨弹盘位置环LQR电流计算 (单发模式)
 * @param  target_pos: 目标位置 (rad)
 * @param  actual_pos_deg: 实际位置 (degree)
 * @param  actual_shaft_speed: 实际输出轴转速 (rpm)
 * @retval 控制电流
 */
float LQR_Rammer_PosCalcCurrent(float target_pos, float actual_pos_deg, float actual_shaft_speed);

/**
 * @brief  力矩到电流转换 (GM6020电机)
 * @param  tau: 力矩 (Nm)
 * @param  kt: 力矩常数 (Nm/A)
 * @param  ratio: 减速比
 * @retval 电流值
 */
float LQR_TorqueToCurrent_GM6020(float tau, float kt, float ratio);

/**
 * @brief  力矩到电流转换 (M3508电机)
 * @param  tau: 力矩 (Nm)
 * @param  kt: 力矩常数 (Nm/A)
 * @param  ratio: 减速比
 * @retval 电流值
 */
float LQR_TorqueToCurrent_M3508(float tau, float kt, float ratio);

/**
 * @brief  力矩到电流转换 (M2006电机)
 * @param  tau: 力矩 (Nm)
 * @param  kt: 力矩常数 (Nm/A)
 * @param  ratio: 减速比
 * @retval 电流值
 */
float LQR_TorqueToCurrent_M2006(float tau, float kt, float ratio);

/**
 * @brief  限幅函数
 * @param  value: 输入值
 * @param  max: 上限
 * @param  min: 下限
 * @retval 限幅后的值
 */
float LQR_Constrain(float value, float max, float min);

#endif /* __LQR_GIMBAL_H */
