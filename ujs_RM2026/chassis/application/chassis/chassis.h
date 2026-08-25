#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>
#include "dmmotor.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "robot_def.h"
#include "can_comm.h"
#include "VMC.h"

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 *        初始化所有电机实例(左右腿关节电机和轮毂电机)、IMU、超级电容等模块
 */
extern volatile uint8_t chassis_init_done; // ChassisInit()完成后置1，各任务等待此标志

void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行(当前框架下不使用)
 *        改为使用ChassisR_task()和ChassisL_task()分别控制左右腿
 */
void ChassisTask();

/* ==================== 电机实例访问器函数 ==================== */
/**
 * @brief 获取右前关节电机实例(T1_r)
 * @return DMMotorInstance* 右前关节电机指针
 */
DMMotorInstance* Chassis_GetMotor_T1R(void);

/**
 * @brief 获取右后关节电机实例(T2_r)
 * @return DMMotorInstance* 右后关节电机指针
 */
DMMotorInstance* Chassis_GetMotor_T2R(void);

/**
 * @brief 获取左前关节电机实例(T1_l)
 * @return DMMotorInstance* 左前关节电机指针
 */
DMMotorInstance* Chassis_GetMotor_T1L(void);

/**
 * @brief 获取左后关节电机实例(T2_l)
 * @return DMMotorInstance* 左后关节电机指针
 */
DMMotorInstance* Chassis_GetMotor_T2L(void);

/**
 * @brief 获取右侧轮毂电机实例(W_r)
 * @return DJIMotorInstance* 右侧轮毂电机指针
 */
DMMotorInstance* Chassis_GetMotor_WR(void);

/**
 * @brief 获取左侧轮毂电机实例(W_l)
 * @return DJIMotorInstance* 左侧轮毂电机指针
 */
DMMotorInstance* Chassis_GetMotor_WL(void);

/**
 * @brief 获取Yaw电机的数据指针
 * @note 用于左右腿任务读取Yaw数据
 */
DMMotorInstance* Chassis_GetMotor_Yaw(void);

/**
 * @brief 获取底盘IMU数据指针
 * @return attitude_t* IMU姿态数据指针
 */
attitude_t* Chassis_GetIMUData(void);

/**
 * @brief 获取板间通信数据指针
 * @note 用于左右腿任务读取遥控器数据
 */
CANCommInstance*   Chassis_CAN_COMM(void);

/**
 * @brief 轮毂电机延时启动控制
 * @param first_startup 第一次启动标志指针
 * @param startup_time 启动时间戳指针
 * @param wheel_motor_enabled 轮毂电机使能标志指针
 * @param delay_ms 延时时间(毫秒)
 * @return uint8_t 返回当前轮毂电机是否可以启动 (1=可以启动, 0=延时中)
 */
uint8_t Chassis_WheelMotorStartupDelay(uint8_t *first_startup, uint32_t *startup_time, 
                                        uint8_t *wheel_motor_enabled, float delay_ms);

/**
 * @brief 轻量化DM电机使能帧
 * @param motor DM电机实例指针
 */
void Chassis_DMMotorEnable(DMMotorInstance *motor);

/**
 * @brief 重置轮毂电机启动状态
 * @param first_startup 第一次启动标志指针
 * @param startup_time 启动时间戳指针
 * @param wheel_motor_enabled 轮毂电机使能标志指针
 */
void Chassis_ResetWheel(uint8_t *first_startup, uint32_t *startup_time, 
                                uint8_t *wheel_motor_enabled);

/**
 * @brief 角度转换为[-pi, pi]范围内
 * @param angle 待转换的角度
 * @return float 转换后的角度
 */
float Chassis_WrapAngleToPi(float angle);

void Chassis_ResetMoveState(Balance_Chassis_e *chassis, float x_set);
void Chassis_ResetRotateState(Balance_Chassis_e *chassis);
void Chassis_UpdateTotalYaw(Balance_Chassis_e *chassis, float cmd_yaw, uint8_t rotate_start);
#endif // CHASSIS_H