#ifndef GIMBAL_YAW_TASK_H
#define GIMBAL_YAW_TASK_H

#include "stdint.h"

/**
 * @brief 云台Yaw轴控制任务入口（由FreeRTOS调度）
 *        双环PID控制：外环角度环 + 内环速度环
 *        控制频率500Hz (2ms)
 */
void Gimbal_Yaw_task(void);

/**
 * @brief FreeRTOS任务启动函数
 */
void StartGimbalYaw_Task(void const *argument);

#endif // GIMBAL_YAW_TASK_H
