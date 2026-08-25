/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "master_process.h"
#include "daemon.h"
#include "buzzer.h"
#include "bsp_log.h"

//轮腿新增任务
#include "ChassisR_task.h"
#include "ChassisL_task.h"
#include "observe_task.h"
#include "dmmotor.h"
#include "Gimbal_Yaw_task.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;
osThreadId ChassisR_TaskHandle;
osThreadId ChassisL_TaskHandle;
osThreadId observe_TaskHandle;
osThreadId gimbalYaw_TaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);
void StartChassisR_task(void const * argument);
void StartChassisL_task(void const * argument);
void StartObserve_Task(void const * argument);
void StartGimbalYaw_Task(void const * argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    // 创建右腿控制任务，优先级最高，堆栈1024字节
    osThreadDef(instask, StartINSTASK, osPriorityRealtime, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); 

    osThreadDef(CHASSISR_TASK, StartChassisR_task, osPriorityAboveNormal, 0, 512);
    ChassisR_TaskHandle = osThreadCreate(osThread(CHASSISR_TASK), NULL);

    osThreadDef(CHASSISL_TASK, StartChassisL_task, osPriorityAboveNormal, 0, 512);
    ChassisL_TaskHandle = osThreadCreate(osThread(CHASSISL_TASK), NULL);

    osThreadDef(GIMBAL_YAW_TASK, StartGimbalYaw_Task, osPriorityAboveNormal, 0, 128);
    gimbalYaw_TaskHandle = osThreadCreate(osThread(GIMBAL_YAW_TASK), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);
    
    osThreadDef(OBSERVE_TASK, StartObserve_Task, osPriorityHigh, 0, 256);
    observe_TaskHandle = osThreadCreate(osThread(OBSERVE_TASK), NULL);

    // osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    // robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    // osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    // uiTaskHandle = osThreadCreate(osThread(uitask), NULL);

    // osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 512);
    // motorTaskHandle = osThreadCreate(osThread(motortask), NULL);
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init(); // 确保BMI088被正确初始化.
    LOGINFO("[freeRTOS] INS Task Start");
    for (;;)
    {
        // 1kHz
        ins_start = DWT_GetTimeline_ms();
        INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;
        if (ins_dt > 1)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]", &ins_dt);
        // VisionSend(); // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;)
    {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        BuzzerTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 10)
            LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
        osDelay(10);
    }
}

__attribute__((noreturn)) void StartChassisR_task(void const *argument)
{
    LOGINFO("[freeRTOS] ChassisR core Task Start");
    for (;;){ 
        ChassisR_task();
        // osDelay(1);
    }
}

__attribute__((noreturn)) void StartChassisL_task(void const *argument)
{
    LOGINFO("[freeRTOS] ChassisL core Task Start");
    for (;;){ 
        ChassisL_task();
        // osDelay(1);
    }
}

__attribute__((noreturn)) void StartObserve_Task(void const *argument)
{
    static float observe_dt;
    static float observe_start;
    LOGINFO("[freeRTOS] OBSERVE Task Start");
    for (;;){
        observe_task();
    }
}

__attribute__((noreturn)) void StartGimbalYaw_Task(void const *argument)
{
    LOGINFO("[freeRTOS] Gimbal Yaw Task Start");
    for (;;){
        Gimbal_Yaw_task();
    }
}

// __attribute__((noreturn)) void StartROBOTTASK(void const *argument)
// {
//     static float robot_dt;
//     static float robot_start;
//     LOGINFO("[freeRTOS] ROBOT core Task Start");
//     // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
//     for (;;)
//     {
//         robot_start = DWT_GetTimeline_ms();
//         RobotTask();
//         robot_dt = DWT_GetTimeline_ms() - robot_start;
//         if (robot_dt > 5)
//             LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%f]", &robot_dt);
//         osDelay(5);
//     }
// }

// __attribute__((noreturn)) void StartUITASK(void const *argument)
// {
//     LOGINFO("[freeRTOS] UI Task Start");
//     MyUIInit();
//     LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
//     for (;;)
//     {
//         // 每给裁判系统发送一包数据会挂起一次,详见UITask函数的refereeSend()
//         UITask();
//         osDelay(1); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
//     }
// }

// __attribute__((noreturn)) void StartMOTORTASK(void const *argument)
// {
//     static float motor_dt;
//     static float motor_start;
//     LOGINFO("[freeRTOS] MOTOR Task Start");
//     for (;;)
//     {
//         motor_start = DWT_GetTimeline_ms();
//         MotorControlTask();
//         motor_dt = DWT_GetTimeline_ms() - motor_start;
//         if (motor_dt > 1)
//             LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
//         osDelay(1);  
//     }
// }