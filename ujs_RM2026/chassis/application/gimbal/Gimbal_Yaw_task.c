/****************************GIMBAL YAW*******************************/
/** 
 *  Version    Date            Author       Modification
 *  V1.0.0     Feb-19-2026     Claude       1. done
 *  @verbatim
 ==============================================================================
    云台Yaw轴双环PID控制任务
    外环：角度环，期望值由遥控器 chassis->chassis_move_balance->total_yaw 输入
    内环：速度环，反馈值为 Yaw 电机 DM_Motor_Measure_s 中的 velocity
    控制频率：500Hz (2ms)
 ==============================================================================
**/
#include "Gimbal_Yaw_task.h"
#include "controller.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "dmmotor.h"
#include "robot_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "ins_task.h"
#include "can_comm.h"
/* ==================== 公用变量 ==================== */
extern Chassis_Ctrl_Cmd_s  chassis_cmd;
float yaw_angle_fdb = 0.0f;
/* ==================== 云台Yaw任务私有变量 ==================== */
static DMMotorInstance *Gimbal_Yaw = NULL;       // Yaw电机实例
static uint8_t gimbal_yaw_initialized = 0;       // 初始化完成标志

// 板间通信（获取遥控器数据）
static CANCommInstance *gimbal_chassis_comm = NULL;

// 控制命令结构体
static Balance_Chassis_Ctrl_Cmd_s gimbal_ctrl;
static Balance_Chassis_e gimbal_chassis_e;
static float speed_ref = 0.0f;
static float speed_fdb = 0.0f;
static float torque_out = 0.0f;
static float yaw_pos_raw_last = 0.0f;
static float yaw_pos_continuous = 0.0f;
static uint8_t yaw_pos_continuous_inited = 0;
static float yaw_enable_time_ms = 0.0f;
#define DM_ENABLE_INTERVAL_MS 100.0f

static float GimbalYaw_GetContinuousPos(float raw_pos)
{
    const float pos_range = (DM_P_MAX - DM_P_MIN);
    const float half_range = pos_range * 0.5f;

    if (!yaw_pos_continuous_inited) {
        yaw_pos_raw_last = raw_pos;
        yaw_pos_continuous = raw_pos;
        yaw_pos_continuous_inited = 1;
        return yaw_pos_continuous;
    }

    float delta = raw_pos - yaw_pos_raw_last;
    if (delta > half_range) {
        delta -= pos_range;
    } else if (delta < -half_range) {
        delta += pos_range;
    }

    yaw_pos_continuous += delta;
    yaw_pos_raw_last = raw_pos;
    return yaw_pos_continuous;
}

void Gimbal_Yaw_task(void)
{
// 首次运行时获取电机实例指针并初始化
    if (!gimbal_yaw_initialized) {
        while (!chassis_init_done) { osDelay(1); } // 等待ChassisInit()完成
        Gimbal_Yaw = Chassis_GetMotor_Yaw();
        gimbal_chassis_comm = Chassis_CAN_COMM();
        DWT_Delay(0.5);

// 初始化结构体指针
        gimbal_chassis_e.chassis_move_balance = &gimbal_ctrl;

// 初始化控制参数默认值
        gimbal_ctrl.total_yaw = 0.0f;

        yaw_angle_fdb = GimbalYaw_GetContinuousPos(Gimbal_Yaw->measure.position);

        yaw_enable_time_ms = DWT_GetTimeline_ms();
        gimbal_yaw_initialized = 1;
    }

// 主控制循环
    while(1)
    {
        yaw_angle_fdb = GimbalYaw_GetContinuousPos(Gimbal_Yaw->measure.position);
        speed_fdb = Gimbal_Yaw->measure.velocity;

        if (DWT_GetTimeline_ms() - yaw_enable_time_ms >= DM_ENABLE_INTERVAL_MS) {
            Chassis_DMMotorEnable(Gimbal_Yaw);
            yaw_enable_time_ms = DWT_GetTimeline_ms();
        }

        if(chassis_cmd.chassis_mode == CHASSIS_ZERO_FORCE)
        {
            DMMotorSendPair_Yaw(Gimbal_Yaw, 0.0f);
            osDelay(1);
            continue;
        }
        // DMMotorSendPair_Yaw(Gimbal_Yaw, 0.0f);
        DMMotorSendPair_Yaw(Gimbal_Yaw,chassis_cmd.T_Gimbal_Yaw);
// 1000Hz控制频率，延时1ms
        osDelay(1);
    }
}
