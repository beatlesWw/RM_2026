#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 8

/* DM电机的控制参数范围定义 */
//DM公用
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_D_MIN 0.0f
#define DM_D_MAX 5.0f
//DM4310
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f
//DM8009
#define DM_P_MIN  -12.56637f
#define DM_P_MAX  12.56637f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_8009_T_MIN  (-54.0f)
#define DM_8009_T_MAX   54.0f
//DM3519
#define DM_3519_V_MIN  (-200.0f)
#define DM_3519_V_MAX  200.0f
#define DM_T_W_MIN (-10.0f)
#define DM_T_W_MAX 10.0f

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque; //电机扭矩
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;

typedef struct 
{
    DM_Motor_Measure_s measure;// 电机反馈的数据
    Motor_Control_Setting_s motor_settings;
    Motor_Type_e motor_type;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
//TODO：这里偷懒了，还是找个合适的结构体存着吧
    float vel;
    float T_Gimbal_dYaw;
    float T_Gimbal_Yaw;
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit();
void enable_motor_mode();

/**
 * @brief 直接发送两个DM电机的CAN指令（零延迟，用于高频控制任务）
 * @param motor1 第一个电机实例
 * @param motor2 第二个电机实例
 * @param torque1 第一个电机的力矩指令
 * @param torque2 第二个电机的力矩指令
 * @note 此函数绕过独立任务，直接在调用任务中发送CAN指令，实现零延迟控制
 */
void DMMotorSendPair(DMMotorInstance *motor, float torque);

void DMMotorSendPair_W(DMMotorInstance *motor, float torque);

void DMMotorSendPair_Yaw(DMMotorInstance *motor, float torque);

void DMMotorSend_POS(DMMotorInstance *motor, float pos,float vel,float Kp,float Kd,float T);
#endif // !DMMOTOR