/**
  * @file       robot_param_omni_infantry.h
  * @brief      这里是全向轮步兵机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_MODE_CHANNEL 0  // 选择底盘状态 开关通道号

#define CHASSIS_TYPE CHASSIS_MECANUM_WHEEL       // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT      // 选择云台类型
#define SHOOT_TYPE SHOOT_NONE                    // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL          // 选择控制类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  //选择机械臂类型

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
#define WHEEL_RADIUS 0.08f                 //(m)轮子直径
#define WHEEL_CENTER_DISTANCE 0.31340867f  //(m)轮子到车的距离（0.22 + 0.21）

//motor parameters ---------------------
//电机ID ---------------------
#define WHEEL_1_ID (1)
#define WHEEL_2_ID (2)
#define WHEEL_3_ID (3)
#define WHEEL_4_ID (4)

//电机CAN ---------------------
#define WHEEL_1_CAN (1)
#define WHEEL_2_CAN (1)
#define WHEEL_3_CAN (1)
#define WHEEL_4_CAN (1)

//电机种类
#define WHEEL_1_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_2_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_3_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_4_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机方向
#define WHEEL_1_DIRECTION (1)
#define WHEEL_2_DIRECTION (1)
#define WHEEL_3_DIRECTION (-1)
#define WHEEL_4_DIRECTION (-1)

//电机减速比
#define WHEEL_1_RATIO (19)
#define WHEEL_2_RATIO (19)
#define WHEEL_3_RATIO (19)
#define WHEEL_4_RATIO (19)

//电机模式
#define WHEEL_1_MODE (0)
#define WHEEL_2_MODE (0)
#define WHEEL_3_MODE (0)
#define WHEEL_4_MODE (0)

//PID parameters ---------------------
//驱动轮速度环PID参数
#define KP_MECANNUM_VEL (50.0f)
#define KI_MECANNUM_VEL (0.3f)
#define KD_MECANNUM_VEL (0.3f)
#define MAX_IOUT_MECANNUM_VEL (10000.0f)
#define MAX_OUT_MECANNUM_VEL (30000.0f)

//云台跟随角度环PID参数
#define KP_CHASSIS_FOLLOW_GIMBAL (2.0f)
#define KI_CHASSIS_FOLLOW_GIMBAL (0.01f)
#define KD_CHASSIS_FOLLOW_GIMBAL (0.5f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (1.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL (3.0f)

//RC parametes ---------------------
//遥控器相关参数
#define CHASSIS_RC_DEADLINE (5.0f)      // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)   //遥控器最大量程
#define CHASSIS_RC_MAX_SPEED (3.0f)     //最大速度(m/s)
#define CHASSIS_RC_MAX_VELOCITY (2.0f)  //最大角速度(rad/s) 仅用于无云台模式

/*-------------------- Gimbal --------------------*/
//gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)1000

//mouse sensitivity ---------------------
#define MOUSE_SENSITIVITY (100000.0f)
//remote controller sensitivity ---------------------
#define REMOTE_CONTROLLER_SENSITIVITY (100000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)
//motor parameters ---------------------
//电机id
#define GIMBAL_DIRECT_YAW_ID ((uint8_t)4)
#define GIMBAL_DIRECT_PITCH_ID ((uint8_t)5)

//电机can口
#define GIMBAL_DIRECT_YAW_CAN ((uint8_t)2)
#define GIMBAL_DIRECT_PITCH_CAN ((uint8_t)2)

//电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION (1)
#define GIMBAL_DIRECT_PITCH_DIRECTION (1)

//减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_DIRECT_YAW_MODE (0)
#define GIMBAL_DIRECT_PITCH_MODE (0)

//physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH (-0.31f)
#define GIMBAL_LOWER_LIMIT_PITCH (0.70f)

//电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID (0.00f)  //云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_DIRECT_YAW_MID (-1.78f)   //云台初始化正对齐的时候使用的yaw轴正中心量
//PID parameters ---------------------
//YAW ANGLE
#define KP_GIMBAL_YAW_ANGLE (20.0f)
#define KI_GIMBAL_YAW_ANGLE (0.0f)
#define KD_GIMBAL_YAW_ANGLE (0.8f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.05f)
#define MAX_OUT_GIMBAL_YAW_ANGLE (20.0f)
//VELOCITY:角速度
#define KP_GIMBAL_YAW_VELOCITY (8000.0f)
#define KI_GIMBAL_YAW_VELOCITY (20.0f)
#define KD_GIMBAL_YAW_VELOCITY (0.1f)
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (10000.0f)
#define MAX_OUT_GIMBAL_YAW_VELOCITY (10000.0f)

//PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE (150.0f)  //90
#define KI_GIMBAL_PITCH_ANGLE (0.1f)    //0.02
#define KD_GIMBAL_PITCH_ANGLE (0.0f)    //50
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (40.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE (500.0f)
//VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY (80.0f)
#define KI_GIMBAL_PITCH_VELOCITY (0.1f)
#define KD_GIMBAL_PITCH_VELOCITY (100.0f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (1000.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY (10000.0f)
/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
#define FRIC_RADIUS 0.03f  // (m)摩擦轮半径
#define BULLET_NUM 8       // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1          // 定义枪管个数（一个枪管2个摩擦轮）

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DM_4310)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机ID
#define TRIGGER_MOTOR_ID 1
#define FRIC_MOTOR_R_ID 7
#define FRIC_MOTOR_L_ID 6

//电机can口
#define TRIGGER_MOTOR_CAN 2
#define FRIC_MOTOR_R_CAN 1
#define FRIC_MOTOR_L_CAN 1

//电机std_id
#define FRIC_STD_ID 0x1FF

//单环拨弹速度
#define TRIGGER_SPEED (-7.0f)
//摩擦轮速度
#define FRIC_R_SPEED (1500.0f)
#define FRIC_L_SPEED (-1490.0f)
#define FRIC_SPEED_LIMIT (700.0f)

/*BLOCK&REVERSE parameters------------*/

//初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED 0.1f
#define BLOCK_TIME 1000
#define REVERSE_TIME 700
#define REVERSE_SPEED (1.0f)  // (rad/s)

/*PID parameters ---------------------*/

#define TRIGGER_ANGEL_PID_KP (20.0f)
#define TRIGGER_ANGEL_PID_KI (0.1f)
#define TRIGGER_ANGEL_PID_KD (0.0f)

#define TRIGGER_ANGEL_PID_MAX_OUT (15.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (100.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (1000.0f)
#define FIRC_SPEED_PID_KI (1.0f)
#define FRIC_SPEED_PID_KD (200.0f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (6000.0f)

#endif /* INCLUDED_ROBOT_PARAM_H */
