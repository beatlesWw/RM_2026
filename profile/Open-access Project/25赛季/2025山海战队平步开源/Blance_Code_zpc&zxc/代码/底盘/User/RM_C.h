#ifndef __RM_C_H
#define __RM_C_H

/*==========延时==========*/
#include "Delay.h"						//延时

/*==========内部资源==========*/
#include "TIM.h"						//定时器

/*==========硬件驱动==========*/
#include "LED.h"						//LED
#include "Buzzer.h"						//蜂鸣器
#include "Remote.h"						//遥控器
#include "BMI088.h"						//陀螺仪
#include "GM6020.h"						//GM6020
#include "MF9025.h"						//MF9025
#include "DM_J8009.h"					//DM_J8009

/*==========通讯协议==========*/
#include "UART.h"						//串口
#include "CAN.h"						//CAN

/*==========滤波算法==========*/
#include "kalman_filter.h"				//卡尔曼滤波
#include "QuaternionEKF.h"				//扩展卡尔曼滤波EKF

/*==========控制算法==========*/
#include "PID.h"						//PID

/*==========算法==========*/
#include "ins_task.h"					//姿态解算EKF算法

/*==========功能==========*/
#include "LinkCheck.h"					//CAN连接检测
#include "Warming.h"					//报警
#include "CToC.h"						//板间通讯
#include "CloseLoopControl.h"			//闭环控制
#include "AttitudeAlgorithms.h"			//姿态解算
#include "IMUTemperatureControl.h"		//陀螺仪恒温控制

/*==========模型==========*/
#include "Data.h"						//数据处理
#include "Observer.h"					//观测器
#include "Leg_Controller.h"				//腿部控制器
#include "LQR.h"						//LQR控制
#include "Locomotion_Controller.h"		//综合运动控制
#include "MotionEstimation.h"			//运动估计

/*==========车体==========*/
#include "Parameter.h"					//参数
#include "RefereeSystem.h"				//裁判系统
#include "RefereeSystem_CRCTable.h"		//裁判系统CRC数组
#include "Ultra_CAP.h"					//超电
#include "UI.h"							//UI
#include "Chassis.h"					//底盘

/*==========安全==========*/
#include "Lock.h"						//安全锁

#include "STP.h"

#endif
