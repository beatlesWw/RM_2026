#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm_8009_drv.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "user_lib.h"
//遥控器状态宏 （本质是在数组中元素的位置）

//in the beginning of task ,wait a time
//任务开始后空闲一段时间
#define CHASSIS_TASK_INIT_TIME 1000

//the channel of choosing chassis mode
//选择底盘状态的开关通道号
#define CHASSIS_MODE_CHANNEL 0


//reducation of 3508 motor
//m3508电机的减速比
#define M3508_MOTOR_REDUCATION 15.764705882f

//m3508 rpm change to chassis speed   576.096  3.14 *07425 = 0.233145
//m3508转子转速(rpm)转化成底盘速度(m/s)的比例，c=pi*r/(30*k)，k为电机减速比
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.0004998609952f

//m3508 rpm change to motor angular velocity
//m3508转子转速(rpm)转换为输出轴角速度(rad/s)的比例
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.00664267f

//m3508 current change to motor torque
//m3508转矩电流(-16384~16384)转为成电机输出转矩(N.m)的比例
//c=20/16384*0.3，   
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f




#define ROLL_PID_KP 0.0f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 0.0f

//#define ROLL_PID_KP 25.0f
//#define ROLL_PID_KI 0.0f 
//#define ROLL_PID_KD 1.0f
#define ROLL_PID_MAX_OUT  100.0f
#define ROLL_PID_MAX_IOUT 0.0f



//#define TP_PID_KP 0.0f
//#define TP_PID_KI 0.0f 
//#define TP_PID_KD 0.0f
//调节roll之前
//#define TP_PID_KP 52.0f
//#define TP_PID_KI 0.0f 
//#define TP_PID_KD 1.314f

#define TP_PID_KP 50.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 10.0f
#define TP_PID_MAX_OUT  5.0f
#define TP_PID_MAX_IOUT 0.0f


#define TURN_PID_KP 10.0f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 0.8f
//#define TURN_PID_KP 0.0f
//#define TURN_PID_KI 0.0f 
//#define TURN_PID_KD 0.0f
#define TURN_PID_MAX_OUT  3.0f//轮毂电机的额定扭矩
#define TURN_PID_MAX_IOUT 0.0f




//底盘控制模式
typedef enum
{
  CHASSIS_REMOTE_MODE,  //遥控模式
  CHASSIS_BALANCE_MODE, //平衡模式
  CHASSIS_DOWN_MODE,    //DOWN模式
} chassis_mode_e;


typedef struct
{
	//关节电机参数
	Joint_Motor_t joint_motor[4];
	//轮毂电机参数  
//	Wheel_Motor_t wheel_motor[2];

//应该是电池
	float vbus;
	uint8_t vbus_mode; // 1-For 4s 
										//	2-For 6s

	float v_set;//期望速度，单位是m/s
	float x_set;//期望位置，单位是m
	float target_v;

	float turn_set;//期望yaw轴弧度
	float roll_set;	//期望roll轴弧度
	float roll_target;
	float roll_x;
	float phi_set;
	float theta_set;

	float target_leg;	
	float leg_set;//期望腿长，单位是m
	float last_leg_set;

	float v_filter;//滤波后的车体速度，单位是m/s
	float v_filter2;//滤波后的车体速度，单位是m/s
	float x_filter;//滤波后的车体位置，单位是m
	
	
	float v;
	float x;
		
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;//两腿夹角误差
	
			
	float turn_T;//yaw轴补偿
	float roll_f0;//roll轴补偿
		
	float leg_tp;//防劈叉补偿
	
	uint8_t start_flag;//启动标志

	uint8_t jump_flag_r;//右腿跳跃标志
	uint8_t jump_flag_l;//左腿跳跃标志
	
	uint8_t prejump_flag;//预跳跃标志
	uint8_t recover_flag;//一种情况下的倒地自起标志
	uint8_t help_jump_flag;
	uint8_t	recover_time;
	
	
  const RC_ctrl_t *chassis_RC;  //获取遥控器指针	
	
	chassis_mode_e chassis_mode;               //底盘控制模式状态机
    chassis_mode_e last_chassis_mode;          //底盘上次控制模式状态机
	
 motor_measure_t wheel_motor[2];

	
} chassis_t;









extern void ChassisR_task(void);
extern void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,pid_type_def *legr);
extern void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void mySaturate(float *in,float min,float max);
extern void Pensation_init(pid_type_def *roll,pid_type_def *Tp,pid_type_def *turn);
extern void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,pid_type_def *leg);
//extern void chassis_set_mode(chassis_t *chassis_move_mode);
extern void chassis_recover(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins,float *LQR_K,pid_type_def *leg  );
extern uint8_t recover_detectR(chassis_t *chassis);

#endif


