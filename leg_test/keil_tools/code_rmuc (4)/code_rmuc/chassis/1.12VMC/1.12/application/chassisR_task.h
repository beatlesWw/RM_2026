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
//ң����״̬�� ����������������Ԫ�ص�λ�ã�

//in the beginning of task ,wait a time
//����ʼ�����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 1000

//the channel of choosing chassis mode
//ѡ�����״̬�Ŀ���ͨ����
#define CHASSIS_MODE_CHANNEL 0



#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S

#define CHASSIS_LEG_KEY KEY_PRESSED_OFFSET_CTRL 









//reducation of 3508 motor
//m3508����ļ��ٱ�
#define M3508_MOTOR_REDUCATION 15.764705882f

//m3508 rpm change to chassis speed   576.096  3.14 *07425 = 0.233145
//m3508ת��ת��(rpm)ת���ɵ����ٶ�(m/s)�ı�����c=pi*r/(30*k)��kΪ������ٱ�
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.0004998609952f

//m3508 rpm change to motor angular velocity
//m3508ת��ת��(rpm)ת��Ϊ�������ٶ�(rad/s)�ı���
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.00664267f

//m3508 current change to motor torque
//m3508ת�ص���(-16384~16384)תΪ�ɵ�����ת��(N.m)�ı���
//c=20/16384*0.3��   
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f




#define ROLL_PID_KP 40.0f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 0.1f

//#define ROLL_PID_KP 25.0f
//#define ROLL_PID_KI 0.0f 
//#define ROLL_PID_KD 1.0f
#define ROLL_PID_MAX_OUT  100.0f
#define ROLL_PID_MAX_IOUT 0.0f



//#define TP_PID_KP 0.0f
//#define TP_PID_KI 0.0f 
//#define TP_PID_KD 0.0f
//����roll֮ǰ
//#define TP_PID_KP 15.0f
//#define TP_PID_KI 0.0f 
//#define TP_PID_KD 0.8f

////#define TP_PID_KP 10.0f
////#define TP_PID_KI 0.0f 
////#define TP_PID_KD 0.8f
//#define TP_PID_MAX_OUT  5.0f
//#define TP_PID_MAX_IOUT 0.0f


//#define TURN_PID_KP 5.0f
//#define TURN_PID_KI 0.0f 
//#define TURN_PID_KD 0.8f
////#define TURN_PID_KP 0.0f
////#define TURN_PID_KI 0.0f 
////#define TURN_PID_KD 0.0f
//#define TURN_PID_MAX_OUT  3.0f//��챵���ĶŤ��
//#define TURN_PID_MAX_IOUT 0.0f



#define TP_PID_KP 25.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 8.0f
#define TP_PID_MAX_OUT  5.0f
#define TP_PID_MAX_IOUT 0.0f


//#define TURN_PID_KP 6.0f
//#define TURN_PID_KI 0.0f 
//#define TURN_PID_KD 200.0f
#define TURN_PID_KP 15.0f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 2.5f
#define TURN_PID_MAX_OUT  5.0f//��챵���ĶŤ��
#define TURN_PID_MAX_IOUT 0.0f



#define WZ_PID_KP 1.0f
#define WZ_PID_KI 0.0f 
#define WZ_PID_KD 0.8f
//#define TURN_PID_KP 0.0f
//#define TURN_PID_KI 0.0f 
//#define TURN_PID_KD 0.0f
#define WZ_PID_MAX_OUT  1.5f//��챵���ĶŤ��
#define WZ_PID_MAX_IOUT 0.0f













//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 18000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f  
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f



#define POWER_PID_KP 9.0f
#define POWER_PID_KI 0.001f
#define POWER_PID_KD -0.08f
#define POWER_PID_MAX_OUT 10.0f
#define POWER_PID_MAX_IOUT 0.2f




//���̿���ģʽ
typedef enum
{
  CHASSIS_REMOTE_MODE,  //ң��ģʽ
  CHASSIS_BALANCE_MODE, //ƽ��ģʽ
  CHASSIS_DOWN_MODE,    //DOWNģʽ
} chassis_mode_e;


typedef struct
{
	//关节电机参数
 	Joint_Motor_t joint_motor[5];
	//轮毂电机参数  
    //Wheel_Motor_t wheel_motor[2];
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
	float v_filter2;
	
	float v_wheel;
	float x_filter;//滤波后的车体位置，单位是m
	float x_filter2;//滤波后的车体位置，单位是m
	
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

    motor_measure_t wheel_motor[5];
    uint8_t DUBS_ON;

    pid_type_def buffer_pid;
    pid_type_def motor_speed_pid[2];  
	
	float yaw_motor_angle;
	float relative_angle;
	
	motor_measure_t motor_chassis[5];
	
	float Wz_set;
	float Wz;
	uint8_t w_flag;	
	uint8_t move_flag;
	
} chassis_t;

extern void ChassisR_task(void);
extern void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,pid_type_def *legr,pid_type_def *wheel);
extern void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void mySaturate(float *in,float min,float max);
extern void Pensation_init(pid_type_def *roll,pid_type_def *Tp,pid_type_def *turn,pid_type_def *wz);
extern void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,pid_type_def *leg);
//extern void chassis_set_mode(chassis_t *chassis_move_mode);
extern void chassis_recover(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins,float *LQR_K,pid_type_def *leg);
extern uint8_t recover_detect(chassis_t *chassis);
extern void control_motor(chassis_t *chassis );

#endif



