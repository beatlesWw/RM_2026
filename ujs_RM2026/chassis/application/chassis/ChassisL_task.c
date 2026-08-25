/****************************LEFT*******************************/
/**
 * @file ChassisL_task.c
 * @author UJS Wheel-Leg Balance Infantry Team
 * @brief 左腿控制任务实现文件
 *  Version    Date            Author       Modification
 *  V1.0.0     Dec-29-2025     刘安东        1. done
 * 
 * @details 左腿控制任务负责：
 *          - 左前关节电机(T1_l, CAN1, ID:0x03)和左后关节电机(T2_l, CAN1, ID:0x04)的VMC控制
 *          - 左侧轮毂电机(W_l, CAN1, ID:2)的速度控制
 *          - 左腿VMC运动学解算和LQR控制
 *          - 左腿离地检测和姿态补偿
 * 
 * @note 本任务与ChassisR_task并行运行，共享chassis.c中初始化的电机实例
 *       任务优先级：osPriorityAboveNormal，堆栈大小：512字节，运行频率：1kHz
 * 
 * @hardware 左腿硬件配置：
 *           - T1_l: DM8009电机，CAN1总线，发送ID:0x03，接收ID:0x13
 *           - T2_l: DM8009电机，CAN1总线，发送ID:0x04，接收ID:0x14
 *           - W_l:  M3508电机，CAN1总线， ID:2
 */

#include <math.h>
#include <stdio.h>
#include "ChassisL_task.h"
#include "cmsis_os.h"
#include "VMC.h"
#include "can_comm.h"
#include "user_lib.h"
#include "power_control.h"
#include "bsp_log.h"
#include "chassis.h"
#include "dmmotor.h"
#include "controller.h"
#include "PID_controll.h"
#include "spring_comp.h"

/* ==================== 左腿全局变量 ==================== */
vmc_leg_t left;  // 左腿VMC结构体
extern float Poly_Coefficient[12][4];  // LQR增益多项式系数（在ChassisR_task.c中定义）
uint8_t left_flag = 0;  // 左腿离地标志
extern vmc_leg_t right;
extern uint8_t right_flag;  // 右腿离地标志（在ChassisR_task.c中定义）
int jump_left_flag = 0;  // 左腿跳跃标志
extern Balance_Chassis_e chassis_move_balance;	
extern Balance_Motor_t balance_motor;            // 平衡步兵电机结构体	
extern Balance_Chassis_Ctrl_Cmd_s chassis_ctrl;  // 双腿控制命令结构体
extern uint32_t CHASSR_TIME;
extern float W_set_turn;
extern float Tp_roll;   // 来自右腿任务的roll补偿输出
float  left_detet =0;
extern Chassis_Upload_Data_s  chassis_feedback;      //回传数据
extern CANCommInstance       *chassis_comm;   
extern Chassis_Ctrl_Cmd_s     chassis_cmd ;           // 遥控器控制命令指针
extern float theta_diff;
/* ==================== 左腿任务私有变量 ==================== */
static DMMotorInstance *T1_l = NULL;              // 左前关节电机实例
static DMMotorInstance *T2_l = NULL;              // 左后关节电机实例
static DMMotorInstance *W_l = NULL;               // 左侧轮毂电机实例
static attitude_t *Chassis_IMU_data = NULL;       // IMU姿态数据指针       
static uint8_t initialized = 0;                   // 初始化完成标志 
static uint8_t first_startup = 1;
static uint32_t startup_time = 0;
static uint8_t wheel_motor_enabled = 0;
static float L_enable_time_ms = 0.0f;
static float L_dm_enable_time_ms = 0.0f;

#define DM_ENABLE_INTERVAL_MS 100.0f
#define GROUND_DETECTION_RECOVER_DELAY_MS 8000.0f

// 腿长PID控制器
static PIDInstance LegL_Pid;  // 左腿的腿长PID
static PID_Init_Config_s legl_pid_config = {
    .Kp = LEGL_PID_KP,
    .Ki = LEGL_PID_KI,
    .Kd = LEGL_PID_KD,
    .MaxOut = LEGL_PID_MAX_OUT,
    .IntegralLimit = LEGL_PID_MAX_IOUT,
    .DeadBand = 0.0f,
    .Improve = PID_IMPROVE_NONE
};


static PIDInstance Theta_err_Pid_L;
static PID_Init_Config_s theta_err_pid_config_L = {
    .Kp = 55.0f,
    .Ki = 0.0f,
    .Kd = 18.0f,
    .MaxOut = 8.0f,
    .IntegralLimit = 0.0f,
    .DeadBand = 0.0f,
    .Improve = PID_IMPROVE_NONE
};

static PIDInstance Theta_gyro_err_Pid_L;
static PID_Init_Config_s theta_gyro_err_pid_config_L = {
    .Kp = 1.5f,
    .Ki = 0.0f,
    .Kd = 3.5f,
    .MaxOut = 12.0f,
    .IntegralLimit = 0.0f,
    .DeadBand = 0.0f,
    .Improve = PID_IMPROVE_NONE
};
// LQR控制器增益数组（12个参数）
extern float LQR_K_R[12];
float LQR_K_L[12] = {
    -11.0224, -2.664, -11.6436, -20.2754, 30.2361, 3.3863,
    10.2554, 0.9963, 1.0764, 3.2532, 40.8928, 3.5782

};

/**
 * @brief 饱和限幅函数
 * @param in 输入值指针
 * @param min 最小值
 * @param max 最大值
 */

static void mySaturate(float *in, float min, float max)
{
    if (*in < min) {
        *in = min;
    } else if (*in > max) {
        *in = max;
    }
}

/**
 * @brief 左腿控制主任务
 * @note 以1kHz频率运行，负责左腿的完整控制流程
 */

static void ChassisL_Init(void)
{
    while (!chassis_init_done) { osDelay(1); }
    T1_l = Chassis_GetMotor_T1L();
    T2_l = Chassis_GetMotor_T2L();
    W_l  = Chassis_GetMotor_WL();
    Chassis_IMU_data = Chassis_GetIMUData();
    DWT_Delay(0.5);

    chassis_move_balance.balance_motor = &balance_motor;
    chassis_move_balance.chassis_move_balance = &chassis_ctrl;

    balance_motor.joint_motor[2] = T1_l;
    balance_motor.joint_motor[3] = T2_l;
    balance_motor.wheel_motor[1] = W_l;

    PIDInit(&LegL_Pid, &legl_pid_config);
    PIDInit(&Theta_err_Pid_L, &theta_err_pid_config_L);
    PIDInit(&Theta_gyro_err_Pid_L, &theta_gyro_err_pid_config_L);
    L_enable_time_ms = DWT_GetTimeline_ms();
    L_dm_enable_time_ms = DWT_GetTimeline_ms();
    initialized = 1;
}

void ChassisL_task(void)
{
    if (!initialized) {
        ChassisL_Init();
    }

    // 主控制循环
    while(1) {        
        // 检查INS是否初始化完成
        if (Chassis_IMU_data == NULL || Chassis_IMU_data->init == 0) {
            osDelay(1);
                continue;
        }       
    
// 0. 检查底盘模式
switch (chassis_cmd.chassis_mode)
    {
case CHASSIS_ZERO_FORCE:
left.phi1 = pi + T1_l->measure.position;
left.phi4 = pi/2 + T2_l->measure.position;
VMC_calc_left(&left, Chassis_IMU_data, ((float)CHASSL_TIME) * 1.5f / 1000.0f);
chassis_ctrl.phi0_err = right.phi0 - left.phi0;
// 倒地自起模式_遥控器左侧拨至中间
if (chassis_cmd.tumble_detect == 1){
        if (DWT_GetTimeline_ms() - L_dm_enable_time_ms >= DM_ENABLE_INTERVAL_MS) {
            Chassis_DMMotorEnable(T1_l);
            Chassis_DMMotorEnable(T2_l);
            Chassis_DMMotorEnable(W_l);
            L_dm_enable_time_ms = DWT_GetTimeline_ms();
        }

// 更新电机角度反馈
        left.F0 = 80.0f;
        VMC_calc_splitter(&left);
        mySaturate(&left.torque_set[0], -25.0f, 25.0f);
        mySaturate(&left.torque_set[1], -25.0f, 25.0f);
        float tumble_vel_cmd = 
          ((((left.theta > 1.0f) && (left.theta < 1.26f)))) ? 0.0f 
        // : (((Chassis_IMU_data->Pitch > 2.8f)&&(Chassis_IMU_data->Pitch < 3.1f)) ? ((theta_diff < -0.16f) ? -3.0f : 0.0f)
        : (((Chassis_IMU_data->Pitch > 0.5f) && (Chassis_IMU_data->Pitch < 1.9f)) ? 3.0f 
        : -3.0f);
         DMMotorSend_POS(T1_l, 0, tumble_vel_cmd, 0, 2.8f, left.torque_set[0]);
         osDelay(1);
         DMMotorSend_POS(T2_l, 0, tumble_vel_cmd, 0, 2.8f, left.torque_set[1]);
        osDelay(1);
        // DMMotorSendPair(T1_l, 0);
        // DMMotorSendPair(T2_l, 0);
        DMMotorSendPair_W(W_l,  0);
            osDelay(1);
                break;
}
// 零力矩制动（左侧开关拨至底下时强制制动）
        left.theta_accum = left.theta;  
        Chassis_ResetMoveState(&chassis_move_balance,0.0f);
        DMMotorDisable(T1_l);
        DMMotorDisable(T2_l);
        DMMotorDisable(W_l);
        Chassis_ResetWheel(&first_startup, &startup_time, &wheel_motor_enabled);
            osDelay(1);
                continue;
case CHASSIS_RUN:
        if (DWT_GetTimeline_ms() - L_dm_enable_time_ms >= DM_ENABLE_INTERVAL_MS) {
            Chassis_DMMotorEnable(T1_l);
            Chassis_DMMotorEnable(T2_l);
            Chassis_DMMotorEnable(W_l);
            L_dm_enable_time_ms = DWT_GetTimeline_ms();
        }

// 1. 更新反馈数据
        chassisL_feedback_update(&chassis_move_balance, T1_l, T2_l, &left, Chassis_IMU_data);

//跨越模式：phi0>2.0且L0>0.32时接管控制，theta>1.2时自动退出
        uint8_t fold_ran_l = 0;  // 检测跨越函数是否执行过
         while (!(right_flag == 1 && left_flag == 1) && LegFold_Override(&left, LQR_K_L, chassis_move_balance.chassis_move_balance->leg_tp_l,&LegL_Pid, SPRING_SIDE_LEFT)) 
         {
            fold_ran_l = 1;     
             // 跨越过程中清零位移速度相关的观测值和目标值
            Chassis_ResetMoveState(&chassis_move_balance, 0.0f);
             
            DMMotorSendPair(T1_l, left.torque_set[0]);
            osDelay(1);
            DMMotorSendPair(T2_l, left.torque_set[1]);
            osDelay(1);
            DMMotorSendPair_W(W_l, 0.0f);
            osDelay(1);
            chassisL_feedback_update(&chassis_move_balance, T1_l, T2_l, &left, Chassis_IMU_data);
            VMC_calc_left(&left, Chassis_IMU_data, ((float)CHASSL_TIME)*1.5f/1000.0f);
            for (int i = 0; i < 12; i++) {
                LQR_K_L[i] = LQR_K_calc(&Poly_Coefficient[i][0], left.L0);
            }
        }
        // 跨越函数刚退出，重置轮毂延时使重进 LQR 时再延时 450ms，并设置期望腿长为 0.18
        if (fold_ran_l) {
            Chassis_ResetWheel(&first_startup, &startup_time, &wheel_motor_enabled);
            chassis_move_balance.chassis_move_balance->leg_set = 0.18f;
            chassis_move_balance.chassis_move_balance->leg_target = 0.35f;  // 退出跨越后收腿
        }

// 2. 执行控制环路计算
        chassisL_control_loop(&chassis_move_balance, &left, Chassis_IMU_data, LQR_K_L);

// 测试
        // left.torque_set[0]=0.0f;
        // left.torque_set[1]=-0.0f;
        // left.wheel_set = 0.0f; 

// 3. 电机输出
        // 第一次起身延时检测：先发关节电机，450ms后再发轮毂电机
        uint8_t wheel_enabled = Chassis_WheelMotorStartupDelay(&first_startup, &startup_time, &wheel_motor_enabled, 550.0f);     
        // 发送关节电机控制帧
        DMMotorSendPair(T1_l, left.torque_set[0]);
        osDelay(1);
        DMMotorSendPair(T2_l, left.torque_set[1]);
        osDelay(1);
        
        // 只有在延时结束后才发送轮毂电机控制帧
        if (wheel_enabled) {
            DMMotorSendPair_W(W_l, left.wheel_set);
        } else {
            DMMotorSendPair_W(W_l, 0.0f);  // 延时期间发送0力矩
        }
        osDelay(1); 
// 4. 发送反馈数据给云台板
        chassis_feedback.chassis_yaw = Chassis_IMU_data->Yaw;
        CANCommSend(chassis_comm, (void*)&chassis_feedback);     
        osDelay(1);
        break;
default:
        osDelay(1);
        break;
        }
    }
}

/**
 * @brief 左腿反馈数据更新
 * @note 更新左腿关节角度、IMU数据、轮毂速度等反馈信息
 */

void chassisL_feedback_update(Balance_Chassis_e *chassis,DMMotorInstance *T1,DMMotorInstance *T2,vmc_leg_t *vmc,attitude_t *ins)
{
    vmc->phi1 =pi+T1->measure.position;
    vmc->phi4 =pi/2+T2->measure.position; 

// 更新左腿IMU数据（左腿Pitch取反）
    chassis->chassis_move_balance->myPithL = 0.0f-ins->Pitch;
    chassis->chassis_move_balance->myPithGyroL=0.0f-ins->Gyro[0];
    chassis->chassis_move_balance->roll = ins->Roll;
    chassis->chassis_move_balance->last_leg_set = chassis->chassis_move_balance->leg_set;
    
    chassis->chassis_move_balance->leg_tp_gyro_l =
    0.0f - 
    // Theta_err_Pid_L.Kp * (chassis->chassis_move_balance->theta_err-0.0f);
    PID_calc(&Theta_err_Pid_L,chassis->chassis_move_balance->theta_err, 0.0f);
    chassis->chassis_move_balance->leg_tp_l =
    // Theta_gyro_err_Pid_L.Kp * (chassis->chassis_move_balance->leg_tp_gyro_l - chassis_move_balance.chassis_move_balance->leg_gyro_l);
    PID_calc(&Theta_gyro_err_Pid_L,chassis->chassis_move_balance->leg_tp_gyro_l, chassis_move_balance.chassis_move_balance->leg_gyro_l);
}

/**
 * @brief 左腿控制环路计算
 * @note 执行VMC运动学解算、LQR控制计算、力矩分配等
 */
void chassisL_control_loop(Balance_Chassis_e *chassis,vmc_leg_t *vmcl,attitude_t *ins,float *LQR_K)
{
// VMC运动学解算（计算theta、d_theta等状态量）
    VMC_calc_left(vmcl, ins, ((float)CHASSL_TIME) * 1.5f / 1000.0f);
    // 根据腿长L0计算LQR增益
    for (int i = 0; i < 12; i++) {
        LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmcl->L0);
    }
// LQR控制器计算轮毂电机输出力矩
    vmcl->wheel_set =0.0f-(
                      LQR_K[0]*(vmcl->theta-0.0f)
					 +LQR_K[1]*(vmcl->d_theta-0.0f)
                     +LQR_K[2]*(chassis->chassis_move_balance->x_filter-(chassis->chassis_move_balance->x_set))
				     +LQR_K[3]*(chassis->chassis_move_balance->v_filter-(chassis->chassis_move_balance->v_set))
                     +LQR_K[4]*(chassis->chassis_move_balance->myPithL-0.005f) 
					 +LQR_K[5]*(chassis->chassis_move_balance->myPithGyroL-0.0f)
                     +W_set_turn
                    // chassis->chassis_move_balance->v_set*15
                      ); 
//关节电机Tp计算    
//LQR控制器计算髋关节输出力矩
    vmcl->Tp = 0.0f-( 
                  LQR_K[6]*(vmcl->theta-0.0f)
                 +LQR_K[7]*(vmcl->d_theta-0.0f)
                 +LQR_K[8]*(chassis->chassis_move_balance->x_filter-(chassis->chassis_move_balance->x_set))
				 +LQR_K[9]*(chassis->chassis_move_balance->v_filter-(chassis->chassis_move_balance->v_set))
				 +LQR_K[10]*(chassis->chassis_move_balance->myPithL-0.005f) 
				 +LQR_K[11]*(chassis->chassis_move_balance->myPithGyroL-0.0f)
                 +chassis->chassis_move_balance->leg_tp_l
                );		

    // mySaturate(&vmcl->Tp, -2.0f, 2.0f);            
    // vmcl->Tp = 0.0f;
    // vmcl->F0 = 0.0f;
// 对轮毂电机输出限幅
    // mySaturate(&vmcl->wheel_set, -5.5f, 5.5f);
// 腿长PID控制：前馈+PID
    slope_following(&chassis->chassis_move_balance->leg_set,&chassis->chassis_move_balance->leg_target,0.01f);
    vmcl->F0 = 
    (50.2f / arm_cos_f32(vmcl->theta)) 
    + PID_calc(&LegL_Pid, vmcl->L0, chassis->chassis_move_balance->leg_target);
    vmcl->F0 = vmcl->F0 + Tp_roll;  // 加上roll补偿	
    vmcl->F0_test = vmcl->F0;
// 气弹簧补偿（等效到腿轴向力F0）
    vmcl->Fv= Fv_Gas_spring(vmcl,&spring_comp_l);
    vmcl->F0 = vmcl->F0 - vmcl->Fv;

 // 左腿离地检测
    left_flag = Ground_detectionL(vmcl, ins);
	if (fabsf(chassis->chassis_move_balance->leg_set - chassis->chassis_move_balance->last_leg_set) > 0.00000002f)
	{
		left_flag = 0;
	}

	if ((DWT_GetTimeline_ms() - L_enable_time_ms) < GROUND_DETECTION_RECOVER_DELAY_MS)
	{
		left_flag = 0;
	}
		if(right_flag == 1 && left_flag == 1)
		{
// 		// 当两腿同时离地并且遥控器没有在控制腿的伸缩时,才认为离地
// 		// 排除跳跃的压缩阶段、上升阶段、跳跃的缩腿阶段
			vmcl->wheel_set = 0.0f;
			vmcl->Tp =0.0f-( 
                         LQR_K[6] * (vmcl->theta_accum - chassis->chassis_move_balance->theta_compensate)
                       + LQR_K[7] * (vmcl->d_theta - 0.0f));
			chassis->chassis_move_balance->x_filter = 0.0f;
			chassis->chassis_move_balance->x_set = chassis->chassis_move_balance->x_filter;
            // chassis->chassis_move_balance->leg_target += 0.0005;
		}
//功率控制
    // chassis_power_control(chassis);

    mySaturate(&vmcl->F0, -300.0f, 300.0f); 
    // VMC力矩分配（将F0和Tp分配到两个关节电机）
    VMC_calc_splitter(vmcl);
}




