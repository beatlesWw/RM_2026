/****************************RIGHT*******************************/
/** 
 *  Version    Date            Author       Modification
 *  V1.0.0     Dec-29-2025     刘安东        1. done
 *  @verbatim
  ==============================================================================

  ==============================================================================
**/
/****************************CAN2*******************************/
 //C板放置：R标横着 
 //C板CAN线序左L右H
#include <stdio.h>
#include "controller.h"
#include "cmsis_os.h"
#include "VMC.h"
#include "ChassisR_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "user_lib.h"
#include "VMC.h"
#include "ins_task.h"
#include "chassis.h"
#include "can_comm.h"
#include "referee_task.h"
#include "Gimbal_Yaw_task.h"
#include "PID_controll.h"
#include "spring_comp.h"

/* ==================== 右腿任务公用变量 ==================== */
float LQR_K_R[12]={
    -4.4916496, -0.61003828, -2.04249787, -3.45958042, 4.6271553, 1.03048968,
    10.2554, 0.9963, 1.0764, 3.2532, 40.8928, 3.5782};                                                   
vmc_leg_t right;//右腿
extern vmc_leg_t left;
extern float yaw_angle_fdb;
Chassis_Ctrl_Cmd_s     chassis_cmd;           //Chassis_Ctrl_Cmd_s格式的板间通信数据
Chassis_Upload_Data_s  chassis_feedback;      //回传数据
CANCommInstance       *chassis_comm;          //CANCommInstance*  格式的板件通信数据
Balance_Chassis_e      chassis_move_balance;	
Balance_Motor_t        balance_motor;        // 双腿平衡电机结构体
Balance_Chassis_Ctrl_Cmd_s chassis_ctrl;     // 双腿控制命令结构体
float Tp_roll = 0.0f;
float right_detet =0;
uint32_t CHASSR_TIME=1;	//1ms
float W_set_turn = 0.0f;  // Yaw控制输出力矩
extern Tumble_t tumble_ctrl; 
extern SpringCompParam_t spring_comp_r;
uint8_t right_flag=0;
float theta_diff = 0.0f;
/* ==================== 右腿任务私有变量 ==================== */
static DMMotorInstance *T1_r = NULL;              // 右前关节电机实例
static DMMotorInstance *T2_r = NULL;              // 右后关节电机实例
static DMMotorInstance *W_r = NULL;              // 右侧轮毂电机实例
static DMMotorInstance *Yaw = NULL;              // 右侧轮毂电机实例
static attitude_t *Chassis_IMU_data = NULL;       // IMU姿态数据指针       
static uint8_t initialized = 0;                   // 初始化完成标志
static uint8_t first_startup = 1;
static uint32_t startup_time = 0;
static uint8_t wheel_motor_enabled = 0;
static float yaw=0;
static float R_enable_time_ms = 0.0f;
static uint32_t R_last_dm_enable_ms = 0;
static float phi0_diff = 0.0f;
static float tumble_vel_cmd = 0.0f;
#define GROUND_DETECTION_RECOVER_DELAY_MS 8000.0f
    
// 腿长PID控制器
static PIDInstance LegR_Pid;  // 右腿的腿长PID
static PID_Init_Config_s legr_pid_config = {
    .Kp = LEGR_PID_KP,
    .Ki = LEGR_PID_KI,
    .Kd = LEGR_PID_KD,
    .MaxOut = LEGR_PID_MAX_OUT,
    .IntegralLimit = LEGR_PID_MAX_IOUT,
    .DeadBand = 0.0f,
    .Improve = PID_IMPROVE_NONE
};

// Roll补偿PID控制器
static PIDInstance Roll_Pid;
static PID_Init_Config_s roll_pid_config = {
	.Kp = 1000.0f,
	.Ki = 0.0f,
	.Kd = 1.5f,
	.MaxOut = 100.0f,
	.IntegralLimit = 0.0f,
	.DeadBand = 0.0f,
	.Improve = PID_IMPROVE_NONE
};

// Yaw角PD控制器
static PIDInstance Yaw_Pid;  // Yaw角跟随PD控制器
static PID_Init_Config_s yaw_pid_config = {
    .Kp = 35.0f,
    .Ki = 0.0f,  
    .Kd = 8.0f,
    .Improve = PID_IMPROVE_NONE
};

// theta_err PID控制器
static PIDInstance Theta_err_Pid_R;
static PID_Init_Config_s theta_err_pid_config_R = {
    .Kp = 55.0f,
    .Ki = 0.0f,
    .Kd = 18.0f,
    .MaxOut = 8.0f,
    .IntegralLimit = 0.0f,
    .DeadBand = 0.0f,
    .Improve = PID_IMPROVE_NONE
};

static PIDInstance Theta_gyro_err_Pid_R;
static PID_Init_Config_s theta_gyro_err_pid_config_R = {
    .Kp = 1.5f,
    .Ki = 0.0f,
    .Kd = 3.5f,
    .MaxOut = 12.0f,
    .IntegralLimit = 0.0f,
    .DeadBand = 0.0f,
    .Improve = PID_IMPROVE_NONE
};
uint8_t filter_flag =0;
//无敌
    // Q=diag([4000 1 1500 1 120000 1]);%theta d_theta x d_x phi d_phi
    // R=[30 0;0 1]; 
//常大参数
    // Q=diag([4000 1 1800 1 80000 1]);%theta d_theta x d_x phi d_phi
    // R=[25 0;0 1]; 
//还行
// float Poly_Coefficient[12][4] = {-532.8484,547.3993,-220.6518,1.1199,-16.2969,17.8375,-16.9297,0.5587,-181.9275,172.6682,-55.1657,-0.1814,-143.7706,134.4124,-43.3599,-1.0725,-960.7564,1123.5680,-482.0054,85.3869,-22.2296,33.7202,-18.7565,4.8039,-460.7698,753.8917,-430.8209,104.2380,101.6618,-80.2380,13.8087,3.6311,-671.3420,772.4486,-324.3346,54.9671,-883.5711,943.8000,-363.3189,56.0083,11238.2330,-10685.7868,3426.9153,-31.4472,493.2623,-491.5090,169.4607,-10.2453};
//封神榜 No.1
// float Poly_Coefficient[12][4] = {-538.7632,549.1031,-220.7242,-0.3168,-17.4586,18.4756,-17.5746,0.5168,-187.3847,176.8993,-56.0936,-0.9351,-138.5558,128.2517,-40.9689,-1.9719,-1187.6613,1342.1884,-554.0777,93.9697,-31.1425,42.3864,-21.6577,5.1878,-711.0041,950.7834,-468.9887,101.2367,75.6175,-56.8880,7.4622,3.6860,-724.9790,809.8837,-328.7588,53.5918,-923.9524,967.2747,-362.5067,53.8440,10356.1910,-9793.2425,3116.5141,7.0976,448.4388,-443.4865,151.4552,-7.7561};
// float Poly_Coefficient[12][4] = {-535.0884,548.1078,-220.6736,0.6349,-16.7256,18.0798,-17.1507,0.5446,-183.9228,174.2175,-55.5068,-0.4320,-142.1270,132.4192,-42.5702,-1.3740,-1039.0641,1198.9785,-506.8323,88.3359,-25.3647,36.7686,-19.7758,4.9379,-556.7241,830.0173,-445.9983,103.2434,91.9057,-71.4655,11.4123,3.6582,-692.2821,787.4446,-326.3929,54.5199,-900.0496,953.9768,-363.5100,55.2673,10929.6098,-10371.2719,3316.5992,-17.6102,477.4530,-474.4257,162.9941,-9.3422};
// float Poly_Coefficient[12][4] = {-532.8484,547.3993,-220.6518,1.1199,-16.2969,17.8375,-16.9297,0.5587,-181.9275,172.6682,-55.1657,-0.1814,-143.7706,134.4124,-43.3599,-1.0725,-960.7564,1123.5680,-482.0054,85.3869,-22.2296,33.7202,-18.7565,4.8039,-460.7698,753.8917,-430.8209,104.2380,101.6618,-80.2380,13.8087,3.6311,-671.3420,772.4486,-324.3346,54.9671,-883.5711,943.8000,-363.3189,56.0083,11238.2330,-10685.7868,3426.9153,-31.4472,493.2623,-491.5090,169.4607,-10.2453};
// float Poly_Coefficient[12][4] = {-543.4000,550.4006,-221.6460,-1.7660,-18.4130,18.8925,-18.3659,0.4793,-198.1380,186.3038,-58.7407,-1.7936,-134.6716,123.4949,-39.1690,-2.9578,-1403.0779,1550.4956,-623.3048,102.3820,-39.0387,50.0935,-24.2645,5.5530,-875.6848,1075.4435,-489.8970,98.1481,57.2151,-40.5238,3.0681,3.7107,-781.7402,856.3188,-339.5371,53.7812,-958.1550,989.4854,-363.9505,52.6059,9548.8369,-8992.9357,2845.2960,39.7040,409.8397,-403.0343,136.6775,-5.7745};
// float Poly_Coefficient[12][4] = {-544.0889,550.4680,-221.6838,-2.0669,-18.5728,18.9636,-18.4954,0.4705,-198.9013,186.8870,-58.8636,-1.9660,-133.2742,121.9717,-38.6184,-3.1542,-1445.3379,1591.3577,-636.8826,104.0207,-40.5669,51.5822,-24.7670,5.6221,-902.8072,1095.6221,-493.0143,97.5190,53.9370,-37.6284,2.3013,3.7051,-786.6831,858.9570,-339.2357,53.4604,-960.4563,989.6032,-362.8328,52.1888,9396.5604,-8843.0354,2794.9261,45.7010,402.4740,-395.3968,133.9202,-5.4089};
// float Poly_Coefficient[12][4] = {-556.2221,571.2403,-231.4704,0.2951,-16.3173,17.7992,-18.1357,0.5592,-200.6941,192.1846,-62.1458,-0.7589,-145.8169,136.8585,-44.5598,-1.9310,-865.8828,1037.6806,-457.3683,83.5267,-20.9877,32.5673,-18.4705,4.8606,-456.5889,727.5562,-408.9869,98.0352,93.2993,-73.8496,12.7819,3.3316,-593.1166,699.6402,-302.0473,52.7453,-816.9118,877.2047,-340.0692,52.8853,8458.6369,-8107.9232,2629.3438,-10.6666,378.2643,-380.3363,132.8314,-6.9943};
// float Poly_Coefficient[12][4] = {-557.8731,585.1958,-242.3153,2.4129,-12.5031,15.1616,-18.0047,0.6435,-213.3936,209.4631,-70.0191,0.2188,-158.2909,152.7851,-51.5633,-0.9987,-438.0423,625.9542,-323.3132,68.4376,-5.0201,16.7845,-13.1610,4.2440,61.0543,303.2256,-316.4291,101.2566,144.5009,-121.5648,26.7194,2.9453,-421.1761,578.3841,-287.9071,57.5105,-699.7042,799.3246,-334.5295,57.2711,7675.4204,-7531.0743,2521.1499,-49.2046,351.7538,-364.8986,132.6533,-8.5455};

//气胎
// float Poly_Coefficient[12][4] = {-541.8444,558.4634,-225.5166,2.0247,-16.0161,17.7902,-17.0433,0.6030,-193.0473,183.9215,-59.0666,0.2559,-154.0939,144.8204,-46.9908,-0.6219,-847.8955,1021.3373,-451.7374,82.5581,-16.6946,28.5461,-17.1457,4.6367,-315.5056,660.9014,-427.8767,111.4932,127.0136,-102.3880,19.4690,3.7347,-688.0872,811.8886,-350.1213,61.0146,-915.4591,992.5413,-389.2962,61.4668,12293.8447,-11734.5484,3783.0216,-60.1406,534.9570,-536.1248,186.1766,-12.3396};
// float Poly_Coefficient[12][4] = {-537.1232,550.8417,-221.6400,1.3545,-16.7383,18.1873,-17.0262,0.5732,-188.8006,178.7290,-56.8973,-0.0634,-149.0075,139.0504,-44.7347,-0.9356,-1021.2509,1185.7458,-504.3496,88.4188,-22.9767,34.6037,-19.1220,4.8620,-513.9123,828.1196,-468.4339,112.2110,110.3233,-86.9293,14.9469,3.9254,-762.7756,871.6143,-362.9671,60.9243,-976.5050,1040.4701,-399.0980,61.2204,12862.9121,-12200.4389,3898.9828,-41.0188,554.4280,-550.9160,189.1839,-11.9696};
// float Poly_Coefficient[12][4] = {-523.0558,536.6387,-216.7402,0.7558,-16.3897,17.7375,-16.8272,0.5414,-180.6572,170.8110,-54.2962,-0.3599,-140.6519,131.0432,-42.1520,-1.2236,-1048.8927,1202.4138,-504.9041,87.4945,-25.6610,36.9018,-19.7271,4.9005,-524.6985,794.4357,-432.4102,101.5466,93.3804,-73.1426,12.0676,3.6052,-710.0903,802.4805,-330.3883,54.8610,-898.6227,951.2922,-362.1358,55.0853,11277.8277,-10683.9990,3409.4677,-19.8332,501.7393,-497.2086,170.2140,-10.0460};
// float Poly_Coefficient[12][4] = {-559.6766,584.8458,-241.7722,2.1496,-13.1910,15.5721,-18.1910,0.6399,-216.7773,211.8239,-70.3967,0.0918,-159.8760,153.6198,-51.5950,-1.1234,-503.8689,691.1251,-345.5808,71.2967,-7.2873,19.0934,-13.9720,4.3565,-23.9258,375.7276,-334.2824,101.8670,138.9030,-116.2322,25.1523,3.0545,-466.9281,619.4016,-299.5601,58.4666,-735.7458,830.8674,-343.1021,57.9020,7964.4425,-7781.9334,2590.8413,-45.2874,362.8193,-374.1797,135.0450,-8.5301};
float Poly_Coefficient[12][4] = {-530.9977,543.6688,-217.8082,0.5977,-15.0201,16.5147,-16.8475,0.5375,-197.0066,190.0991,-62.0972,-0.4830,-139.8632,132.5022,-43.7198,-1.5472,-652.6285,816.4466,-375.5120,71.4688,-16.3564,26.9845,-15.9260,4.3122,-358.9776,657.6231,-399.3962,100.1811,105.1073,-84.5286,15.7926,3.2014,-543.7736,667.0078,-299.6029,54.3754,-747.9105,820.1413,-326.3859,52.4175,7794.9354,-7524.6995,2463.5646,-23.4175,363.5315,-367.5093,129.2375,-7.1699};

// /**
// * @description: 数据融合
// * @param {dt} 时间步长
// * @param {acc} 加速度测量值
// * @param {speed} 速度测量值
// * @param {vel_esti} 指向估计速度的指针
// * @return {*}
// */
float Rz = 1; 
float Qz = 0.03;
uint8_t W_cplt_flag=0;
float angle=0;
// void speed_est(const float dt, const float acc, const float speed, float *vel_esti)
// {
// 	static float _x = 0;
// 	static float _Pz = 1;

// 	float H = 1;
// 	float A = 1;
// 	float B = dt;
// 	float _x_est;
// 	float _p_est;
// 	float K;

// 	// 预测步骤

// 	_x_est = A * _x + B * acc;
// 	_p_est = _Pz + Qz; // A * _P * A_T + Q

// 	// 更新步骤
// 	K = _p_est / (_p_est + Rz);
// 	_x = _x_est + K * (speed - _x_est);
// 	_Pz = (1 - K) * _p_est;   

// 	*vel_esti = _x;
// }
// // 初始化静态变量
// //static float Rz = 1.0f;       // 初始测量噪声方差
// //static float Qz = 0.03f;      // 固定过程噪声方差
// //static float _x = 0.0f;       // 状态估计
// //static float _Pz = 1.0f;      // 估计协方差

// //void speed_est(const float dt, const float acc, const float speed, float *vel_esti) {
// //    const float ALPHA = 0.03f; // 平滑系数 (5%)
// //    const float MAX_ERR = 3.5f; // 最大允许残差

// //    float H = 1.0f, A = 1.0f, B = dt;
// //    float _x_est, _p_est, K;

// //    // 预测步骤
// //    _x_est = A * _x + B * acc;
// //    _p_est = _Pz + Qz;

// //    // 动态更新Rz
// //    float residual = speed - _x_est;
// //    residual = fmaxf(fminf(residual, MAX_ERR), -MAX_ERR); // ???
// //    Rz = (1 - ALPHA) * Rz + ALPHA * residual * residual;

// //    // 更新步骤
// //    K = _p_est / (_p_est + Rz);
// //    _x = _x_est + K * residual;
// //    _Pz = (1 - K) * _p_est;

// //    *vel_esti = _x;
// //}

// float kf_fusion;
// float forward_acc;

// float data_fusion(void)//速度融合调用
// 	{
// 	float fusion;
// 	forward_acc = -INS.MotionAccel_b[0];
// 	float speed;
// 	//速度方向 修改
// 	//	chassis->v = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;	

// 	speed = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;
	
// 	speed_est(0.001,forward_acc,speed,&fusion);

// 	return fusion;
// }

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}

static void ChassisR_Init(void)
{
    while (!chassis_init_done) { osDelay(1);}
    T1_r = Chassis_GetMotor_T1R();
    T2_r = Chassis_GetMotor_T2R();
    W_r  = Chassis_GetMotor_WR();
    Yaw  = Chassis_GetMotor_Yaw();
    Chassis_IMU_data = Chassis_GetIMUData();
    chassis_comm = Chassis_CAN_COMM();
    DWT_Delay(0.5);

    chassis_move_balance.balance_motor = &balance_motor;
    chassis_move_balance.chassis_move_balance = &chassis_ctrl;

    balance_motor.joint_motor[0] = T1_r;
    balance_motor.joint_motor[1] = T2_r;
    balance_motor.wheel_motor[0] = W_r;

//原始参数初始化
    chassis_ctrl.leg_set = 0.19f;
    chassis_ctrl.leg_target = 0.35f;
    chassis_ctrl.v_set = 0.0f;
    chassis_ctrl.x_set = 0.0f;
    chassis_ctrl.turn_set = 0.0f;
    chassis_ctrl.roll_set = 0.0f;
    chassis_ctrl.total_yaw = 0.0f;
    chassis_ctrl.start_flag = 0;
    chassis_ctrl.W_Strat = 0;

    PIDInit(&LegR_Pid, &legr_pid_config);
    PIDInit(&Yaw_Pid, &yaw_pid_config);
    PIDInit(&Theta_err_Pid_R, &theta_err_pid_config_R);
    PIDInit(&Theta_gyro_err_Pid_R, &theta_gyro_err_pid_config_R);
    PIDInit(&Roll_Pid, &roll_pid_config);
    TumbleRecover_Init();
    R_enable_time_ms = DWT_GetTimeline_ms();
 
    initialized = 1;
}

void ChassisR_task(void)
{
    if (!initialized) {
        ChassisR_Init();
    }

    // 主控制循环
    while(1)
    {      
// 0. 从CAN接收最新的控制命令
        chassis_cmd = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chassis_comm);
        
        // 检查INS是否初始化完成
        if (Chassis_IMU_data == NULL || Chassis_IMU_data->init == 0) {
            osDelay(1);
            continue;
        }

// 1. 检查底盘模式
switch (chassis_cmd.chassis_mode)
    {
case CHASSIS_ZERO_FORCE:
// 倒地自起模式_遥控器左侧拨至上侧
 right.phi1 = pi - T1_r->measure.position;
 right.phi4 = pi/2 - T2_r->measure.position;
 VMC_calc_right(&right, Chassis_IMU_data, ((float)CHASSR_TIME) * 1.5f / 1000.0f);
 right.leg_tumble=tumble_detect(Chassis_IMU_data,&right);
 chassis_ctrl.phi0_err = right.phi0 - left.phi0;
 theta_diff = right.theta - left.theta;
 if (chassis_cmd.tumble_detect == 1){
         // 更新电机角度反馈
         right.F0 = -80.0f;
         right.Tp = 0.0f;
        VMC_calc_splitter(&right);
        mySaturate(&right.torque_set[0], -25.0f, 25.0f);
        mySaturate(&right.torque_set[1], -25.0f, 25.0f);
        tumble_vel_cmd =      
          ((right.theta >  1.0f) && (right.theta < 1.26f)) ?  0.0f 
        // : (((Chassis_IMU_data->Pitch > 2.8f) && (Chassis_IMU_data->Pitch < 3.1f)) ? ((theta_diff > 0.16f) ? 3.0f : 0.0f)
        : ((Chassis_IMU_data->Pitch >= 0.5f) && (Chassis_IMU_data->Pitch < 1.9f)) ? -3.0f 
        : 3.0f;//phi0差0.15f时停止转动先统一摆角       
        DMMotorSend_POS(T1_r, 0, tumble_vel_cmd, 0, 2.8, right.torque_set[0]);
        osDelay(1);
        DMMotorSend_POS(T2_r, 0, tumble_vel_cmd, 0, 2.8, right.torque_set[1]);
        osDelay(1);
        DMMotorSendPair_W(W_r,  0);
        osDelay(1);
        break;
        }
// 制动模式_遥控器右侧开关拨至底下时强制制动
        right.theta_accum = right.theta;
        Chassis_ResetMoveState(&chassis_move_balance,0.0f);
        DMMotorSendPair(T1_r, 0);
        DMMotorSendPair(T2_r, 0);
        DMMotorSendPair_W(W_r,0);
        chassis_move_balance.chassis_move_balance->W_Strat = 0;
        Chassis_ResetWheel(&first_startup, &startup_time, &wheel_motor_enabled);
            osDelay(1);
                continue;

case CHASSIS_RUN:
 // 运行模式，遥控器右侧拨至中间

        if ((DWT_GetTimeline_ms() - R_last_dm_enable_ms) >= 100.0f) {
            Chassis_DMMotorEnable(T1_r);
            Chassis_DMMotorEnable(T2_r);
            Chassis_DMMotorEnable(W_r);
            R_last_dm_enable_ms = DWT_GetTimeline_ms();
        }

// 2. 更新反馈数据
         chassisR_feedback_update(&chassis_move_balance, T1_r, T2_r, &right, Chassis_IMU_data);
 
 //跨越模式：phi0>2.0且L0>3.2时接管控制，theta>1.2时自动退出
// uint8_t fold_ran_r = 0;  // 检测跨越函数是否执行过
//           while (LegFold_Override(&right, LQR_K_R,chassis_move_balance.chassis_move_balance->leg_tp_r,&LegR_Pid, SPRING_SIDE_RIGHT)) 
//         {
//              fold_ran_r = 1;             
//              // 跨越过程中清零位移速度相关的观测值和目标值
//              Chassis_ResetMoveState(&chassis_move_balance, 0.0f);     
//              DMMotorSendPair(T1_r, right.torque_set[0]);
//              osDelay(1);
//              DMMotorSendPair(T2_r, right.torque_set[1]);
//              osDelay(1);
//             //  DMMotorSendPair(T1_r, 0.0f);
//             //  DMMotorSendPair(T2_r, 0.0f);
//              DMMotorSendPair_W(W_r, 0.0f);
//              osDelay(1);
//              chassisR_feedback_update(&chassis_move_balance, T1_r, T2_r, &right, Chassis_IMU_data);
//              VMC_calc_right(&right, Chassis_IMU_data, ((float)CHASSR_TIME)*1.5f/1000.0f);
//              for (int i = 0; i < 12; i++) {
//                  LQR_K_R[i] = LQR_K_calc(&Poly_Coefficient[i][0], right.L0);
//              }
//          }
// // 跨越函数刚退出，重置轮毂延时使重进 LQR 时再延时 550ms
//          if (fold_ran_r) {
//              Chassis_ResetWheel(&first_startup, &startup_time, &wheel_motor_enabled);
//             chassis_move_balance.chassis_move_balance->leg_set = 0.185f;
//             chassis_move_balance.chassis_move_balance->leg_target = 0.345f;  // 退出跨越后收腿
//          }

// 3. 执行控制环路计算
         chassisR_control_loop(&chassis_move_balance, &right, Chassis_IMU_data, LQR_K_R);

uint8_t wheel_enabled =
Chassis_WheelMotorStartupDelay(&first_startup, &startup_time, &wheel_motor_enabled, 550.0f);
//测试
        //   right.torque_set[0]=-0.0f;
        //   right.torque_set[1]=0.0f;
        //   right.wheel_set = 0.0f;
        
// 4.电机输出 - DM电机发送
        // 发送关节电机控制帧
        DMMotorSendPair(T1_r, right.torque_set[0]);
        osDelay(1);
        DMMotorSendPair(T2_r, right.torque_set[1]);
        osDelay(1);
        
        // 只有在延时结束后才发送轮毂电机控制帧
        if (wheel_enabled) {
            DMMotorSendPair_W(W_r, right.wheel_set);
        } else {
            DMMotorSendPair_W(W_r, 0.0f);  // 延时期间发送0力矩
        }
        osDelay(1);
                break;
default:
        osDelay(1);
        break;
      }
    }
}

 //底盘数据反馈更新
void chassisR_feedback_update(Balance_Chassis_e *chassis,DMMotorInstance *T1,DMMotorInstance *T2,vmc_leg_t *vmc,attitude_t *ins)
{

    vmc->phi1=pi-T1->measure.position;
	vmc->phi4=pi/2-T2->measure.position;

//角度项
	chassis->chassis_move_balance->myPithR=0.0f-ins->Pitch;
	chassis->chassis_move_balance->myPithGyroR=0.0f-ins->Gyro[0];
	chassis->chassis_move_balance->roll=ins->Roll;
    
    //刹车补偿项
    // chassis->chassis_move_balance->theta_compensate = chassis->chassis_move_balance->v_set-chassis_move_balance.chassis_move_balance->v_filter;
    // chassis->chassis_move_balance->theta_compensate =  mean_filter_five(chassis->chassis_move_balance->theta_compensate )* 2.0f;
   
    //  chassis->chassis_move_balance->theta_compensate = 0.0f;
    //  mySaturate(&chassis->chassis_move_balance->theta_compensate, -0.02f, 0.02f);

 // total_yaw发生变化,重设状态值
     Chassis_UpdateTotalYaw(chassis, chassis_cmd.yaw, chassis_cmd.rotate_start);
     
     chassis->chassis_move_balance->relative_yaw=ecd_to_angle_change(Yaw->measure.position,0,8191,0.000766990394f);
     chassis->chassis_move_balance->theta_err=vmc->theta-left.theta;   

//位移速度项
    chassis->chassis_move_balance->x_set = chassis->chassis_move_balance->x_set+chassis->chassis_move_balance->v_set*(float)CHASSR_TIME*2.0f/1000.0f;
    chassis->chassis_move_balance->v_target = chassis_cmd.vy*1.0f;
    slope_following(&chassis->chassis_move_balance->v_target,&chassis->chassis_move_balance->v_set , 0.008f);

//腿长项
    chassis->chassis_move_balance->last_leg_set = chassis->chassis_move_balance->leg_set;
    chassis->chassis_move_balance->leg_set =
    //  0.16f;
    chassis->chassis_move_balance->leg_set + 0.5*chassis_cmd.leg_set;
    mySaturate(&chassis->chassis_move_balance->leg_set,0.192f,0.33f);	

//机身Yaw角PD控制
if (chassis_cmd.rotate_start == 0){
     yaw = Chassis_WrapAngleToPi(yaw_angle_fdb);
     W_set_turn =
                0.0f
                // -Yaw_Pid.Kp*(Chassis_IMU_data->YawTotalAngle-chassis->chassis_move_balance->total_yaw) //底盘不跟随云台        
                +Yaw_Pid.Kp * yaw
                -Yaw_Pid.Kd * Chassis_IMU_data->Gyro[2];//因使用角速度计不使用PID_calc函数            
    } 
//小陀螺_左侧拨至下侧
else if(chassis_cmd.rotate_start == 1){
    W_set_turn = 35.5f * 0.65f - 2.15f * Chassis_IMU_data->Gyro[2];//类开环控制，我没太懂为啥这样写相当于Kp*期望Wz-Kd*机体的gyro_wz
}

    if (chassis_cmd.rotate_start == 1) 
        {
            Chassis_ResetMoveState(chassis, 0.0f);
        }
	mySaturate(&W_set_turn, -10.0f, 10.0f);
    
// Roll补偿PID
    Tp_roll = Roll_Pid.Kp * (Chassis_IMU_data->Roll - 0) - Roll_Pid.Kd * Chassis_IMU_data->Gyro[1];
//防劈叉PID
    
    chassis->chassis_move_balance->leg_tp_gyro_r = 
    PID_calc(&Theta_err_Pid_R,chassis->chassis_move_balance->theta_err, 0.0f);
    // Theta_err_Pid_R.Kp*(chassis->chassis_move_balance->theta_err-0.0f);
    chassis->chassis_move_balance->leg_tp_r = 
    // Theta_gyro_err_Pid_R.Kp * (chassis->chassis_move_balance->leg_tp_gyro_r - chassis_move_balance.chassis_move_balance->leg_gyro_r);  
    PID_calc(&Theta_gyro_err_Pid_R,chassis->chassis_move_balance->leg_tp_gyro_r, chassis_move_balance.chassis_move_balance->leg_gyro_r);
}

extern uint8_t left_flag;
int jump_right_flag = 0;
void chassisR_control_loop(Balance_Chassis_e *chassis,vmc_leg_t *vmcr,attitude_t *ins,float *LQR_K)
{
	
	VMC_calc_right(vmcr,ins,((float)CHASSR_TIME)*1.5f/1000.0f);
     
   for(int i=0;i<12;i++)
   {
	LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0);	
   }
	  
	  vmcr->wheel_set = (
                          LQR_K[0]*(vmcr->theta-0.0f)
                         +LQR_K[1]*(vmcr->d_theta-0.0f)       
                         +LQR_K[2]*(chassis->chassis_move_balance->x_filter-(chassis->chassis_move_balance->x_set))
				         +LQR_K[3]*(chassis->chassis_move_balance->v_filter-(chassis->chassis_move_balance->v_set))
                         +LQR_K[4]*(chassis->chassis_move_balance->myPithR-0.0f) 
						 +LQR_K[5]*(chassis->chassis_move_balance->myPithGyroR-0.0f)
                         -W_set_turn //转向补偿力矩 
                        // chassis->chassis_move_balance->v_set*15
                        );		
//右边髋关节输出力矩
//关节电机Tp计算  
	  vmcr->Tp =(
                  LQR_K[6]*(vmcr->theta-0.0f)
                 +LQR_K[7]*(vmcr->d_theta-0.0f)
                 +LQR_K[8]*(chassis->chassis_move_balance->x_filter-(chassis->chassis_move_balance->x_set))
				 +LQR_K[9]*(chassis->chassis_move_balance->v_filter-(chassis->chassis_move_balance->v_set))
				 +LQR_K[10]*(chassis->chassis_move_balance->myPithR-0.0f)
				 +LQR_K[11]*(chassis->chassis_move_balance->myPithGyroR-0.0f)
                 +chassis->chassis_move_balance->leg_tp_r//防劈叉
                );
    // mySaturate(&vmcr->Tp, -2.0f, 2.0f);  
    // vmcr->Tp = 0.0f;
    // vmcr->F0 = 0.0f;
// 对轮毂电机输出限幅
	// mySaturate(&vmcr->wheel_set,-5.5f,5.5f);	
// 腿长PID控制：前馈+PID
    slope_following(&chassis->chassis_move_balance->leg_set,&chassis->chassis_move_balance->leg_target, 0.01f);
    vmcr->F0 =0.0f-
    ((50.2f/arm_cos_f32(vmcr->theta))
    +PID_calc(&LegR_Pid, vmcr->L0, chassis->chassis_move_balance->leg_target));
    vmcr->F0 = vmcr->F0 + Tp_roll;  // 加上roll补偿
// 气弹簧补偿（等效到腿轴向力F0）	
    vmcr->Fv=Fv_Gas_spring(vmcr,&spring_comp_r);
    vmcr->F0_test = 0.0f-vmcr->F0;
    vmcr->F0 = vmcr->F0 + vmcr->Fv;   

 // 右腿离地检测
	right_flag = Ground_detectionR(vmcr, ins);
	if (fabsf(chassis->chassis_move_balance->leg_set - chassis->chassis_move_balance->last_leg_set) > 0.00000002f)
	{
		right_flag = 0;
	}
	if ((DWT_GetTimeline_ms() - R_enable_time_ms) < GROUND_DETECTION_RECOVER_DELAY_MS)//延迟8s再启动
	{
		right_flag = 0;
	}
		if(right_flag == 1 && left_flag == 1)
		{
// 		// 当两腿同时离地并且遥控器没有在控制腿的伸缩时,才认为离地
// 		// 排除跳跃的压缩阶段、上升阶段、跳跃的缩腿阶段
			vmcr->wheel_set = 0.0f;
			vmcr->Tp = LQR_K[6] * (vmcr->theta_accum - chassis->chassis_move_balance->theta_compensate) 
                     + LQR_K[7] * (vmcr->d_theta - 0.0f);
			chassis->chassis_move_balance->x_filter = 0.0f;
			chassis->chassis_move_balance->x_set = chassis->chassis_move_balance->x_filter;
            // chassis->chassis_move_balance->leg_target += 0.0005;
		}
//功率控制
    // chassis_power_control(chassis);

	mySaturate(&vmcr->F0,-300.0f,300.0f);//限幅
	VMC_calc_splitter(vmcr);//计算期望的关节输出力矩 

}

