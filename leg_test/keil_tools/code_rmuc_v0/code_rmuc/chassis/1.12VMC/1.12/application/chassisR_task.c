/****************************RIGHT*******************************/
/****************************CAN1*******************************/
//C板放置：R标横着 R的上半部分对应没有贴纸的，下半部分（竖和娜）娜对应唯一一个贴标签6的电机
//C板CAN线序左L右H
#include <math.h>
#include <stdio.h>
#include "CANdata_analysis.h"
#include "chassisR_task.h"
#include "chassisL_task.h"
#include "can.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "chassis_power_control.h"
#include "shoot.h"
#define YAW_MOUSE_SEN   0.00005f//0.00005f
#define PITCH_MOUSE_SEN -0.00015f//0.00015f
////reducation of 3508 motor
////m3508电机的减速比
//#define M3508_MOTOR_REDUCATION 15.764705882f
////m3508 rpm change to chassis speed   576.096  3.14 *07425 = 0.233145
////m3508转子转速(rpm)转化成底盘速度(m/s)的比例，c=pi*r/(30*k)，k为电机减速比
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.0004998609952f

////m3508 rpm change to motor angular velocity
////m3508转子转速(rpm)转换为输出轴角速度(rad/s)的比例
//#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.00664267f

////m3508 current change to motor torque
////m3508转矩电流(-16384~16384)转为成电机输出转矩(N.m)的比例
////c=20/16384*0.3，   
//#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f

//rocker value (max 660) change to vertial speed (m/s) 
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.0100f
//右边
float LQR_K_R[12]={-11.0224 , -2.664 ,-11.6436,-20.2754,30.2361 ,3.3863,10.2554,0.9963,1.0764,3.2532,40.8928, 3.5782};                                                          
float Poly_Coefficient[12][4] = {-1181.43687, 1000.47107, -332.98519, 12.18070, -387.15753, 306.79332, -91.24644, 5.74351, -145.69782, 109.51968, -26.66966, -1.76855, -355.88018, 269.45691, -67.16654, -1.20834, -1310.50504, 992.53930, -248.77565, 23.51614, -48.25353, 36.98562, -9.59252, 1.19229, -898.79176, 673.13201, -159.36841, 14.32566, -105.95203, 80.20003, -18.76384, 1.67748, -407.24777, 307.01849, -75.75709, 6.56699, -642.72938, 484.70280, -119.54222, 10.38984, 517.55704, -386.58536, 94.55970, 20.10669, 37.77101, -28.39868, 7.01833, 0.30775};

vmc_leg_t right;//右腿

extern INS_t INS;
extern vmc_leg_t left;

chassis_t chassis_move_balance;
extern c_fbpara_t  C_data;
float jump_time_r;
extern float jump_time_l;
												
pid_type_def LegR_Pid;//右腿的腿长pd
pid_type_def Tp_Pid;//防劈叉补偿pd
pid_type_def Turn_Pid;//转向pd
pid_type_def Roll_Pid;//横滚角补偿pd
pid_type_def Wheel_Pid; //3508PID
pid_type_def Wz_Pid;
extern pid_type_def buffer_pid;
extern shoot_control_t shoot_control;          

/**
* @description: 数据融合
* @param {dt} 时间步长
* @param {acc} 加速度测量值
* @param {speed} 速度测量值
* @param {vel_esti} 指向估计速度的指针
* @return {*}
*/
float Rz = 1; 
float Qz = 0.03;

void speed_est(const float dt, const float acc, const float speed, float *vel_esti)
{
	static float _x = 0;
	static float _Pz = 1;

	// float H = 1;
	float A = 1;
	float B = dt;
	float _x_est;
	float _p_est;
	float K;

	// 预测步骤
	_x_est = A * _x + B * acc;
	_p_est = _Pz + Qz; // A * _P * A_T + Q

	// 更新步骤
	K = _p_est / (_p_est + Rz);
	_x = _x_est + K * (speed - _x_est);
	_Pz = (1 - K) * _p_est;   

	*vel_esti = _x;
}

extern float LQR_K_L[12];
float kf_fusion;
float forward_acc;
float data_fusion(void)//速度融合调用
	{
	float fusion;
	forward_acc = -INS.MotionAccel_b[0];
	float speed;
	//速度方向 修改
	//	chassis->v = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;	
	speed = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;	
	speed_est(0.001,forward_acc,speed,&fusion);
	return fusion;
}

uint32_t CHASSR_TIME=1;	//1ms
float yaw_sen;
float pitch_sen;
float mode_rc;
float fire_mode;

void ChassisR_task(void)
{
	//保持平衡
	chassis_move_balance.leg_set=0.20; 
	chassis_move_balance.turn_set = 0;
	chassis_move_balance.x_set = 0;
	chassis_move_balance.v_set = 0;
	chassis_move_balance.recover_flag = 0;
	chassis_move_balance.roll_set = 0.0005;
	shoot_control.shoot_send_flag = 0;
	// chassis_move_balance.target_x = 0;
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}
	//电机初始化
	//包括了右边关节电机的id,mode初始化，以及电机的使能；
	  ChassisR_init(&chassis_move_balance,&right,&LegR_Pid,&Wheel_Pid);
	//   Pensation_init(&Roll_Pid,&Tp_Pid,&Turn_Pid,&Wz_Pid);//pid初始化	
	//   shoot_init();	
	while(1)
	{	
		 //等待电机使能
		chassis_move_balance.DUBS_ON=toe_is_error(DBUS_TOE);//0:正常 1:异常

		if(chassis_move_balance.chassis_RC->rc.s[1] == 1){
	
		fire_mode = 1;		
		}
		if(chassis_move_balance.chassis_RC->rc.s[1] == 3){
		
		fire_mode = 0;		
		}				
        //板间通信
	    // c_transmit_date(yaw_sen, pitch_sen,mode_rc,shoot_control.shoot_send_flag, 11);
		// osDelay(1);				
		float dt = (float)CHASSR_TIME/1000.0f;//1/1000	

		if( chassis_move_balance.chassis_RC->rc.s[0] == 3)
		{
		   chassis_move_balance.start_flag=1;
			mode_rc  = 1; 
		}	
		if( chassis_move_balance.chassis_RC->rc.s[0] == 2)//右拨杆最下
		{			
		   chassis_move_balance.start_flag=0;
			mode_rc  = 0;   //无力模式
		}
		if( chassis_move_balance.chassis_RC->rc.s[0] == 1)
		{
			chassis_move_balance.w_flag  = 1;//底盘不跟随云台
		}else{
		chassis_move_balance.w_flag  = 0;//底盘跟随云台
		}
		//倒地检测
	    chassis_move_balance.recover_flag = recover_detect(&chassis_move_balance);	

		//CHASSR_TIME=1;	
	   if(chassis_move_balance.start_flag==1)
       {	
	   if(chassis_move_balance.chassis_RC->key.v & CHASSIS_FRONT_KEY){
	
	        chassis_move_balance.target_v = 2;	
	    }else if(chassis_move_balance.chassis_RC->key.v & CHASSIS_BACK_KEY){
			
			chassis_move_balance.target_v = -2;
	    } 
		chassis_move_balance.target_v=
		                               0;
 		                            // ((float)chassis_move_balance.chassis_RC->rc.ch[1])*(0.0027f);
		slope_following(&chassis_move_balance.target_v,&chassis_move_balance.v_set,0.0035f);	//斜坡函数
//x位控
        chassis_move_balance.x_set = 
		                            0.0f;
		                            // chassis_move_balance.x_set+chassis_move_balance.v_set*(float)CHASSR_TIME*5.82f/1000.0f;	

		yaw_sen = chassis_move_balance.chassis_RC->rc.ch[2]*(0.00003f) - chassis_move_balance.chassis_RC->mouse.x * YAW_MOUSE_SEN;	
		pitch_sen =((float)chassis_move_balance.chassis_RC->rc.ch[3])*(0.00003f) + chassis_move_balance.chassis_RC->mouse.y * PITCH_MOUSE_SEN;
	
	if((shoot_control.shoot_rc->key.v & CHASSIS_LEG_KEY) && 
        !(shoot_control.last_key & CHASSIS_LEG_KEY)){
	   	chassis_move_balance.leg_set = chassis_move_balance.leg_set + 0.1f; 
	}
   	chassis_move_balance.leg_set = chassis_move_balance.leg_set+(((float)chassis_move_balance.chassis_RC->rc.ch[0])*(0.0000037f)); 

		mySaturate(&chassis_move_balance.leg_set,0.200f,0.35f);

		if(fabsf(chassis_move_balance.last_leg_set-chassis_move_balance.leg_set)>0.0007f)
	{
				//遥控器控制腿长在变化
				right.leg_flag=1;	//为1标志着遥控器在控制腿长伸缩，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判为离地了
				left.leg_flag=1;	 			
}	
	chassis_move_balance.last_leg_set=chassis_move_balance.leg_set;
//小陀螺
	// if(chassis_move_balance.w_flag == 1)  
	// {
	// 	chassis_move_balance.Wz_set= 1;
	// 	chassis_move_balance.v_set = 0;
	// 	chassis_move_balance.x_set = 0;
	// 	// chassis_move_balance.target_x=0;
	// }else {
	// chassis_move_balance.w_flag=0;	
	// chassis_move_balance.Wz_set=0;
	// }		
}	
//更新数据
	    chassisR_feedback_update(&chassis_move_balance,&right,&INS);
//控制计算
	    chassisR_control_loop(&chassis_move_balance,&right,&INS,LQR_K_L,&LegR_Pid);	

//足电机控制
		if(chassis_move_balance.start_flag==1)	
		{
			mit_ctrl(&hcan2,0x08, 0.0f, 0.0f,0.0f, 0.8f,right.torque_set[1]);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hcan2,0x06, 0.0f, 0.0f,0.0f, 0.8f,right.torque_set[0]);//right.torque_set[0]
			osDelay(CHASSR_TIME);	
			//顺时针为正
			chassis_move_balance.wheel_motor[0].given_current = (chassis_move_balance.wheel_motor[0].wheel_T/0.000396211f);
			CAN_cmd_chassis(-chassis_move_balance.wheel_motor[0].given_current);
            osDelay(CHASSR_TIME);
		}
		 else if(chassis_move_balance.start_flag==0)
		{
			mit_ctrl(&hcan2,0x08, 0.0f, 0.0f,0.0f, 0.8f,0.0f);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hcan2,0x06, 0.0f, 0.0f,0.0f, 0.8f,0.0f);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			CAN_cmd_chassis(0);
			osDelay(CHASSR_TIME);
		}
  }
}

//底盘初始化
void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,pid_type_def *legr,pid_type_def *wheel)
{	
	chassis->chassis_RC = get_remote_control_point();//获取遥控指针

   const static float legr_pid[3] = 
                                      {0,0,0};
                                //    {200.0f,0.0f,500.0f};//{LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};

   const static fp32 power_pid[3] = {POWER_PID_KP, POWER_PID_KI, POWER_PID_KD};
  
   const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	
    //右边关节电机的初始化
	joint_motor_init(&chassis->joint_motor[0],1,MIT_MODE);//发送id为1
	joint_motor_init(&chassis->joint_motor[1],2,MIT_MODE);//发送id为2	
	VMC_init(vmc);//给杆长赋值	
	//腿长pid初始化
	PID_init(legr, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hcan2,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hcan2,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	  osDelay(1);
	}	
	for(int m=0;m<2;m++)
    {
      
       PID_init(&chassis->motor_speed_pid[m], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }	
	   PID_init(&chassis->buffer_pid, PID_POSITION,power_pid,POWER_PID_MAX_OUT, POWER_PID_MAX_IOUT);
}

float roll_pid[3] = 
                 {0,0,0};   //测试使用
                    // {ROLL_PID_KP, ROLL_PID_KI,ROLL_PID_KD};
float tp_pid[3]   = 
                 {0,0,0};
                    // {TP_PID_KP, TP_PID_KI, TP_PID_KD};
float turn_pid[3] = 
                 {0,0,0};
                    // {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
float wz_pid[3]   = 
                 {0,0,0};
                    // {WZ_PID_KP, WZ_PID_KI, WZ_PID_KD};
//PID初始化
void Pensation_init(pid_type_def *roll,pid_type_def *Tp,pid_type_def *turn,pid_type_def *wz)
{//补偿pid初始化：横滚角补偿、防劈叉补偿、偏航角补偿
//	横滚角补偿
	PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
//  防劈叉补偿
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
//  偏航角补偿
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
//  Wz侧补偿	
	PID_init(wz, PID_POSITION, wz_pid, WZ_PID_MAX_OUT, WZ_PID_MAX_IOUT);
}

//底盘数据反馈更新
void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
	
	//get remote control point
    //调用获取遥控器指针，得到当前各个通道的值 
    chassis->chassis_RC = get_remote_control_point();
	
    vmc->phi1=pi/2.0f+chassis->joint_motor[0].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[1].para.pos;
		
	chassis->myPithR=ins->Pitch;
	chassis->myPithGyroR=ins->Gyro[1];
	
	chassis->relative_angle = chassis->yaw_motor_angle;
	
	chassis->total_yaw=ins->YawTotalAngle;
	chassis->roll=ins->Roll;
	chassis->theta_err=0.0f-(vmc->theta+left.theta);
	
	chassis->wheel_motor[0].vel = chassis->wheel_motor[0].speed_rpm * CHASSIS_MOTOR_RPM_TO_OMG_SEN;  
	chassis->wheel_motor[0].speed = 0.0004998609952f * chassis->wheel_motor[0].speed_rpm;  

	chassis->v = data_fusion();
	chassis->x = chassis->x + chassis->v*((float)CHASSR_TIME/1000.0f);

	//倒地检测	
	//根据pitch角度判断倒地自起是否完成
	if(ins->Pitch<(3.1415926f/15.5f)&&ins->Pitch>(-3.1415926f/15.5f))
	{
		chassis->recover_flag=0;
	}	
}

	
uint8_t right_flag=0;
extern uint8_t left_flag;
int jump_right_flag = 0;

void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,pid_type_def *leg)
{
	VMC_calc_right(vmcr,ins,((float)CHASSR_TIME)*3.0f/1000.0f);
     
    for(int i=0;i<12;i++)
   {
	LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0);	
   }
   if(chassis->w_flag==1){	
	   chassis->turn_T=Turn_Pid.Kp*(chassis->Wz_set-0)-Turn_Pid.Kd*ins->Gyro[2];
	}
	else{
		chassis->turn_T=Turn_Pid.Kp*(chassis->relative_angle-0)-Turn_Pid.Kd*ins->Gyro[2];
      //chassis->turn_T = - PID_calc(&Turn_Pid,chassis->relative_angle,0);
	}
	//Roll轴补偿
	chassis->roll_f0=Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[0];
	//Roll轴补偿的pid限幅
	mySaturate(&chassis->roll_f0,-Roll_Pid.max_out,Roll_Pid.max_out);
	//防劈叉pid计算
	chassis->leg_tp=PID_calc(&Tp_Pid, chassis->theta_err,0.00f);

	//轮毂电机
     // chassis->wheel_motor[0].wheel_T =   0;
 	chassis->wheel_motor[0].wheel_T =   (LQR_K[0]*(vmcr->theta-0.0f)
									    +LQR_K[1]*(vmcr->d_theta-0.0f)
									    +LQR_K[2]*(chassis->x_filter-(X_err+chassis->x_set))
									    +LQR_K[3]*(chassis->v_filter2-(chassis->v_set))
										+LQR_K[4]*(chassis->myPithR-0.0f) 
									    +LQR_K[5]*(chassis->myPithGyroR-0.0f));	
	
	//右边髋关节输出力矩				
	  vmcr->Tp = (LQR_K[6]*(vmcr->theta-0.0f)	
				 +LQR_K[7]*(vmcr->d_theta-0.0f)
				 +LQR_K[8]*(chassis->x_filter-(X_err+chassis->x_set))
				 +LQR_K[9]*(chassis->v_filter2-chassis->v_set)
				 +LQR_K[10]*(chassis->myPithR-0.0f)
				 +LQR_K[11]*(chassis->myPithGyroR-0.0f));

	    //    chassis->wheel_motor[0].wheel_T = 0;
		chassis->wheel_motor[0].wheel_T=chassis->wheel_motor[0].wheel_T;
		//  - chassis->turn_T;
	
//对轮毂电机输出限幅
	mySaturate(&chassis->wheel_motor[0].wheel_T,-4.2f,4.2f);	
	// vmcr->Tp=vmcr->Tp+chassis->leg_tp;//髋关节输出力矩 LQR得到的+防劈叉补偿
	// vmcr->Tp=0.0f;//测试使用
	vmcr->F0=17.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,chassis->leg_set)-chassis->roll_f0;//前馈+pd	

	if(chassis_move_balance.chassis_RC->rc.s[0]==2)//右拨杆拨至最下边
	{
		chassis_move_balance.x_set=0.0f;
	}
//跳跃逻辑
//  	if(chassis->chassis_RC->rc.s[1] ==1)//左上拨杆拨至最上边
//  	{
//  		if(chassis->chassis_RC->rc.ch[4] >500){  //左拨轮往下拨
		
//  		    chassis->help_jump_flag =1;		
//  		}
// 	//压缩阶段	
//  		  if(chassis->jump_flag_r==0 && chassis->help_jump_flag ==1){
			
//  		      chassis->leg_set = 0.130;

//  		       if(vmcr->L0<0.17f)
//  		     {
//  		        jump_time_r++;  
//  		      }
//  		     if(jump_time_r>=10&&jump_time_l>=10)
//  		     {  
//  			   jump_time_r=0;
//  			   jump_time_l=0;
//  			   chassis->jump_flag_r=1;//压缩完毕进入上升加速阶段
// 			   chassis->jump_flag_l=1;//压缩完毕进入上升加速阶段
// 		     }			 		 
//  		   }

//  //上升加速阶段			
//  		else if(chassis->jump_flag_r==1&& chassis->help_jump_flag ==1)
//  		{		
//  			chassis->leg_set = 0.30;

//  			 if(vmcr->L0>0.22f)
//  			 {
//  				jump_time_r++;
//  			 }
//  			 if(jump_time_r>=10&&jump_time_l>=10)
//  			 {  
//  				 jump_time_r=0;
//  				 jump_time_l=0;
//  				 chassis->jump_flag_l=2;//上升完毕进入缩腿阶段
//  				 chassis->jump_flag_r=2;
//  			 }	 

//  			}
// //缩腿阶段		
//  	 else if(chassis->jump_flag_r==2&& chassis->help_jump_flag ==1)
//  		{
//  			chassis->leg_set = 0.13;
//  			chassis->theta_set=0.0f;
			
//  			chassis->x_filter=0.0f;
//  			chassis->x_set=chassis->x_filter;
			
//  		  if(vmcr->L0<0.17f)
//  		  {
//  			 jump_time_r++;
//  		  }
//  		  if(jump_time_r>=5&&jump_time_l>=5)
//  		  { 
//  			 jump_time_r=0;
//  			 jump_time_l=0;
//  			 chassis->leg_set=0.130f;
//  			 chassis->last_leg_set=0.130f;
//  			 chassis->jump_flag_r=0;//缩腿完毕
//  		     chassis->jump_flag_l=0;
//          chassis->help_jump_flag = 0;	
//  		  }
//  		}
//  }	

//右腿离地检测
//    right_flag = ground_detectionR(vmcr,ins);
	//  if(chassis->recover_flag==0)	//倒地自起不需要检测是否离地		
	//  { 
	// 	if(right_flag==1&&left_flag==1&&vmcr->leg_flag==0)
	// 	{ 
	// 		//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
	// 		//排除跳跃的压缩阶段、上升阶段、跳跃的缩腿阶段
	// 			chassis->wheel_motor[0].wheel_T=0.0f;
	// 			vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);

	// 			chassis->x_filter=0.0f;
	// 			chassis->x_set = chassis->x_filter;
	// 			vmcr->Tp=vmcr->Tp+chassis->leg_tp;			 
	// 	}
	// 	else
	// 	{//没有离地
	// 		vmcr->leg_flag=0;//置为0
							
	// 		// if(chassis->jump_flag_r==0)
	// 		// {//不跳跃的时候需要roll轴补偿					
	// 		//  vmcr->F0=vmcr->F0+chassis->roll_f0;//roll轴补偿取反然后加上去   			
	// 		// }
	// 	}
	//  }
	//  else if(chassis->recover_flag==1)

	//  {
	// 	 vmcr->Tp=0.0f;
	// 	 vmcr->F0=0.0f;
	//  }

  //功率控制
    // chassis_power_control(chassis);
	mySaturate(&vmcr->F0,-100.0f,100.0f);//限幅
	VMC_calc_2(vmcr);//计算期望的关节输出力矩

//限幅函数，后期要改参数
	mySaturate(&vmcr->torque_set[1],-18.0f,18.0f);	
	mySaturate(&vmcr->torque_set[0],-18.0f,18.0f);	
		 
}
	
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

uint8_t recover_detect(chassis_t *chassis)
{	
	if(((chassis->myPithR<((-3.1415926f)/10.5f)&&chassis->myPithR>((-3.1415926f)/2.0f))
					  ||(chassis->myPithR>(3.1415926f/12.8f)&&chassis->myPithR<(3.1415926f/2.0f))))	
	{
				
			chassis->leg_set = 0.130;
	
				return 1;//需要自起		
		}

	else
	  {
		  
	  return 0;	
		  
	   }
}

