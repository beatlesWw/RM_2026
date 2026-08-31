/****************************LIFT*******************************/
/****************************CAN2*******************************/
#include "chassisL_task.h"
#include "can.h"
#include "shoot.h"
#include "cmsis_os.h"
vmc_leg_t left;  
float LQR_K_L[12]={  
//  -3.3259  , -0.2272,   -1.2959,   -1.2122 ,   3.3133,    0.3249,
//    2.5219 ,   0.2142 ,   1.8118 ,   1.6073 ,   8.2857 ,   0.4180
-11.0224 , -2.664 ,-11.6436,	-20.2754 , 30.2361 , 3.3863,//      
10.2554 ,  0.9963 ,   1.0764  ,  3.2532 , 40.8928  , 3.5782		
//   -7.9542 ,  -0.7620 ,  -2.3764  , -3.8669 ,   9.5258,    1.4091,
//   11.9374  ,  1.0310 ,   1.4607  ,  1.0218  , 3.6514 ,   1.3526	
//	   
};

extern float Poly_Coefficient[12][4];
extern chassis_t chassis_move_balance;
float jump_time_l;
extern float jump_time_r;	
extern shoot_control_t shoot_control;       
pid_type_def LegL_Pid;	
extern INS_t INS;
uint32_t CHASSL_TIME=1;	
int16_t	  shoot_can_set_current;
	  
void ChassisL_task(void)
{
  while(INS.ins_flag==0)
	{
	  osDelay(1);	
	}	
		shoot_control.shoot_send_flag = 0;

	
        ChassisL_init(&chassis_move_balance,&left,&LegL_Pid);	
	
		    //shoot init
            shoot_init();

	while(1) 
	{	
	
		chassisL_feedback_update(&chassis_move_balance,&left,&INS);
		
		shoot_can_set_current = shoot_control_loop();
		
		chassisL_control_loop(&chassis_move_balance,&left,&INS,LQR_K_L,&LegL_Pid);
	  
//		shoot_can_set_current = shoot_control_loop();        
//			
		if(chassis_move_balance.start_flag==1)	
	
	{
	mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.8f,left.torque_set[1]);//left.torque_set[1]
	osDelay(CHASSL_TIME);
	mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.8f,left.torque_set[0]);
	osDelay(CHASSL_TIME);

	chassis_move_balance.wheel_motor[1].given_current = chassis_move_balance.wheel_motor[1].wheel_T/0.000366211f;			
	CAN_cmd_gimbal(-chassis_move_balance.wheel_motor[1].given_current,0,shoot_can_set_current,0);
	osDelay(CHASSL_TIME);	
	 }
	 	else if(chassis_move_balance.start_flag==0)	
		{			
		    mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.8f,0.0f);//left.torque_set[1]
			osDelay(CHASSL_TIME);
			mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.8f,0.0f);
			osDelay(CHASSL_TIME);

			CAN_cmd_gimbal(0,0,0,0);
			osDelay(CHASSL_TIME);			
		}
	}
}

void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc,pid_type_def *legl)
{
    const static float legl_pid[3] = 
	                                // {0,0,0};
	                                {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
    
	static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};

	joint_motor_init(&chassis->joint_motor[2],3,MIT_MODE);
	joint_motor_init(&chassis->joint_motor[3],4,MIT_MODE);
	
	VMC_init(vmc);
	
	PID_init(legl, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
 
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hcan1,chassis->joint_motor[3].para.id,chassis->joint_motor[3].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hcan1,chassis->joint_motor[2].para.id,chassis->joint_motor[2].mode);
	  osDelay(1);
	}

}

void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
    vmc->phi1=pi/2.0f+chassis->joint_motor[2].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[3].para.pos;
		
	chassis->myPithL=0.0f-ins->Pitch;
	chassis->myPithGyroL=0.0f-ins->Gyro[0];
	
	chassis->wheel_motor[1].vel = chassis->wheel_motor[1].speed_rpm * CHASSIS_MOTOR_RPM_TO_OMG_SEN;  
	chassis->wheel_motor[1].speed = 0.0004998609952f * chassis->wheel_motor[1].speed_rpm; 
	chassis->wheel_motor[1].wheel_T = CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN * chassis->wheel_motor[1].given_current;
	
	chassis->yaw_motor_angle = motor_ecd_to_angle_change(chassis->motor_chassis[4].ecd,0);
	
	static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

	speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (chassis->wheel_motor[3].speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;
	
}

extern uint8_t right_flag;
uint8_t left_flag;
extern float mg;
extern float jump_time_r;

void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,pid_type_def *leg)
{
	VMC_calc_left(vmcl,ins,((float)CHASSL_TIME)*3.0f/1000.0f);
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	
	}

	chassis->wheel_motor[1].wheel_T=   ( LQR_K[0]*(vmcl->theta-0.0f)
	                                    +LQR_K[1]*(vmcl->d_theta-0.0f)
										+LQR_K[2]*((chassis->x_set)-chassis->x_filter)
										+LQR_K[3]*((chassis->v_set)-chassis->v_filter2)
										+LQR_K[4]*(chassis->myPithL-(-0.0f)) 
										+LQR_K[5]*(chassis->myPithGyroL-0.0f));

	     vmcl->Tp=(      LQR_K[6]*(vmcl->theta-0.0f)
						+LQR_K[7]*(vmcl->d_theta-0.0f)
						+LQR_K[8]*((chassis->x_set)-chassis->x_filter)
						+LQR_K[9]*(chassis->v_set-chassis->v_filter2)
						+LQR_K[10]*(chassis->myPithL-(-0.0f))
						+LQR_K[11]*(chassis->myPithGyroL-0.0f));		


	    // vmcl->Tp=vmcl->Tp+chassis->leg_tp;
        // chassis->wheel_motor[1].wheel_T = 0.0f;
	    chassis->wheel_motor[1].wheel_T = chassis->wheel_motor[1].wheel_T;
		// -chassis->turn_T;

		mySaturate(&chassis->wheel_motor[1].wheel_T,-4.2f,4.2f);
        		
		// vmcl->F0=17.2f/arm_cos_f32(vmcl->theta)+PID_calc(leg,vmcl->L0,chassis->leg_set)+chassis->roll_f0;

//跳跃逻辑
//  	if(chassis->chassis_RC->rc.s[1] ==1)
//  {
//  		if(chassis->chassis_RC->rc.ch[4] >500){
		
//  		chassis->help_jump_flag =1;
//  		 }
//  		 if(chassis->jump_flag_l==0&& chassis->help_jump_flag ==1)
//  		{
			
//  		 chassis->leg_set = 0.130;
	
//  		 if(vmcl->L0<0.17f)
//  		 {
//  		  jump_time_l++;
//  		 }
//  		 if(jump_time_l>=10&&jump_time_l>=10)
//  		 {  
//  			 jump_time_l=0;
//  			 jump_time_r=0;
//  			 chassis->jump_flag_l=1;
//  			 chassis->jump_flag_r=1;
//  		 }			 
//  		}
	
//  		else if(chassis->jump_flag_l==1&& chassis->help_jump_flag ==1)
//  		{

//  			chassis->leg_set = 0.30;
			
			
//  			 if(vmcl->L0>0.22f)
//  			 {
//  				jump_time_l++;
//  			 }
//  			 if(jump_time_l>=10&&jump_time_l>=10)
//  			 {  
//  				 jump_time_l=0;
//  				  jump_time_r=0;
//  				 chassis->jump_flag_l=2;
//  				 chassis->jump_flag_r=2;
//  			 }
//  		}
					
//  		else if(chassis->jump_flag_l==2&& chassis->help_jump_flag ==1)
//  		{
//  		 	chassis->leg_set = 0.13;
//  			chassis->theta_set=0.0f;
//  			chassis->x_filter=0.0f;
//  			chassis->x_set=chassis->x_filter;
//  		  if(vmcl->L0<0.18f)
//  		  {
//  			 jump_time_l++;
//  		  }
//  		  if(jump_time_l>=5&&jump_time_r>=5)
//  		  { 
//  				 jump_time_l=0;
//  				 jump_time_r=0;
//  				 chassis->leg_set=0.130f;
//  				 chassis->last_leg_set=0.130f;
//  				 chassis->jump_flag_l=0;
//  				 chassis->jump_flag_r=0;
//  			  chassis->help_jump_flag = 0;

//  		  }
//  		}
//  }	
 
//左腿离地检测
	//  left_flag=ground_detectionL(vmcl,ins);
	 
	//  if(chassis->recover_flag==0)		
	//  {
	// 	if(right_flag==1&&left_flag==1&&vmcl->leg_flag==0)
	// 	{  
	// 			chassis->wheel_motor[1].wheel_T=0.0f;
	// 			vmcl->Tp=LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f);

	// 			chassis->x_filter=0.0f;
	// 			chassis->x_set=chassis->x_filter;
	// 			chassis->turn_set=chassis->total_yaw;
	// 			vmcl->Tp=vmcl->Tp+chassis->leg_tp;	
	// 	}
	// 	else
	// 	{
	// 		vmcl->leg_flag=0;
	// 		if(chassis->jump_flag_l==0)
	// 		{
				
	// 			vmcl->F0=vmcl->F0+chassis->roll_f0;			
	// 		}
	// 	}
	//  }

	//  else if(chassis->recover_flag==1)
	//  {
	// 	 vmcl->Tp=0.0f;
	// 	 vmcl->F0=0.0f;
	//  }

	mySaturate(&vmcl->F0,-100.0f,100.0f); 

	VMC_calc_2(vmcl);

    mySaturate(&vmcl->torque_set[1],-1.0f,1.0f);	
	mySaturate(&vmcl->torque_set[0],-1.0f,1.0f);	
}

/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}
