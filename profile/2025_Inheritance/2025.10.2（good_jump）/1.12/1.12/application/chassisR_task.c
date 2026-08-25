
/****************************RIGHT*******************************/
/****************************CAN1*******************************/
//C板放置：R标横着 R的上半部分对应没有贴纸的，下半部分（竖和娜）娜对应唯一一个贴标签6的电机
//C板CAN线序左L右H



#include "chassisR_task.h"
#include "can.h"
#include "cmsis_os.h"

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
	float LQR_K_R[12]={       
	-1.0224 , -0.2554 ,-3.6436,	-5.8554 , 17.2361 , 2.3863,//      短距离不会晃动，有扰动后疯狂抖动
	9.2554 ,   1.7963 ,   2.7764  ,  2.5532 , 4.6828  , -1.1782		



	};
					   
	////	 Q=diag([5 1 20 50 600 1]);

	////R=[1 0;0 0.25];
	//	
	//	//nice但是三子台阶收敛翻车
	////float Poly_Coefficient[12][4] = {-530.56254, 522.04951, -201.35625, 5.81346,
	////	-113.71648, 106.71366, -41.22171, 1.68544,
	////	-96.25677, 83.18131, -21.24060, -1.83299, 
	////	-197.99263, 171.28794, -44.30590, -3.73680,
	////	-157.96645, 152.23392, -83.02290, 33.04249,
	////	26.54151, -22.98194, 1.25121, 4.06498,
	////	659.16889, -580.30264, 187.96273, -3.74313, 
	////	125.51825, -104.88944, 30.97526, -0.77725, 
	////127.31478, -96.58071, 20.42665, 1.24595, 
	////264.35282, -199.69374, 42.30404, 2.41148,
	////404.16706, -386.42653, 156.14425, 25.47839, 
	////-33.79435, 20.75503, 2.41196, 1.89930};		
	//		

//ooo
	//float Poly_Coefficient[12][4] = {-421.75465, 404.14066, -163.18492, 1.52481, -100.33598, 88.97437, -33.12351, 0.46394, -13.79006, 20.45913, -5.20346, -6.44626, 0.23096, 8.70198, -1.16290, -7.84332, -145.33854, 158.48464, -89.03226, 32.92067, -27.19789, 19.76695, -10.47246, 4.95830, 615.75677, -496.49431, 155.68024, -1.46787, 129.15010, -93.95712, 23.77051, 0.02808, 207.43929, -152.95834, 33.54335, 1.53506, 211.73999, -148.21559, 29.04610, 2.57854, 88.69753, -151.25099, 93.98502, 36.97904, -21.28958, 10.81717, 4.85406, 2.35990};


//float Poly_Coefficient[12][4] = {-790.61928, 782.26690, -284.57084, 7.42844, -117.89599, 116.14906, -45.60867, 1.97170, -128.23953, 110.73663, -28.62830, -1.05241, -294.42237, 255.24433, -67.05019, -2.29070, -254.16480, 252.82343, -116.41452, 33.85460, 12.40277, -8.05018, -2.74366, 3.37031, 830.45512, -728.15279, 216.14258, -0.02068, 124.03592, -107.56058, 30.99103, -0.52904, 66.74994, -43.96404, 3.29141, 2.96663, 160.80023, -108.06718, 10.23011, 6.43703, 655.93861, -601.09142, 213.79050, 26.98381, 30.72005, -33.34413, 16.23659, 1.46101};

//	float Poly_Coefficient[12][4] = {-357.22525, 376.01506, -175.04320, 1.05545, -126.84092, 101.99275, -33.95612, 1.45318, -28.91689, 20.96706, -4.08418, -1.76681, -78.04556, 56.29538, -11.29340, -3.85627, 237.29503, -71.53535, -60.50602, 24.75084, 57.75872, -28.25933, -4.57790, 4.11389, 902.16034, -769.43960, 231.34278, -2.54894, 152.35310, -120.99880, 26.66722, -1.48312, 53.10767, -33.06381, 3.62372, -0.76531, 127.03223, -80.27883, 9.41370, -2.09424, 184.42249, -251.29960, 127.17416, 30.98789, -7.14941, -12.65231, 14.24176, 4.91018};
		
//float Poly_Coefficient[12][4] = {
//-233.439546850467	,276.935271152620	,-150.545961298122	,0.843745991347781,
//-1.56018123585066	,2.85117617176040	,-9.67122838857148	,0.119218620506790,
//-12.1580443848496	,9.87447505897652	,-2.48602678575299	,-1.38185822602069,
//-29.6767308463453	,23.6000958091375	,-5.96756949328194	,-3.81676760425510,
//-171.739278698152	,229.201159577166	,-125.886856608897	,32.8314155186377 ,
//-18.9570800917976	,27.8130822713852	,-16.2987645794551	,5.26079869181403 ,
//535.583748857383	,-485.614565679618	,161.855573578961	,8.72394124696298 ,
//29.7519359736402	,-27.3325091837029	,5.43642886775718	,0.366331774830735,
//-43.4842580259166	,45.8993363811188	,-18.5430380534061	,2.19317127540535 ,
//-116.740807610165	,122.785000430884	,-49.5506144081597	,5.55564076559390 ,
//915.863217849202	,-880.401962687950	,319.234949102067	,11.7879938147306 ,
//144.587014300436	,-140.804059887658	,52.0224364018309	,0.817848106153873
//};	
		//float Poly_Coefficient[12][4] = {-750.37091, 674.65333, -264.83487, 6.61073, -165.70860, 130.33069, -48.34246, 1.99983, -112.41372, 88.27984, -22.36833, -1.81057, -274.12904, 213.96496, -54.59379, -4.26855, 124.07351, 6.68282, -83.93672, 45.00612, 84.42312, -52.97705, 3.26273, 6.17020, 1002.54935, -908.03209, 299.65431, -1.61756, 172.43066, -143.93387, 38.95117, -0.98294, 95.29996, -61.97671, 6.31565, 2.44423, 236.65063, -155.63029, 17.58057, 5.32039, 331.06497, -457.91684, 244.40584, 32.70065, -35.98225, -2.76939, 21.70274, 2.72641};	


//float Poly_Coefficient[12][4] = {-1308.15112, 1154.41326, -389.10544, 13.07885, -267.54549, 219.87755, -72.05800, 3.41637, -271.38652, 211.35690, -49.18523, -2.13719, -526.02854, 408.41929, -95.69906, -4.05356, 468.65948, -231.39539, -40.97693, 42.69389, 166.93870, -112.79620, 14.41839, 6.02959, 1178.41529, -1013.07559, 304.48783, -4.99252, 215.85062, -175.81295, 42.73170, -1.95306, 195.78436, -136.03975, 23.37179, -0.05201, 392.36906, -275.05871, 48.82494, -0.85440, -115.50630, -71.40573, 117.54034, 40.64108, -102.21632, 52.77445, 4.09113, 5.15949};

//0.1095
//float Poly_Coefficient[12][4] = {-1468.11737, 1276.21227, -416.50563, 13.50559, -281.17334, 229.40738, -73.89423, 3.56771, -287.45197, 224.04286, -52.74681, -1.67702, -549.53405, 427.56240, -101.71556, -2.97402, 227.90763, -69.49427, -62.60748, 37.21026, 108.07369, -71.87229, 8.26912, 4.39202, 1006.17455, -863.37927, 255.05196, -2.63741, 175.25694, -143.84517, 36.71808, -1.45713, 147.65004, -101.44299, 15.84182, 1.23401, 293.55831, -204.28860, 33.86700, 1.70793, 91.20832, -183.73021, 115.78735, 34.62692, -40.70107, 15.38262, 6.81432, 3.38996};

float Poly_Coefficient[12][4] = {-688.11451, 624.48182, -224.51725, 4.56519, -152.59905, 121.93600, -37.46152, 1.83974, -49.39516, 39.29094, -10.02675, -0.92060, -122.21126, 96.34839, -24.47413, -1.98153, -79.05408, 120.55925, -74.75379, 22.65550, 9.53059, -0.21933, -5.45542, 2.90177, 472.72026, -445.09530, 143.16510, 6.13144, 107.87232, -89.47695, 22.55285, -0.62237, -42.26527, 39.33083, -15.43315, 2.56475, -80.30675, 76.90728, -31.69804, 5.34967, 529.68109, -502.56955, 183.08719, 9.46531, 64.65825, -62.85518, 24.11282, 0.60818};

	
	
	
	vmc_leg_t right;//右腿

	extern INS_t INS;

	extern vmc_leg_t left;

	chassis_t chassis_move_balance;

	float jump_time_r;
	extern float jump_time_l;

						
	pid_type_def LegR_Pid;//右腿的腿长pd
	pid_type_def Tp_Pid;//防劈叉补偿pd
	pid_type_def Turn_Pid;//转向pd
	pid_type_def Roll_Pid;//横滚角补偿pd






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

		float H = 1;
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



	uint32_t CHASSR_TIME=1;	


void ChassisR_task(void)
{

		chassis_move_balance.leg_set=0.13; 
		chassis_move_balance.turn_set = 0;
		chassis_move_balance.x_set = 0;
		chassis_move_balance.v_set = 0;
		chassis_move_balance.recover_flag = 0;
		chassis_move_balance.roll_set = -0.02;


	while(INS.ins_flag==0)
	{//等待加速度收敛
	osDelay(1);	
	}


	//Fast  电机初始化
	//包括了右边关节电机的id,mode初始化，以及电机的使能；
	ChassisR_init(&chassis_move_balance,&right,&LegR_Pid);

	Pensation_init(&Roll_Pid,&Tp_Pid,&Turn_Pid);//补偿pid初始化

	//chassis_move_balance.leg_set = 0.127f;//原始腿长    腿长限制在0.127------0.32  会比设定高1-2cm
	while(1)
	{
	float dt = (float)CHASSR_TIME/1000.0f;	

	if( chassis_move_balance.chassis_RC->rc.s[0] == 3){
	chassis_move_balance.start_flag=1;	
	}


	if( chassis_move_balance.chassis_RC->rc.s[0] == 2){

	chassis_move_balance.start_flag=0;

	}

	//		chassis_move_balance.recover_flag = recover_detect(&chassis_move_balance);	



	if(chassis_move_balance.start_flag==1 )

	{
		//		chassis_move_balance.target_v=((float)chassis_move_balance.chassis_RC->rc.ch[1])*(0.00800f);//往前大于0	
		//		slope_following(&chassis_move_balance.target_v,&chassis_move_balance.v_set,0.01f);	//	坡度跟随  地胶
		chassis_move_balance.target_v=((float)chassis_move_balance.chassis_RC->rc.ch[1])*(0.00500f);//往前大于0	
		slope_following(&chassis_move_balance.target_v,&chassis_move_balance.v_set,0.004f);	//	坡度跟随  地胶

		chassis_move_balance.x_set =chassis_move_balance.x_set+chassis_move_balance.v_set*(float)CHASSR_TIME*2.0f/1000.0f;
		chassis_move_balance.turn_set = chassis_move_balance.turn_set+chassis_move_balance.chassis_RC->rc.ch[2]*(-0.00003f);//往右大于0				
		chassis_move_balance.leg_set=chassis_move_balance.leg_set+(((float)chassis_move_balance.chassis_RC->rc.ch[3])*(0.000005f)); 
		//		slope_following(&chassis_move_balance.target_leg,&chassis_move_balance.leg_set,0.003f);
		//		chassis_move_balance.roll_target= ((float)chassis_move_balance.chassis_RC->rc.ch[0])*(0.0004f);
		//		slope_following(&chassis_move_balance.roll_target,&chassis_move_balance.roll_set,0.0075f);			
		//			
	if(fabsf(chassis_move_balance.last_leg_set-chassis_move_balance.leg_set)>0.0005f)
	{//遥控器控制腿长在变化
		right.leg_flag=1;	//为1标志着遥控器在控制腿长伸缩，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判为离地了
		left.leg_flag=1;	 			
	}
	chassis_move_balance.last_leg_set=chassis_move_balance.leg_set;

	mySaturate(&chassis_move_balance.leg_set,0.127f,0.32f);


	}



	//更新数据
	chassisR_feedback_update(&chassis_move_balance,&right,&INS);


	//	chassis_move_balance.turn_set = 0;

	//	 chassis_set_mode(&chassis_move_balance);

	//控制计算
	chassisR_control_loop(&chassis_move_balance,&right,&INS,LQR_K_R,&LegR_Pid);	
	//		mit_ctrl(&hcan1,0X06,0,1,0,1,0);



	if(chassis_move_balance.start_flag==1)	
	{
		mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.5f,right.torque_set[1]);//right.torque_set[1]
		osDelay(CHASSR_TIME);
		mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.5f,right.torque_set[0]);//right.torque_set[0]
		osDelay(CHASSR_TIME);
		//			
		//			mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[1]
		//			osDelay(CHASSR_TIME);
		//			mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[0]
		//			osDelay(CHASSR_TIME);
		//			//顺时针为正
		chassis_move_balance.wheel_motor[0].given_current = (chassis_move_balance.wheel_motor[0].wheel_T/0.000366211f);
		CAN_cmd_chassis(-chassis_move_balance.wheel_motor[0].given_current);
		osDelay(CHASSR_TIME);
	}


	else if(chassis_move_balance.start_flag==0)	
	{
		mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[1]
		osDelay(CHASSR_TIME);
		mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[0]
		osDelay(CHASSR_TIME);
		CAN_cmd_chassis(0);
		osDelay(CHASSR_TIME);
	    }

	 }
}

	void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,pid_type_def *legr)
	{

	//in beginning， chassis mode is DOWN
	//底盘开机状态为DOWN模式  无力模式  电机失能or控制参数给0
	//    chassis->chassis_mode = CHASSIS_DOWN_MODE;
	//	

	//get remote control point
	//调用获取遥控器指针，得到当前各个通道的值  
	chassis->chassis_RC = get_remote_control_point();


	//void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legr)
	const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};

	//右边关节电机的初始化
	joint_motor_init(&chassis->joint_motor[0],6,MIT_MODE);//发送id为6
	joint_motor_init(&chassis->joint_motor[1],8,MIT_MODE);//发送id为8


	VMC_init(vmc);//给杆长赋值
	//腿长pid初始化
	PID_init(legr, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

	for(int j=0;j<10;j++)
	{
	enable_motor_mode(&hcan1,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
	osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	enable_motor_mode(&hcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	osDelay(1);
	}





	}


	void Pensation_init(pid_type_def *roll,pid_type_def *Tp,pid_type_def *turn)
	{//补偿pid初始化：横滚角补偿、防劈叉补偿、偏航角补偿
	const static float roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI,ROLL_PID_KD};
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	//	横滚角补偿
	PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
	//防劈叉补偿
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	//偏航角补偿
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);

	}

	void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
	{
	vmc->phi1=pi/2.0f+chassis->joint_motor[0].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[1].para.pos;

	chassis->myPithR=ins->Pitch;
	chassis->myPithGyroR=ins->Gyro[1];

	chassis->total_yaw=ins->YawTotalAngle;
	chassis->roll=ins->Roll;
	chassis->theta_err=0.0f-(vmc->theta+left.theta);


	chassis->wheel_motor[0].vel = chassis->wheel_motor[0].speed_rpm * CHASSIS_MOTOR_RPM_TO_OMG_SEN;  //角速度
	chassis->wheel_motor[0].speed = 0.0004998609952f * chassis->wheel_motor[0].speed_rpm;  //速度
	chassis->wheel_motor[0].wheel_T = CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN * chassis->wheel_motor[0].given_current;

		chassis->v = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;	
//	chassis->v = data_fusion();
	chassis->x = chassis->x + chassis->v*((float)1/1000.0f);

	if(ins->Pitch<(3.1415926f/15.5f)&&ins->Pitch>(-3.1415926f/15.5f))
	{//根据pitch角度判断倒地自起是否完成
	chassis->recover_flag=0;
	}

	}


	void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
	{ 
	if(data_len==8)
	{//返回的数据有8个字节
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
	}
	}



	uint8_t right_flag=0;
	extern uint8_t left_flag;

	int jump_right_flag = 0;


	void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,pid_type_def *leg)
	{
		
	VMC_calc_1_right(vmcr,ins,((float)CHASSR_TIME)*3.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是3*0.001秒

	for(int i=0;i<12;i++)
	{
	LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0);	
	}

	//		chassis->turn_T=PID_calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw轴pid计算
	chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*(ins->Gyro[2]);//这样计算更稳一点

	chassis->roll_f0=Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[0];

	mySaturate(&chassis->roll_f0,-Roll_Pid.max_out,Roll_Pid.max_out);

	chassis->leg_tp=PID_calc(&Tp_Pid, chassis->theta_err,0.0f);//防劈叉pid计算



	//轮毂电机
	chassis->wheel_motor[0].wheel_T = (LQR_K[0]*(vmcr->theta-0.0f)
	+LQR_K[1]*(vmcr->d_theta-0.0f)
//	+LQR_K[2]*(chassis->x-chassis->x_set)
//	+LQR_K[3]*(chassis->v-chassis->v_set)
									+LQR_K[2]*(chassis->x_filter-chassis->x_set)
									+LQR_K[3]*(chassis->v_filter2-0.4f*chassis->v_set)
	+ LQR_K[4]*(chassis->myPithR-0.01025f-chassis->phi_set) 
	+ LQR_K[5]*(chassis->myPithGyroR-0.01f));	

	//chassis->wheel_motor[0].wheel_T= ( LQR_K[3]*(chassis->v-0.4f*chassis->v_set) - LQR_K[4]*(chassis->myPithR-0.04f-chassis->phi_set) - LQR_K[5]*(chassis->myPithGyroR-0.0f));		


	////		//右边髋关节输出力矩				

	vmcr->Tp= (LQR_K[6]*(vmcr->theta-0.0f)	
	+LQR_K[7]*(vmcr->d_theta-0.0f)
//	+LQR_K[8]*(chassis->x-chassis->x_set)
//	+LQR_K[9]*(chassis->v-chassis->v_set)
							+LQR_K[8]*(chassis->x_filter-chassis->x_set)
				            +LQR_K[9]*(chassis->v_filter2-0.4f*chassis->v_set)
	+LQR_K[10]*(chassis->myPithR-0.01025f-chassis->phi_set)
	+LQR_K[11]*(chassis->myPithGyroR-0.01f));

	vmcr->Tp=vmcr->Tp+chassis->leg_tp;//髋关节输出力矩	
	chassis->wheel_motor[0].wheel_T=chassis->wheel_motor[0].wheel_T - chassis->turn_T;	//轮毂电机输出力矩	


	//		vmcr->Tp=vmcr->Tp;

	//对轮毂电机输出限幅
	mySaturate(&chassis->wheel_motor[0].wheel_T,-4.2f,4.2f);




	//不跳跃的时候需要roll轴补偿
	//	 vmcr->F0=vmcr->F0-chassis->roll_f0;//roll轴补偿取反然后加上去    			
	vmcr->F0=17.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,chassis->leg_set)-chassis->roll_f0;//前馈+pd


	//	
	//	vmcr->F0=PID_calc(leg,vmcr->L0,chassis->leg_set);//前馈+pd
	//	vmcr->F0=11.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,chassis->leg_set)-chassis->roll_f0;//前馈+pd

	// vmcr->F0=vmcr->F0+chassis->roll_f0;//roll轴补偿取反然后加上去    		


	if(chassis->chassis_RC->rc.s[1] ==1)
	{
	if(chassis->chassis_RC->rc.ch[4] >500){

	chassis->help_jump_flag =1;

	}
	if(chassis->jump_flag_r==0 && chassis->help_jump_flag ==1)
	{//压缩阶段
	//		     chassis->leg_set = 0.130;
	vmcr->F0=17.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,0.130f);//前馈+pd

	if(vmcr->L0<0.16f)
	{
	jump_time_r++;
	}
	if(jump_time_r>=10&&jump_time_l>=10)
	{  
	jump_time_r=0;
	jump_time_l=0;
	chassis->jump_flag_r=1;//压缩完毕进入上升加速阶段
	chassis->jump_flag_l=1;//压缩完毕进入上升加速阶段
	}			 
	}

	else if(chassis->jump_flag_r==1&& chassis->help_jump_flag ==1)
	{//上升加速阶段			
	//chassis->leg_set = 0.30;
	vmcr->F0=17.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,0.30);//前馈+pd

	if(vmcr->L0>0.26f)
	{
	jump_time_r++;
	}
	if(jump_time_r>=10&&jump_time_l>=10)
	{  
	jump_time_r=0;
	jump_time_l=0;
	chassis->jump_flag_l=2;//上升完毕进入缩腿阶段
	chassis->jump_flag_r=2;
	}	 
	}



	else if(chassis->jump_flag_r==2&& chassis->help_jump_flag ==1)
	{//缩腿阶段
	//				chassis->leg_set = 0.128;
	vmcr->F0=17.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,0.130);//前馈+pd

	chassis->theta_set=0.0f;
	chassis->x_filter=0.0f;
	chassis->x_set=chassis->x_filter;

	if(vmcr->L0<0.16f)
	{
	jump_time_r++;
	}
	if(jump_time_r>=10&&jump_time_l>=10)
	{ 
	jump_time_r=0;
	jump_time_l=0;
	chassis->leg_set=0.140f;
	chassis->last_leg_set=0.140f;
	chassis->jump_flag_r=0;//缩腿完毕
	chassis->jump_flag_l=0;
	chassis->help_jump_flag = 0;			  
	}
	}

	else
	{
	vmcr->F0=17.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,chassis->leg_set);//前馈+pd
	}
	}	



	//	vmcr->F0=11.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,chassis->leg_set);//前馈+pd

	right_flag=ground_detectionR(vmcr,ins);//右腿离地检测

	chassis ->recover_flag = recover_detectR(chassis);


	if(chassis->recover_flag==0)		
	{//倒地自起不需要检测是否离地	 
	//		if((right_flag==1&&left_flag==1&&vmcr->leg_flag==0&&chassis->jump_flag!=1&&chassis->jump_flag2!=1&&chassis->jump_flag!=2&&chassis->jump_flag2!=2)
	//			||chassis->jump_flag==3)
	if(right_flag==1&&left_flag==1&&vmcr->leg_flag==0&&chassis->jump_flag_r!=1&&chassis->jump_flag_l!=1&&chassis->jump_flag_r!=2&&chassis->jump_flag_l!=2)
	{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
	//排除跳跃的压缩阶段、上升阶段、跳跃的缩腿阶段
	chassis->wheel_motor[0].wheel_T=0.0f;
	vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);

	//				chassis->x_filter=0.0f;
	//				chassis->x_set=chassis->x_filter;
	chassis->x=0.0f;
	chassis->x_set=chassis->x;
	vmcr->Tp=vmcr->Tp+chassis->leg_tp;			 
	}
	else
	{//没有离地
	vmcr->leg_flag=0;//置为0


	//			if(chassis->jump_flag==0)
	//			{//不跳跃的时候需要roll轴补偿						
	//			 vmcr->F0=vmcr->F0+chassis->roll_f0;//roll轴补偿取反然后加上去    			
	//			}
	}
	}
	else if(chassis->recover_flag==1)
	//if(chassis->recover_flag==1)

	{
	vmcr->Tp=0.0f;
	vmcr->F0=0.0f;

	vmcr->torque_set[0] = 0;
	vmcr->torque_set[1] = 0;
	}






	mySaturate(&vmcr->F0,-100.0f,100.0f);//限幅 

	VMC_calc_2(vmcr);//计算期望的关节输出力矩


	//限幅函数，后期要改参数
	mySaturate(&vmcr->torque_set[1],-18.0f,18.0f);	
	mySaturate(&vmcr->torque_set[0],-18.0f,18.0f);	

	}


	//static void chassis_set_mode(chassis_move_t *chassis_move_mode)
	//{
	//    if (chassis_move_mode == NULL)
	//    {
	//        return;
	//    }
	//    //in file "chassis_behaviour.c"
	//    chassis_behaviour_mode_set(chassis_move_mode);
	//}	
	//	




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




	/**
	* @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
	* @param[out]     chassis_move_mode: "chassis_move" valiable point
	* @retval         none
	*/
	/**
	* @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
	* @param[out]     chassis_move_mode:"chassis_move"变量指针.
	* @retval         none
	*/
	//static void chassis_set_mode(chassis_t *chassis_move_mode)
	//{
	//    if (chassis_move_mode == NULL)
	//    {
	//        return;
	//    }
	//    //in file "chassis_behaviour.c"
	//    chassis_behaviour_mode_set(chassis_move_mode);
	//}

	///**
	//  * @brief          set chassis control target-point, movement control value is set by "chassis_behaviour_control_set".
	//  * @param[out]     chassis_move_update: "chassis_move" valiable point
	//  * @retval         none
	//  */
	///**
	//  * @brief          设置底盘控制目标值, 运动控制值是通过chassis_behaviour_control_set函数设置的
	//  * @param[out]     chassis_move_update:"chassis_move"变量指针.
	//  * @retval         none
	//  */
	//static void chassis_set_contorl(chassis_t *chassis_move_control)
	//{
	//    if (chassis_move_control == NULL)
	//    {
	//      return;
	//    }

	//    fp32 vx_set = 0.0f, turn_set = 0.0f;
	//		
	//    //get movement control target-points, 获取运动控制目标值
	//	
	//	//先对底盘行为模式进行设定与选择
	//    chassis_behaviour_control_set(&vx_set, &turn_set, chassis_move_control);

	//	//
	//    if (chassis_move_control->chassis_mode == CHASSIS_REMOTE_MODE)
	//    {
	//		
	//    }
	//		else if (chassis_move_control->chassis_mode == CHASSIS_BALANCE_MODE)
	//    {

	//    }
	//		else if (chassis_move_control->chassis_mode == CHASSIS_DOWN_MODE)
	//		{
	//			
	//    }
	//}	
	uint8_t recover_detectR(chassis_t *chassis)
	{	
	if(((chassis->myPithR<((-3.1415926f)/9.4f)&&chassis->myPithR>((-3.1415926f)/2.0f))
	||(chassis->myPithR>(3.1415926f/9.4f)&&chassis->myPithR<(3.1415926f/2.0f))))	
	{

	chassis->leg_set = 0.125;

	return 1;//需要自起	
	}

	else
	{
	return 0;	
	}
	}

