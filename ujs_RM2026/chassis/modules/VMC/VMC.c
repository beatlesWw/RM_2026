#include "VMC.h"
#include "robot_def.h"

#define G_ACC 9.81f
#define WHEEL_SUPPORT_BIAS 4.842f
#define WHEEL_MASS_EQ (WHEEL_SUPPORT_BIAS / G_ACC)
#define WHEEL_Z_DDOT_LIMIT 25.0f
#define VMC_DERIV_LPF_ALPHA 0.12f
#define VMC_ACCEL_LPF_ALPHA 0.10f
#define VMC_ZW_LPF_ALPHA 0.10f
#define GROUND_DETECTION_FN_THRESHOLD 30.0f

static float VMC_Clamp(float in, float min, float max)
{
	if (in < min)
	{
		return min;
	}
	if (in > max)
	{
		return max;
	}
	return in;
}

static float VMC_LowPass(float input, float *state, float alpha)
{
	*state += alpha * (input - *state);
	return *state;
}

static uint8_t VMC_IsGroundLost(float fn_avg, vmc_leg_t *vmc, attitude_t *ins)
{
	(void)vmc;
	(void)ins;
	if (fn_avg < GROUND_DETECTION_FN_THRESHOLD)
	{
		return 1;
	}
	return 0;
}

static float VMC_GetBodyVerticalAccel(void)
{
	INS_t *ins_ptr = INS_GetPointer();
	if (ins_ptr != 0 && ins_ptr->init == 1)
	{
		return ins_ptr->MotionAccel_n[2];
	}
	return 0.0f;
}

static float VMC_CalcWheelVerticalAccel(vmc_leg_t *vmc)
{
	float sin_theta = arm_sin_f32(vmc->theta);
	float cos_theta = arm_cos_f32(vmc->theta);
	vmc->z_m_ddot = VMC_LowPass(VMC_GetBodyVerticalAccel(), &vmc->z_m_ddot_lpf, VMC_ACCEL_LPF_ALPHA);
	float d_L0 = VMC_LowPass(vmc->d_L0, &vmc->d_L0_lpf, VMC_DERIV_LPF_ALPHA);
	float dd_L0 = VMC_LowPass(vmc->dd_L0, &vmc->dd_L0_lpf, VMC_DERIV_LPF_ALPHA);
	float d_theta = VMC_LowPass(vmc->d_theta, &vmc->d_theta_lpf, VMC_DERIV_LPF_ALPHA);
	float dd_theta = VMC_LowPass(vmc->dd_theta, &vmc->dd_theta_lpf, VMC_DERIV_LPF_ALPHA);

	return vmc->z_m_ddot
		     - dd_L0 * cos_theta
		     + 2.0f * d_L0 * d_theta * sin_theta
		     + vmc->L0 * dd_theta * sin_theta
		     + vmc->L0 * d_theta * d_theta * cos_theta;
}

static float VMC_CalcNormalForce(vmc_leg_t *vmc)
{
	float sin_theta = arm_sin_f32(vmc->theta);
	float cos_theta = arm_cos_f32(vmc->theta);
	float L0_safe = vmc->L0;
	float support_from_joint;
	vmc->z_w_ddot = VMC_CalcWheelVerticalAccel(vmc);
//保证不出现奇异值
	if (L0_safe < 0.01f)
	{
		L0_safe = 0.01f;
	}

	support_from_joint = vmc->F0_test * cos_theta + vmc->Tp * sin_theta / L0_safe;
	vmc->z_w_ddot = VMC_Clamp(vmc->z_w_ddot, -WHEEL_Z_DDOT_LIMIT, WHEEL_Z_DDOT_LIMIT);
	vmc->z_w_ddot = VMC_LowPass(vmc->z_w_ddot, &vmc->z_w_ddot_lpf, VMC_ZW_LPF_ALPHA);

	return support_from_joint + WHEEL_SUPPORT_BIAS 
	+ WHEEL_MASS_EQ * vmc->z_w_ddot
	;
}

void VMC_init(vmc_leg_t *vmc)//给杆长赋值
{
	vmc->l5=0.0f;//AE长度 //单位为m
	vmc->l1=0.21f;//单位为m
	vmc->l2=0.25f;//单位为m
	vmc->l3=0.25f;//单位为m
	vmc->l4=0.21f;//单位为m
	vmc->d_theta_lpf = 0.0f;
	vmc->dd_theta_lpf = 0.0f;
	vmc->d_L0_lpf = 0.0f;
	vmc->dd_L0_lpf = 0.0f;
	vmc->z_m_ddot_lpf = 0.0f;
	vmc->z_w_ddot_lpf = 0.0f;
	vmc->z_w_ddot = 0.0f;
}

//计算theta和d_theta给lqr用，同时也计算腿长L0
void VMC_calc_right(vmc_leg_t *vmc,attitude_t *ins,float dt)//计算theta和d_theta给lqr用，同时也计算腿长L0
{
	  static float PitchR=0.0f;
	  static float PithGyroR=0.0f;
	  PitchR=0.0f-ins->Pitch;
	  PithGyroR=0.0f-ins->Gyro[0];

//虚拟四连杆解算	
	  vmc->YD = vmc->l4*arm_sin_f32(vmc->phi4);//D的y坐标
	  vmc->YB = vmc->l1*arm_sin_f32(vmc->phi1);//B的y坐标
	  vmc->XD = vmc->l5 + vmc->l4*arm_cos_f32(vmc->phi4);//D的x坐标
	  vmc->XB = vmc->l1*arm_cos_f32(vmc->phi1); //B的x坐标			
	  vmc->lBD = sqrt((vmc->XD - vmc->XB)*(vmc->XD - vmc->XB) + (vmc->YD -vmc-> YB)*(vmc->YD - vmc->YB));
	
	  vmc->A0 = 2*vmc->l2*(vmc->XD - vmc->XB);
	  vmc->B0 = 2*vmc->l2*(vmc->YD - vmc->YB);
	  vmc->C0 = vmc->l2*vmc->l2 + vmc->lBD*vmc->lBD - vmc->l3*vmc->l3;
	  vmc->phi2 = 2*atan2f((vmc->B0 + sqrt(vmc->A0*vmc->A0 + vmc->B0*vmc->B0 - vmc->C0*vmc->C0)),vmc->A0 + vmc->C0);			
	  vmc->phi3 = atan2f(vmc->YB-vmc->YD+vmc->l2*arm_sin_f32(vmc->phi2),vmc->XB-vmc->XD+vmc->l2*arm_cos_f32(vmc->phi2));
	  //C点直角坐标
	  vmc->XC = vmc->l1*arm_cos_f32(vmc->phi1) + vmc->l2*arm_cos_f32(vmc->phi2);
	  vmc->YC = vmc->l1*arm_sin_f32(vmc->phi1) + vmc->l2*arm_sin_f32(vmc->phi2);
	  //C点极坐标
	  vmc->L0 = sqrt((vmc->XC - vmc->l5/2.0f)*(vmc->XC - vmc->l5/2.0f) + vmc->YC*vmc->YC);
	  vmc->phi0 = atan2f(vmc->YC,(vmc->XC - vmc->l5/2.0f));//phi0用于计算lqr需要的theta,偏差补偿
      vmc->alpha=pi/2.0f-vmc->phi0;
		
		if(vmc->first_flag==0)
		{
			vmc->last_phi0=vmc->phi0;
			vmc->theta_prev=(vmc->phi0-pi/2.0f-PitchR);
			vmc->theta_accum=vmc->theta_prev;
			vmc->first_flag=1;
		}
		
		// 角度差连续化处理，避免 ±π 翻转
		float delta_phi0 = vmc->phi0 - vmc->last_phi0;
		if(delta_phi0 > pi) {
			delta_phi0 -= 2.0f * pi;
		} else if(delta_phi0 < -pi) {
			delta_phi0 += 2.0f * pi;
		}
		vmc->d_phi0 = delta_phi0 / dt;//计算phi0变化率，d_phi0用于计算lqr需要的d_theta
		vmc->d_alpha=0.0f-vmc->d_phi0;
		
		vmc->theta=(vmc->phi0-pi/2.0f-PitchR);//得到状态变量1
	
	// theta 跳变补偿
	if(vmc->first_flag==1) {
		float delta_theta = vmc->theta - vmc->theta_prev;
		if(delta_theta > pi) {
			delta_theta -= 2.0f * pi;
		} else if(delta_theta < -pi) {
			delta_theta += 2.0f * pi;
		}
		vmc->theta_accum += delta_theta;
		vmc->theta_prev = vmc->theta;
	}
	
	    vmc->d_theta=(vmc->d_phi0+PithGyroR);//得到状态变量2
		
		vmc->last_phi0=vmc->phi0;
    
		vmc->d_L0=(vmc->L0-vmc->last_L0)/dt;//腿长L0的一阶导数
        vmc->dd_L0=(vmc->d_L0-vmc->last_d_L0)/dt;//腿长L0的二阶导数
		
		vmc->last_d_L0=vmc->d_L0;
		vmc->last_L0=vmc->L0;
		
		vmc->dd_theta=(vmc->d_theta-vmc->last_d_theta)/dt;
		vmc->last_d_theta=vmc->d_theta;
}

//计算theta和d_theta给lqr用，同时也计算腿长L0
void VMC_calc_left(vmc_leg_t *vmc,attitude_t *ins,float dt)//计算theta和d_theta给lqr用，同时也计算腿长L0
{		
	  static float PitchL=0.0f;
	  static float PithGyroL=0.0f;
	  PitchL=0.0f-ins->Pitch;
	  PithGyroL=0.0f-ins->Gyro[0];

//虚拟四连杆解算
	  vmc->YD = vmc->l4*arm_sin_f32(vmc->phi4);//D的y坐标
	  vmc->YB = vmc->l1*arm_sin_f32(vmc->phi1);//B的y坐标
	  vmc->XD = vmc->l4*arm_cos_f32(vmc->phi4);//D的x坐标
	  vmc->XB = vmc->l1*arm_cos_f32(vmc->phi1); //B的x坐标
			
	  vmc->lBD = sqrt((vmc->XD - vmc->XB)*(vmc->XD - vmc->XB) + (vmc->YD -vmc-> YB)*(vmc->YD - vmc->YB));
	
	  vmc->A0 = 2*vmc->l2*(vmc->XD - vmc->XB);
	  vmc->B0 = 2*vmc->l2*(vmc->YD - vmc->YB);
	  vmc->C0 = vmc->l2*vmc->l2 + vmc->lBD*vmc->lBD - vmc->l3*vmc->l3;
	  vmc->phi2 = 2*atan2f((vmc->B0 + sqrt(vmc->A0*vmc->A0 + vmc->B0*vmc->B0 - vmc->C0*vmc->C0)),vmc->A0 + vmc->C0);			
	  vmc->phi3 = atan2f(vmc->YB-vmc->YD+vmc->l2*arm_sin_f32(vmc->phi2),vmc->XB-vmc->XD+vmc->l2*arm_cos_f32(vmc->phi2));
//C点直角坐标
	  vmc->XC = vmc->l1*arm_cos_f32(vmc->phi1) + vmc->l2*arm_cos_f32(vmc->phi2);
	  vmc->YC = vmc->l1*arm_sin_f32(vmc->phi1) + vmc->l2*arm_sin_f32(vmc->phi2);
//C点极坐标
	  vmc->L0 = sqrt((vmc->XC)*(vmc->XC) + (vmc->YC)*(vmc->YC));		
	  vmc->phi0 = atan2f(vmc->YC,vmc->XC);//phi0用于计算lqr需要的theta		
	  vmc->alpha=pi/2.0f-vmc->phi0;

		if(vmc->first_flag==0)
		{
			vmc->last_phi0=vmc->phi0;
			vmc->theta_prev=(vmc->phi0-pi/2.0f-PitchL);
			vmc->theta_accum=vmc->theta_prev;
			vmc->first_flag=1;
		}
		
// 角度差连续化处理，避免 ±π 翻转
		float delta_phi0 = vmc->phi0 - vmc->last_phi0;
		if(delta_phi0 > pi) {
			delta_phi0 -= 2.0f * pi;
		} else if(delta_phi0 < -pi) {
			delta_phi0 += 2.0f * pi;
		}
		vmc->d_phi0 = delta_phi0 / dt;//计算phi0变化率，d_phi0用于计算lqr需要的d_theta
		vmc->d_alpha=0.0f-vmc->d_phi0;
		
		vmc->theta=(vmc->phi0-pi/2.0f-PitchL);//得到状态变量1
	
// theta 跳变补偿
	if(vmc->first_flag==1) {
		float delta_theta = vmc->theta - vmc->theta_prev;
		if(delta_theta > pi) {
			delta_theta -= 2.0f * pi;
		} else if(delta_theta < -pi) {
			delta_theta += 2.0f * pi;
		}
		vmc->theta_accum += delta_theta;
		vmc->theta_prev = vmc->theta;
	}
	
	vmc->d_theta=(PithGyroL+vmc->d_phi0);//得到状态变量2
		
		vmc->last_phi0=vmc->phi0;

		vmc->d_L0=(vmc->L0-vmc->last_L0)/dt;//腿长L0的一阶导数
        vmc->dd_L0=(vmc->d_L0-vmc->last_d_L0)/dt;//腿长L0的二阶导数
		
		vmc->last_d_L0=vmc->d_L0;
		vmc->last_L0=vmc->L0;
		
		vmc->dd_theta=(vmc->d_theta-vmc->last_d_theta)/dt;
		vmc->last_d_theta=vmc->d_theta;
}


void VMC_calc_splitter(vmc_leg_t *vmc)//计算期望的关节输出力矩
{
//虚拟四连杆解算	
	float sin_phi3_2 = arm_sin_f32(vmc->phi3 - vmc->phi2);
	
	// 防止除以零：检查分母是否过小
	if ((sin_phi3_2 > -0.01f && sin_phi3_2 < 0.01f) || vmc->L0 < 0.05f) {
		// 奇异姿态，输出零力矩
		vmc->torque_set[0] = 0.0f;
		vmc->torque_set[1] = 0.0f;
		return;
	}
	vmc->j11 = (vmc->l1*arm_sin_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/sin_phi3_2;
	vmc->j12 = (vmc->l1*arm_cos_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/(vmc->L0*sin_phi3_2);
	vmc->j21 = (vmc->l4*arm_sin_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/sin_phi3_2;
	vmc->j22 = (vmc->l4*arm_cos_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/(vmc->L0*sin_phi3_2);

	vmc->torque_set[0]=vmc->j11*vmc->F0+vmc->j12*vmc->Tp;//得到RightFront的输出轴期望力矩，F0为四连杆机构末端沿腿的推力 
	vmc->torque_set[1]=vmc->j21*vmc->F0+vmc->j22*vmc->Tp;//得到RightBack的输出轴期望力矩，Tp为沿中心轴的力矩 
}

//右腿离地检测
float averr[4]={0.0f};
float aver_fnr=0.0f;
uint8_t Ground_detectionR(vmc_leg_t *vmc,attitude_t *ins)
{
	vmc->FN = VMC_CalcNormalForce(vmc);

	averr[0]=averr[1];
	averr[1]=averr[2];
	averr[2]=averr[3];
	averr[3]=vmc->FN;
	aver_fnr=0.25f*averr[0]+0.25f*averr[1]+0.25f*averr[2]+0.25f*averr[3];//对支持力进行均值滤波
	
	return VMC_IsGroundLost(aver_fnr, vmc, ins);
}

//左腿离地检测
float averl[4]={0.0f};
float aver_fnl=0.0f;
uint8_t Ground_detectionL(vmc_leg_t *vmc,attitude_t *ins)
{
	vmc->FN = VMC_CalcNormalForce(vmc);
	averl[0]=averl[1];
	averl[1]=averl[2];
	averl[2]=averl[3];
	averl[3]=vmc->FN;
	
	aver_fnl=0.25f*averl[0]+0.25f*averl[1]+0.25f*averl[2]+0.25f*averl[3];//对支持力进行均值滤波
	
	return VMC_IsGroundLost(aver_fnl, vmc, ins);
}

//K矩阵计算
float LQR_K_calc(float *coe,float len)
{   
  return coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3];
}
