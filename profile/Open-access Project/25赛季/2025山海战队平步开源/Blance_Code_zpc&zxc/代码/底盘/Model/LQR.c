#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Observer.h"
#include "Chassis.h"
#include "Parameter.h"


#define ChangeK		0
#define MPC

//倒地自起LQR的K阵

#ifndef ChangeK
	//二阶倒立摆LQR的K阵
//	float K2[12]={	-27.2160f,	-3.2660f,	-9.8674f,	-8.8360f,	31.6798f,	3.9758f,
//48.2697f,	7.3746f,	30.4792f,	25.0840f,	101.7113f,	4.8301f};//摆长0.2
	float K2[12]={	-25.4979f,	-3.0107f,	-8.6225f,	-7.8582f,	23.1920f,	3.3814f,
53.4494f-0.0029f,	8.1997f-0.7132f,	34.3828f+0.0368f,	28.1270f+1.7813f,	54.9262f-0.9588f,	2.4037f-0.2754f};//摆长0.2
	float MPC[12]
#else
	float K2_p11[4]={20.4392,94.2238,-155.5329,1.7024};
	float K2_p12[4]={55.9817,-50.4603,-8.3689,0.2381};
	float K2_p13[4]={-181.8475,219.3275,-96.9871,3.4664};
	float K2_p14[4]={-103.5657,138.6344,-72.4948,1.9338};
	float K2_p15[4]={275.7576,-157.1632,-21.7816,31.6595};
	float K2_p16[4]={27.8745,-17.9068,-1.3970,4.1551};
	float K2_p21[4]={1497.2384,-1401.4599,420.1510,13.5622};
	float K2_p22[4]={111.2058,-131.8536,59.7382,0.6340};
	float K2_p23[4]={431.5558,-236.0704,-41.0164,48.6301};
	float K2_p24[4]={316.9914,-189.6058,-18.4850,36.9037};
	float K2_p25[4]={1343.4311,-1608.5975,710.8411,-33.7040};
	float K2_p26[4]={153.3062,-189.6469,88.1234,-8.8669};
	#ifdef MPC
		float MPC_p1[4]={27.4806,-13.4896,2.9985,-2.2173};
		float MPC_p2[4]={6.4265,-6.3638,0.0000,28.2315};
		float MPC_p3[4]={-0.2467,-2.5364,0.2166,-35.1521};
		float MPC_p4[4]={-8.5042,-2.0736,61.3301,-23.3387};
		float MPC_p5[4]={8.1575,-20.9040,4.0762,-0.2526};
		float MPC_p6[4]={1.2747,-20.8108,0.6807,-0.2444};
		float MPC_p7[4]={0.0315,-20.1382,0.0067,-5.2262};
		float MPC_p8[4]={2.5547,-6.3532,0.0000,29.1197};
		float MPC_p9[4]={-0.0626,-2.6442,0.0012,-10.8676};
		float MPC_p10[4]={-2.9758,-2.2621,12.8894,-23.6228};
		float MPC_p11[4]={1.9242,-21.0055,0.9766,-0.2494};
		float MPC_p12[4]={0.5190,-20.7693,0.2803,-0.2432};
	#endif
#endif

/*
 *函数简介:LQR计算
 *参数说明:左轮T
 *参数说明:左腿Tp
 *参数说明:右轮T
 *参数说明:右腿Tp
 *参数说明:目标位移x
 *参数说明:目标速度dx
 *返回类型:无
 *备注:无
 */
			float K2_l[12],K2_r[12],MPC_l[12],MPC_r[12];
void LQR_Clc(float *Tl,float *Tpl,float *Tr,float *Tpr,float target_x,float target_dx)
{
	{
		#ifndef ChangeK
			float *K;
			K=K2;
		
			if(Observer_BalanceStatus.LeftLeg.FN<FN_Threshold)
			{
				(*Tl)=0;
				(*Tpl)=K[6]*Observer_BalanceStatus.LeftLeg.theta+K[7]*Observer_BalanceStatus.LeftLeg.dtheta;
			}
			else
			{
				(*Tl)=K[0]*Observer_BalanceStatus.LeftLeg.theta+K[1]*Observer_BalanceStatus.LeftLeg.dtheta+K[2]*(Observer_BalanceStatus.Body.x-target_x)+K[3]*(Observer_BalanceStatus.Body.dx-target_dx)+K[4]*Observer_BalanceStatus.Body.Pitch+K[5]*Observer_BalanceStatus.Body.dPitch;
				(*Tpl)=K[6]*Observer_BalanceStatus.LeftLeg.theta+K[7]*Observer_BalanceStatus.LeftLeg.dtheta+K[8]*(Observer_BalanceStatus.Body.x-target_x)+K[9]*(Observer_BalanceStatus.Body.dx-target_dx)+K[10]*Observer_BalanceStatus.Body.Pitch+K[11]*Observer_BalanceStatus.Body.dPitch;
			}
			
			if(Observer_BalanceStatus.RightLeg.FN<FN_Threshold)
			{
				(*Tr)=0;
				(*Tpr)=K[6]*Observer_BalanceStatus.RightLeg.theta+K[7]*Observer_BalanceStatus.RightLeg.dtheta;
			}
			else
			{
				(*Tr)=K[0]*Observer_BalanceStatus.RightLeg.theta+K[1]*Observer_BalanceStatus.RightLeg.dtheta+K[2]*(Observer_BalanceStatus.Body.x-target_x)+K[3]*(Observer_BalanceStatus.Body.dx-target_dx)+K[4]*Observer_BalanceStatus.Body.Pitch+K[5]*Observer_BalanceStatus.Body.dPitch;
				(*Tpr)=K[6]*Observer_BalanceStatus.RightLeg.theta+K[7]*Observer_BalanceStatus.RightLeg.dtheta+K[8]*(Observer_BalanceStatus.Body.x-target_x)+K[9]*(Observer_BalanceStatus.Body.dx-target_dx)+K[10]*Observer_BalanceStatus.Body.Pitch+K[11]*Observer_BalanceStatus.Body.dPitch;
			}
		#else
			float L0_l=Observer_BalanceStatus.LeftLeg.L_0,L0_r=Observer_BalanceStatus.RightLeg.L_0;
			float Delta_Tp_l=0,Delta_Tp_r=0;
			
			K2_l[0]=K2_p11[0]*L0_l*L0_l*L0_l+K2_p11[1]*L0_l*L0_l+K2_p11[2]*L0_l+K2_p11[3];
			K2_l[1]=K2_p12[0]*L0_l*L0_l*L0_l+K2_p12[1]*L0_l*L0_l+K2_p12[2]*L0_l+K2_p12[3];
			K2_l[2]=K2_p13[0]*L0_l*L0_l*L0_l+K2_p13[1]*L0_l*L0_l+K2_p13[2]*L0_l+K2_p13[3];
			K2_l[3]=K2_p14[0]*L0_l*L0_l*L0_l+K2_p14[1]*L0_l*L0_l+K2_p14[2]*L0_l+K2_p14[3];
			K2_l[4]=K2_p15[0]*L0_l*L0_l*L0_l+K2_p15[1]*L0_l*L0_l+K2_p15[2]*L0_l+K2_p15[3];
			K2_l[5]=K2_p16[0]*L0_l*L0_l*L0_l+K2_p16[1]*L0_l*L0_l+K2_p16[2]*L0_l+K2_p16[3];
			K2_l[6]=K2_p21[0]*L0_l*L0_l*L0_l+K2_p21[1]*L0_l*L0_l+K2_p21[2]*L0_l+K2_p21[3];
			K2_l[7]=K2_p22[0]*L0_l*L0_l*L0_l+K2_p22[1]*L0_l*L0_l+K2_p22[2]*L0_l+K2_p22[3];
			K2_l[8]=K2_p23[0]*L0_l*L0_l*L0_l+K2_p23[1]*L0_l*L0_l+K2_p23[2]*L0_l+K2_p23[3];
			K2_l[9]=K2_p24[0]*L0_l*L0_l*L0_l+K2_p24[1]*L0_l*L0_l+K2_p24[2]*L0_l+K2_p24[3];
			K2_l[10]=K2_p25[0]*L0_l*L0_l*L0_l+K2_p25[1]*L0_l*L0_l+K2_p25[2]*L0_l+K2_p25[3];
			K2_l[11]=K2_p26[0]*L0_l*L0_l*L0_l+K2_p26[1]*L0_l*L0_l+K2_p26[2]*L0_l+K2_p26[3];

			K2_r[0]=K2_p11[0]*L0_r*L0_r*L0_r+K2_p11[1]*L0_r*L0_r+K2_p11[2]*L0_r+K2_p11[3];
			K2_r[1]=K2_p12[0]*L0_r*L0_r*L0_r+K2_p12[1]*L0_r*L0_r+K2_p12[2]*L0_r+K2_p12[3];
			K2_r[2]=K2_p13[0]*L0_r*L0_r*L0_r+K2_p13[1]*L0_r*L0_r+K2_p13[2]*L0_r+K2_p13[3];
			K2_r[3]=K2_p14[0]*L0_r*L0_r*L0_r+K2_p14[1]*L0_r*L0_r+K2_p14[2]*L0_r+K2_p14[3];
			K2_r[4]=K2_p15[0]*L0_r*L0_r*L0_r+K2_p15[1]*L0_r*L0_r+K2_p15[2]*L0_r+K2_p15[3];
			K2_r[5]=K2_p16[0]*L0_r*L0_r*L0_r+K2_p16[1]*L0_r*L0_r+K2_p16[2]*L0_r+K2_p16[3];
			K2_r[6]=K2_p21[0]*L0_r*L0_r*L0_r+K2_p21[1]*L0_r*L0_r+K2_p21[2]*L0_r+K2_p21[3];
			K2_r[7]=K2_p22[0]*L0_r*L0_r*L0_r+K2_p22[1]*L0_r*L0_r+K2_p22[2]*L0_r+K2_p22[3];
			K2_r[8]=K2_p23[0]*L0_r*L0_r*L0_r+K2_p23[1]*L0_r*L0_r+K2_p23[2]*L0_r+K2_p23[3];
			K2_r[9]=K2_p24[0]*L0_r*L0_r*L0_r+K2_p24[1]*L0_r*L0_r+K2_p24[2]*L0_r+K2_p24[3];
			K2_r[10]=K2_p25[0]*L0_r*L0_r*L0_r+K2_p25[1]*L0_r*L0_r+K2_p25[2]*L0_r+K2_p25[3];
			K2_r[11]=K2_p26[0]*L0_r*L0_r*L0_r+K2_p26[1]*L0_r*L0_r+K2_p26[2]*L0_r+K2_p26[3];
			
			#ifdef MPC
				static float Last_theta_l,Last_theta_r,Last_dtheta_l,Last_dtheta_r;
				static float Last_x,Last_dx,Last_Pitch,Last_dPitch;
			
				MPC_l[0]=MPC_p1[0]*expf(MPC_p1[1]*L0_l)+MPC_p1[2]*expf(MPC_p1[3]*L0_l);
				MPC_l[1]=MPC_p2[0]*expf(MPC_p2[1]*L0_l)+MPC_p2[2]*expf(MPC_p2[3]*L0_l);
				MPC_l[2]=MPC_p3[0]*expf(MPC_p3[1]*L0_l)+MPC_p3[2]*expf(MPC_p3[3]*L0_l);
				MPC_l[3]=MPC_p4[0]*expf(MPC_p4[1]*L0_l)+MPC_p4[2]*expf(MPC_p4[3]*L0_l);
				MPC_l[4]=MPC_p5[0]*expf(MPC_p5[1]*L0_l)+MPC_p5[2]*expf(MPC_p5[3]*L0_l);
				MPC_l[5]=MPC_p6[0]*expf(MPC_p6[1]*L0_l)+MPC_p6[2]*expf(MPC_p6[3]*L0_l);
				MPC_l[6]=MPC_p7[0]*expf(MPC_p7[1]*L0_l)+MPC_p7[2]*expf(MPC_p7[3]*L0_l);
				MPC_l[7]=MPC_p8[0]*expf(MPC_p8[1]*L0_l)+MPC_p8[2]*expf(MPC_p8[3]*L0_l);
				MPC_l[8]=MPC_p9[0]*expf(MPC_p9[1]*L0_l)+MPC_p9[2]*expf(MPC_p9[3]*L0_l);
				MPC_l[9]=MPC_p10[0]*expf(MPC_p10[1]*L0_l)+MPC_p10[2]*expf(MPC_p10[3]*L0_l);
				MPC_l[10]=MPC_p11[0]*expf(MPC_p11[1]*L0_l)+MPC_p11[2]*expf(MPC_p11[3]*L0_l);
				MPC_l[11]=MPC_p12[0]*expf(MPC_p12[1]*L0_l)+MPC_p12[2]*expf(MPC_p12[3]*L0_l);

				MPC_r[0]=MPC_p1[0]*expf(MPC_p1[1]*L0_r)+MPC_p1[2]*expf(MPC_p1[3]*L0_r);
				MPC_r[1]=MPC_p2[0]*expf(MPC_p2[1]*L0_r)+MPC_p2[2]*expf(MPC_p2[3]*L0_r);
				MPC_r[2]=MPC_p3[0]*expf(MPC_p3[1]*L0_r)+MPC_p3[2]*expf(MPC_p3[3]*L0_r);
				MPC_r[3]=MPC_p4[0]*expf(MPC_p4[1]*L0_r)+MPC_p4[2]*expf(MPC_p4[3]*L0_r);
				MPC_r[4]=MPC_p5[0]*expf(MPC_p5[1]*L0_r)+MPC_p5[2]*expf(MPC_p5[3]*L0_r);
				MPC_r[5]=MPC_p6[0]*expf(MPC_p6[1]*L0_r)+MPC_p6[2]*expf(MPC_p6[3]*L0_r);
				MPC_r[6]=MPC_p7[0]*expf(MPC_p7[1]*L0_r)+MPC_p7[2]*expf(MPC_p7[3]*L0_r);
				MPC_r[7]=MPC_p8[0]*expf(MPC_p8[1]*L0_r)+MPC_p8[2]*expf(MPC_p8[3]*L0_r);
				MPC_r[8]=MPC_p9[0]*expf(MPC_p9[1]*L0_r)+MPC_p9[2]*expf(MPC_p9[3]*L0_r);
				MPC_r[9]=MPC_p10[0]*expf(MPC_p10[1]*L0_r)+MPC_p10[2]*expf(MPC_p10[3]*L0_r);
				MPC_r[10]=MPC_p11[0]*expf(MPC_p11[1]*L0_r)+MPC_p11[2]*expf(MPC_p11[3]*L0_r);
				MPC_r[11]=MPC_p12[0]*expf(MPC_p12[1]*L0_r)+MPC_p12[2]*expf(MPC_p12[3]*L0_r);
				
				K2_l[6]=K2_l[6]-MPC_l[6];
				K2_l[7]=K2_l[7]-MPC_l[7];
				K2_l[8]=K2_l[8]-MPC_l[8];
				K2_l[9]=K2_l[9]-MPC_l[9];
				K2_l[10]=K2_l[10]-MPC_l[10];
				K2_l[11]=K2_l[11]-MPC_l[11];

				K2_r[6]=K2_r[6]-MPC_r[6];
				K2_r[7]=K2_r[7]-MPC_r[7];
				K2_r[8]=K2_r[8]-MPC_r[8];
				K2_r[9]=K2_r[9]-MPC_r[9];
				K2_r[10]=K2_r[10]-MPC_r[10];
				K2_r[11]=K2_r[11]-MPC_r[11];

				float Delta_theta_l=Observer_BalanceStatus.LeftLeg.theta-Last_theta_l;
				float Delta_theta_r=Observer_BalanceStatus.RightLeg.theta-Last_theta_r;
				float Delta_dtheta_l=Observer_BalanceStatus.LeftLeg.dtheta-Last_dtheta_l;
				float Delta_dtheta_r=Observer_BalanceStatus.RightLeg.dtheta-Last_dtheta_r;
				float Delta_x=Observer_BalanceStatus.Body.x-Last_x;
				float Delta_dx=Observer_BalanceStatus.Body.dx-Last_dx;
				float Delta_Pitch=Observer_BalanceStatus.Body.Pitch-Last_Pitch;
				float Delta_dPitch=Observer_BalanceStatus.Body.dPitch-Last_dPitch;
				Delta_Tp_l=MPC_l[0]*Delta_theta_l+MPC_l[1]*Delta_dtheta_l+MPC_l[2]*Delta_x+MPC_l[3]*Delta_dx+MPC_l[4]*Delta_Pitch+MPC_l[5]*Delta_dPitch;
				Delta_Tp_r=MPC_r[0]*Delta_theta_r+MPC_r[1]*Delta_dtheta_r+MPC_r[2]*Delta_x+MPC_r[3]*Delta_dx+MPC_r[4]*Delta_Pitch+MPC_r[5]*Delta_dPitch;
				
				Last_theta_l=Observer_BalanceStatus.LeftLeg.theta;
				Last_theta_r=Observer_BalanceStatus.RightLeg.theta;
				Last_dtheta_l=Observer_BalanceStatus.LeftLeg.dtheta;
				Last_dtheta_r=Observer_BalanceStatus.RightLeg.dtheta;
				Last_x=Observer_BalanceStatus.Body.x;
				Last_dx=Observer_BalanceStatus.Body.dx;
				Last_Pitch=Observer_BalanceStatus.Body.Pitch;
				Last_dPitch=Observer_BalanceStatus.Body.dPitch;
			#endif
			
			if(Observer_BalanceStatus.LeftLeg.FN<FN_Threshold)
			{
				(*Tl)=0;
				(*Tpl)=K2_l[6]*Observer_BalanceStatus.LeftLeg.theta+K2_l[7]*Observer_BalanceStatus.LeftLeg.dtheta;
			}
			else
			{
				(*Tl)=K2_l[0]*Observer_BalanceStatus.LeftLeg.theta+K2_l[1]*Observer_BalanceStatus.LeftLeg.dtheta+K2_l[2]*(Observer_BalanceStatus.Body.x-target_x)+K2_l[3]*(Observer_BalanceStatus.Body.dx-target_dx)+K2_l[4]*Observer_BalanceStatus.Body.Pitch+K2_l[5]*Observer_BalanceStatus.Body.dPitch;
				(*Tpl)=K2_l[6]*Observer_BalanceStatus.LeftLeg.theta+K2_l[7]*Observer_BalanceStatus.LeftLeg.dtheta+K2_l[8]*(Observer_BalanceStatus.Body.x-target_x)+K2_l[9]*(Observer_BalanceStatus.Body.dx-target_dx)+K2_l[10]*Observer_BalanceStatus.Body.Pitch+K2_l[11]*Observer_BalanceStatus.Body.dPitch;
				(*Tpl)=(*Tpl)-Delta_Tp_l;
			}
			
			if(Observer_BalanceStatus.RightLeg.FN<FN_Threshold)
			{
				(*Tr)=0;
				(*Tpr)=K2_r[6]*Observer_BalanceStatus.RightLeg.theta+K2_r[7]*Observer_BalanceStatus.RightLeg.dtheta;
			}
			else
			{
				(*Tr)=K2_r[0]*Observer_BalanceStatus.RightLeg.theta+K2_r[1]*Observer_BalanceStatus.RightLeg.dtheta+K2_r[2]*(Observer_BalanceStatus.Body.x-target_x)+K2_r[3]*(Observer_BalanceStatus.Body.dx-target_dx)+K2_r[4]*Observer_BalanceStatus.Body.Pitch+K2_r[5]*Observer_BalanceStatus.Body.dPitch;
				(*Tpr)=K2_r[6]*Observer_BalanceStatus.RightLeg.theta+K2_r[7]*Observer_BalanceStatus.RightLeg.dtheta+K2_r[8]*(Observer_BalanceStatus.Body.x-target_x)+K2_r[9]*(Observer_BalanceStatus.Body.dx-target_dx)+K2_r[10]*Observer_BalanceStatus.Body.Pitch+K2_r[11]*Observer_BalanceStatus.Body.dPitch;
				(*Tpr)=(*Tpr)-Delta_Tp_r;
			}
		#endif
	}
}
