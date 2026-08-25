#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include "RM_C.h"

int main()
{
	//Lock_Init();//安全锁初始化
 	Warming_Init();//报警初始化
	LED_BON();//蓝灯点亮表示代码在运行
	AttitudeAlgorithms_Init();//姿态解算初始化
	RefereeSystem_Init();//裁判系统数据接收初始化
	LinkCheck_Init();//连接检测初始化
	Ultra_CAP_Init();//超电初始化
	Delay_ms(500);//延时,等待校准和模块启动
	CloseLoopControl_Init();//闭环控制初始化
	//UART2_Init();
	UI_Init();
	STP_Init();

	uint16_t Count=0;

	while(1)
	{
		Count++;
		//Observer_GetWheelStatus();//获取轮子的状态
		CToC_SlaveSendRefereeSystemData();//向主机发送裁判系统数据
		
		if(Count%20==0)
		{
			//printf("%f,%f\n",Leg_Controller_LeftLegControlPID.Need_Value,Observer_BalanceStatus.LeftLeg.L_0);
		}
		
		if(Count==500)
		{
			Count=0;
			LED_BTurn();
		}
		Delay_us(1000);
	}
}
