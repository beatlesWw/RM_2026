#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include "RM_C.h"

//#define SingleChassis		0//注释后Yaw跟云台同步

int main(void)
{
	#ifdef SingleChassis
		//Lock_Init();//安全锁初始化
		Warming_Init();//报警初始化
		LED_BON();//蓝灯点亮表示代码在运行
//		AttitudeAlgorithms_Init();//姿态解算初始化
		Delay_s(1);//延时,等待校准和模块启动
//		LinkCheck_Init();//连接检测初始化
//		RefereeSystem_Init();//图传链路初始化
//		Visual_Init();//视觉初始化
//		CloseLoopControl_Init();//闭环控制初始化
		CAN_CANInit();
		Remote_Init();//遥控器初始化
	#else
		//Lock_Init();//安全锁初始化
		Warming_Init();//报警初始化21.86 -14.95
		LED_BON();//蓝灯点亮表示代码在运行
		AttitudeAlgorithms_Init();//姿态解算初始化
		Delay_ms(500);//延时,等待校准和模块启动
		LinkCheck_Init();//连接检测初始化
		RefereeSystem_Init();//图传链路初始化
		Visual_Init();//视觉初始化
		CloseLoopControl_Init();//闭环控制初始化
		Remote_Init();//遥控器初始化
		
		//UART2_Init();
	#endif
	
	uint16_t Count=0;
	
	while(1)
	{
		Count++;
		IWDG_ReloadCounter();//喂狗
		
		CToC_MasterSendControl();//CToC发送遥控器控制数据
		Delay_us(200);
		CToC_MasterSendData();//CToC发送遥控器摇杆数据
		Delay_us(200);
		CToC_MasterSendKnobData();//CToC发送遥控器旋钮数据
		
		if(Count%10==0)
		{
			//printf("%d\n",GM6020_MotorStatus[0].Angle);
		}
		
		if(Count==500)
		{
			Count=0;
			LED_BTurn();
		}
		Delay_us(600);//CToC周期1ms
	}
}
