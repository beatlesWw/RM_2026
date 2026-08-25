#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "UART.h"
#include "Remote.h"

/*
 *函数简介:键盘初始化
 *参数说明:无
 *返回类型:无
 *备注:初始化UART1(USART6)
 */
void Keyboard_Init(void)
{
	UART1_Init();
}

/*
 *函数简介:键盘数据处理
 *参数说明:接收数据
 *返回类型:无
 *备注:无
 */
void Keyboard_DataProcess(uint8_t *Data)
{
	Remote_RxData.Remote_KeyLast_Q=Remote_RxData.Remote_Key_Q;
	Remote_RxData.Remote_KeyLast_G=Remote_RxData.Remote_Key_G;
	Remote_RxData.Remote_KeyLast_Z=Remote_RxData.Remote_Key_Z;
	Remote_RxData.Remote_KeyLast_B=Remote_RxData.Remote_Key_B;
	Remote_RxData.Remote_KeyLast_Ctrl=Remote_RxData.Remote_Key_Ctrl;

	Remote_RxData.Remote_Mouse_RL=(int16_t)((uint16_t)Data[1]<<8 | Data[0]);
	Remote_RxData.Remote_Mouse_DU=-(int16_t)((uint16_t)Data[3]<<8 | Data[2]);
	Remote_RxData.Remote_Mouse_Wheel=(int16_t)((uint16_t)Data[5]<<8 | Data[4]);
	Remote_RxData.Remote_Mouse_KeyL=Data[6];
	Remote_RxData.Remote_Mouse_KeyR=Data[7];
	
	Remote_RxData.Remote_Key_W=Data[8] & 0x01;
	Remote_RxData.Remote_Key_S=(Data[8]>>1) & 0x01;
	Remote_RxData.Remote_Key_A=(Data[8]>>2) & 0x01;
	Remote_RxData.Remote_Key_D=(Data[8]>>3) & 0x01;
	Remote_RxData.Remote_Key_Shift=(Data[8]>>4) & 0x01;
	Remote_RxData.Remote_Key_Ctrl=(Data[8]>>5) & 0x01;
	Remote_RxData.Remote_Key_Q=(Data[8]>>6) & 0x01;
	Remote_RxData.Remote_Key_E=(Data[8]>>7) & 0x01;
	Remote_RxData.Remote_Key_R=Data[9] & 0x01;
	Remote_RxData.Remote_Key_F=(Data[9]>>1) & 0x01;
	Remote_RxData.Remote_Key_G=(Data[9]>>2) & 0x01;
	Remote_RxData.Remote_Key_Z=(Data[9]>>3) & 0x01;
	Remote_RxData.Remote_Key_X=(Data[9]>>4) & 0x01;
	Remote_RxData.Remote_Key_C=(Data[9]>>5) & 0x01;
	Remote_RxData.Remote_Key_V=(Data[9]>>6) & 0x01;
	Remote_RxData.Remote_Key_B=(Data[9]>>7) & 0x01;

	if(Remote_RxData.Remote_KeyLast_Ctrl==0 && Remote_RxData.Remote_Key_Ctrl==1)Remote_RxData.Remote_KeyPush_Ctrl=!Remote_RxData.Remote_KeyPush_Ctrl;
	if(Remote_RxData.Remote_KeyLast_Q==0 && Remote_RxData.Remote_Key_Q==1)Remote_RxData.Remote_KeyPush_Q=!Remote_RxData.Remote_KeyPush_Q;//检测是否按下
	if(Remote_RxData.Remote_KeyLast_G==0 && Remote_RxData.Remote_Key_G==1)Remote_RxData.Remote_KeyPush_G=!Remote_RxData.Remote_KeyPush_G;
	if(Remote_RxData.Remote_KeyLast_Z==0 && Remote_RxData.Remote_Key_Z==1)Remote_RxData.Remote_KeyPush_Z=!Remote_RxData.Remote_KeyPush_Z;
	if(Remote_RxData.Remote_KeyLast_B==0 && Remote_RxData.Remote_Key_B==1)Remote_RxData.Remote_KeyPush_B=!Remote_RxData.Remote_KeyPush_B;
}
