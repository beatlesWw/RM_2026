#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "DM_J8009.h"
#include "Delay.h"
#include "Data.h"

DM_J8009_Motor DM_J8009_MotorStatus[10];//DM_J8009电机状态数组

/*
 *函数简介:DM_J8009 CAN总线发送
 *参数说明:DM_J8009电机ID
 *参数说明:8字节数据段
 *返回类型:1-发送成功,0-发送失败
 *备注:默认标准格式数据帧,8字节数据段
 */
uint8_t DM_J8009_CANSend(DM_J8009_ID ID,uint8_t *Data)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=ID;//ID标准标识符
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=Data[0];
	TxMessage.Data[1]=Data[1];
	TxMessage.Data[2]=Data[2];
	TxMessage.Data[3]=Data[3];
	TxMessage.Data[4]=Data[4];
	TxMessage.Data[5]=Data[5];
	TxMessage.Data[6]=Data[6];
	TxMessage.Data[7]=Data[7];
	
	uint8_t mbox=CAN_Transmit(CAN1,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN1,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:DM_J8009初始化
 *参数说明:无
 *返回类型:无
 *备注:每发送两组数据,需要加200us延时
 *使能命令:0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC
 */
void DM_J8009_Init(void)
{
	uint8_t Start[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	DM_J8009_CANSend(DM_J8009_1,Start);
	DM_J8009_CANSend(DM_J8009_2,Start);
	Delay_us(200);
	DM_J8009_CANSend(DM_J8009_3,Start);
	DM_J8009_CANSend(DM_J8009_4,Start);
	Delay_us(200);
	DM_J8009_CANSend(DM_J8009_1,Start);
	DM_J8009_CANSend(DM_J8009_2,Start);
	Delay_us(200);
	DM_J8009_CANSend(DM_J8009_3,Start);
	DM_J8009_CANSend(DM_J8009_4,Start);
}

/*
 *函数简介:DM_J8009浮点数float转换成整型uint
 *参数说明:浮点数x
 *参数说明:浮点数最小值x_min
 *参数说明:浮点数最大值x_max
 *参数说明:字节长bits
 *返回类型:无
 *备注:无
 */
uint16_t DM_J8009_float_to_uint(float x,float x_min,float x_max,int bits)
{
	float span=x_max-x_min;
	float offset=x_min;
	return (uint16_t)((x-offset)/span*((float)((1<<bits)-1)));
}

/*
 *函数简介:DM_J8009整型uint转换成浮点数float
 *参数说明:整型x
 *参数说明:整型最小值x_min
 *参数说明:整型最大值x_max
 *参数说明:字节长bits
 *返回类型:无
 *备注:无
 */
float DM_J8009_uint_to_float(int x,float x_min,float x_max,int bits)
{
	float span=x_max-x_min;
	float offset=x_min;
	return (float)(((float)x)*span/((float)(1<<bits)-1)+offset);
}

/*
 *函数简介:CAN总线设置DM_J8009扭矩
 *参数说明:DM_J8009电机ID
 *参数说明:扭矩值
 *返回类型:1-发送成功,0-发送失败
 *备注:默认标准格式数据帧,8字节数据段
 *备注:发送时需要将扭矩值通过DM_J8009_float_to_uint转换成12位整型
 */
void DM_J8009_SetTorque(DM_J8009_ID ID,float Torque)
{
	Torque=Data_Clipping(Torque,DM_J8009_TMIN,DM_J8009_TMAX);
	
	uint8_t Data[8];
	uint16_t Tau=DM_J8009_float_to_uint(Torque,DM_J8009_TMIN,DM_J8009_TMAX,12);
	Data[0]=0;
    Data[1]=0;
    Data[2]=0;
    Data[3]=0;
    Data[4]=0;
    Data[5]=0;
    Data[6]=Tau>>8;//转矩值高八位
    Data[7]=Tau & 0x00FF;//转矩值低八位
	DM_J8009_CANSend(ID,Data);
}	

/*
 *函数简介:DM_J8009数据处理
 *参数说明:DM_J8009电机ID号枚举,DM_J8009_1~4对应ID号0xF1~0xF4
 *参数说明:反馈数据(8字节)
 *返回类型:无
 *备注:保存到DM_J8009_MotorStatus结构体数组
 *备注:接收时需要将接收数据通过DM_J8009_uint_to_float转换成浮点型
 */
void DM_J8009_DataProcess(DM_J8009_ID ID,uint8_t *Data)
{
	DM_J8009_MotorStatus[ID-0xF1].Status=(Data[0]>>4) & 0x0F;

	uint16_t Position_int=(int16_t)((uint16_t)Data[1]<<8 | Data[2]);
	uint16_t Speed_int=(int16_t)((uint16_t)Data[3]<<4 | Data[4]>>4);
	uint16_t Torque_int=(int16_t)((uint16_t)(Data[4] & 0x0F)<<8 | Data[5]);
	
	DM_J8009_MotorStatus[ID-0xF1].Position=DM_J8009_uint_to_float(Position_int,DM_J8009_PMIN,DM_J8009_PMAX,16);
	DM_J8009_MotorStatus[ID-0xF1].Speed=DM_J8009_uint_to_float(Speed_int,DM_J8009_VMIN,DM_J8009_VMAX,12);
	DM_J8009_MotorStatus[ID-0xF1].Torque=DM_J8009_uint_to_float(Torque_int,DM_J8009_TMIN,DM_J8009_TMAX,12);
}	
