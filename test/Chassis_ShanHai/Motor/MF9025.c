#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "MF9025.h"
#include "Data.h"

#define MF9025_Receive_ID1		0xA1//MF9025电机数据接收ID1(电机状态2)

#define MF9025_TorqueConstant	0.32f//MF9025电机转矩常数0.81N·m/A

MF9025_Motor MF9025_MotorStatus[8];//MF9025电机状态数组

/*
 *函数简介:CAN1总线获取MF9025电机状态
 *参数说明:MF9025电机ID
 *返回类型:1-发送成功,0-发送失败
 *备注:发送电机数据接收ID1获取电机状态2
 *备注:默认标准格式数据帧,4字节数据段
 *备注:读取频率要高于达妙DM_J8009的读取频率,不然会覆盖,在本代码中MF9025电机读取频率1kHz,达妙DM_J8009的读取频率为500Hz
 */
uint8_t MF9025_CANGetStatus(MF9025_ID ID)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=ID;//标准标识符0x9C
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=0x9C;//命令码
	TxMessage.Data[1]=0;
	TxMessage.Data[2]=0;
	TxMessage.Data[3]=0;
	TxMessage.Data[4]=0;
	TxMessage.Data[5]=0;
	TxMessage.Data[6]=0;
	TxMessage.Data[7]=0;
	
	uint8_t mbox=CAN_Transmit(CAN1,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN1,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:CAN总线设置MF9025扭矩
 *参数说明:MF9025电机ID
 *参数说明:扭矩值
 *返回类型:1-发送成功,0-发送失败
 *备注:默认标准格式数据帧,8字节数据段
 *备注:转矩T(N·m)=电流I(A)×转矩常数(N·m/A)
 *备注:原指令是设定电流,通过一定的变换,等价于设定转矩
 *备注:发送时-2048~2048对应-16.5A~16.5A,与接收不同
 */
uint8_t MF9025_CANSetTorque(MF9025_ID ID,float Torque)
{
	Torque=Data_Clipping(Torque,-20,20);
	int16_t Current=Torque/MF9025_TorqueConstant/16.5f*2048.0f;//将转矩变换成电流(-2048~2048对应-16.5A~16.5A)
	Current=Data_Clipping(Current,-2048,2048);
	
	CanTxMsg TxMessage;
	TxMessage.StdId=ID;//ID标准标识符
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=0xA1;//命令码
	TxMessage.Data[1]=0;
	TxMessage.Data[2]=0;
	TxMessage.Data[3]=0;
	TxMessage.Data[4]=Current & 0x00FF;//电流低八位
	TxMessage.Data[5]=(Current>>8) & 0x00FF;//电流高八位
	TxMessage.Data[6]=0;
	TxMessage.Data[7]=0;
	
	uint8_t mbox=CAN_Transmit(CAN1,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN1,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:MF9025数据处理
 *参数说明:MF9025电机ID号枚举,MF9025_1~2对应ID号0x141~0x142
 *参数说明:反馈数据(8字节)
 *返回类型:无
 *备注:保存到MF9025_MotorStatus结构体数组
 *备注:MF9025转矩常数0.32N·m/A,
 *备注:接收时-2048~2048对应-33A~33A,与发送不同
 *备注:转矩T(N·m)=电流I(A)×转矩常数(N·m/A),功率P(kW)=√3*转速v(RPM)*转矩T(N·m)/9550
 */
void MF9025_CANDataProcess(MF9025_ID ID,uint8_t *Data)
{
	if(Data[0]==MF9025_Receive_ID1)//检测命令码
	{
		MF9025_MotorStatus[ID-0x141].Temperature=Data[1];//电机温度
		MF9025_MotorStatus[ID-0x141].Current=(int16_t)((((uint16_t)Data[3])<<8) | Data[2]);//转矩电流
		int16_t Speed_dsp=(int16_t)((((uint16_t)Data[5])<<8) | Data[4]);//以dsp为单位的转速
		MF9025_MotorStatus[ID-0x141].Speed=Speed_dsp*Data_Deg2Rad;//=((int16_t)((((uint16_t)Data[5])<<8) | Data[4]))/180.0f*3.141592653589793238462643383279f;//转速(单位rad/s)
		//MF9025_MotorStatus[ID-0x141].Speed=0.15f*MF9025_MotorStatus[ID-0x141].Speed+0.85f*Speed_dsp*Data_Deg2Rad;//=((int16_t)((((uint16_t)Data[5])<<8) | Data[4]))/180.0f*3.141592653589793238462643383279f;//转速(单位rad/s)
		
		uint16_t MF9025_NowAngle=(uint16_t)((((uint16_t)Data[7])<<8) | Data[6]);//本次机械角度原始数据
		if(MF9025_NowAngle-MF9025_MotorStatus[ID-0x141].Angle>4000 && MF9025_MotorStatus[ID-0x141].First_Flag==1)MF9025_MotorStatus[ID-0x141].r--;//本次机械角度原始数据和上次机械角度原始数据出现跃变
		else if(MF9025_MotorStatus[ID-0x141].Angle-MF9025_NowAngle>4000 && MF9025_MotorStatus[ID-0x141].First_Flag==1)MF9025_MotorStatus[ID-0x141].r++;
		else if(MF9025_MotorStatus[ID-0x141].First_Flag!=1)
		{
			MF9025_MotorStatus[ID-0x141].PositionCheck=MF9025_NowAngle;
			MF9025_MotorStatus[ID-0x141].First_Flag++;
		}

		MF9025_MotorStatus[ID-0x141].Angle=MF9025_NowAngle;//机械角度
		MF9025_MotorStatus[ID-0x141].Position=65536*MF9025_MotorStatus[ID-0x141].r+MF9025_NowAngle-MF9025_MotorStatus[ID-0x141].PositionCheck;//角度位置
		
		MF9025_MotorStatus[ID-0x141].Torque=MF9025_MotorStatus[ID-0x141].Current*0.008056640625f*MF9025_TorqueConstant;//=MF9025_MotorStatus[ID-0x141].Current/4096.0f*33.0f*MF9025_TorqueConstant;//转矩
		MF9025_MotorStatus[ID-0x141].Power=Speed_dsp*MF9025_MotorStatus[ID-0x141].Torque*0.03022776278479716044550517175403f;//=Data_Sqrt3*(Speed_dsp*60.0f/360.0f)*MF9025_MotorStatus[ID-0x141].Torque/9.55f;//功率
		if(MF9025_MotorStatus[ID-0x141].Power<0)MF9025_MotorStatus[ID-0x141].Power*=-1;//功率去负数
	}
}
