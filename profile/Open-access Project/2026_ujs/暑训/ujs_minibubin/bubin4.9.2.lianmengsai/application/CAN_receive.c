/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receivea.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"


#include "detect_task.h"

#define P_MIN -12.56637f
#define P_MAX 12.56637f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
float PowerData[4];
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_chassis[9];
	Pitch_Motor_t pitch_motor[2];
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  superCAP_tx_message;
static uint8_t              superCAP_can_send_data[8];
static CAN_TxHeaderTypeDef  fric_tx_message;
static uint8_t              fric_can_send_data[8];
static CAN_TxHeaderTypeDef  gimbal4310_tx_message;
static uint8_t              pitch_can_send_data[2];
 CAN_TxHeaderTypeDef  dm_tx_message;
 uint8_t              dm_can_send_data[8];	
	
	
uint32_t receive_data[8];
int board_receive_data[8];

	
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}



/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/
void dm4310_fbdata(Pitch_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
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
	
	
	
	
	
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
 
  int i;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

if (hcan==&hcan1){ 
	switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
		
	 {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            detect_hook(CHASSIS_MOTOR1_TOE + i);
            break;
        }
			
		case CAN_BOARD_SEND_RECEIVE_ID:
        {
		if(rx_header.StdId==CAN_BOARD_SEND_RECEIVE_ID)
		{
		for(int i=0;i<8;i++)
		receive_data[i]=(int)rx_data[i];			
		}
            static uint8_t i = 0;
           
            break;
        }
		  case CAN_SUPERCAP_ID:
			{
				
				uint16_t* pPowerdata  = (uint16_t *) rx_data;//CAN收到的8个字节的数组
				PowerData[0] = (float)pPowerdata[0] / 100.f;//输入电压
				PowerData[1] = (float)pPowerdata[1] / 100.f;//电容电压
				PowerData[2] = (float)pPowerdata[2] / 100.f;//输入电流
				PowerData[3] = (float)pPowerdata[3] / 100.f;//设定功率
				break;
			}
		case CAN_TRIGGER_MOTOR_ID:
		{
			get_motor_measure(&motor_chassis[6], rx_data);
			detect_hook(PITCH_GIMBAL_MOTOR_TOE  + 1);
			break;
		}
		case 0x100:
        {
					board_receive_data[0]=(int)rx_data[0];
					board_receive_data[1]=(int)rx_data[1];
					board_receive_data[2]=(int)rx_data[2];
		
        
            break;
        }
        default:
        {
            break;
        }
    }
	}
else if (hcan==&hcan2)
	switch (rx_header.StdId)
	{    
		case  3:
		{   
		{
		dm4310_fbdata(&pitch_motor[1], rx_data,8);
		}
		if(pitch_motor[1].para.pos>=6.283185f&&pitch_motor[1].para.pos<=12.56637f)
										pitch_motor[1].para.pos-=6.283185f;
	    if(pitch_motor[1].para.pos<=0&&pitch_motor[1].para.pos>-6.283185f)
										pitch_motor[1].para.pos+=6.283185f;
		if(pitch_motor[1].para.pos<=-6.283185f&&pitch_motor[1].para.pos>=-12.56637f)
										pitch_motor[1].para.pos+=12.56637f;
									
	    motor_chassis[5].last_ecd=motor_chassis[5].ecd;
		motor_chassis[5].ecd=pitch_motor[1].para.pos*8191/6.283185f;
		
		break;
							
		detect_hook(PITCH_GIMBAL_MOTOR_TOE);
		break;
		}

//		case CAN_PIT_MOTOR_ID:
//        {
//            get_motor_measure(&motor_chassis[5], rx_data);
//			motor_chassis[5].speed_rpm=(motor_chassis[5].speed_rpm);
//			i=motor_chassis[5].ecd;
//            detect_hook(PITCH_GIMBAL_MOTOR_TOE);
//			break;
//        }
	
	    case CAN_FRIC_L_ID:
        case CAN_FRIC_R_ID:
	{
        static uint8_t j = 0;
        j = rx_header.StdId - CAN_FRIC_L_ID + 7;
        get_motor_measure(&motor_chassis[j], rx_data);
        detect_hook(CAN_FRIC_L_ID + j - 7);
        break;
	}
//			else if (hcan==&hcan2){
//		switch (rx_header.StdId)
//		{
//			case CAN_SUPERCAP_ID:
//			{
//				
//				uint16_t* pPowerdata  = (uint16_t *) rx_data;//CAN收到的8个字节的数组
//				PowerData[0] = (float)pPowerdata[0] / 100.f;//输入电压
//				PowerData[1] = (float)pPowerdata[1] / 100.f;//电容电压
//				PowerData[2] = (float)pPowerdata[2] / 100.f;//输入电流
//				PowerData[3] = (float)pPowerdata[3] / 100.f;//设定功率
//			}
//		}
}
		
}

void CAN_cmd_SuperCap(uint16_t powerLimit)
{
    uint32_t send_mail_box;
    superCAP_tx_message.StdId = 0x210;
    superCAP_tx_message.IDE = CAN_ID_STD;
    superCAP_tx_message.RTR = CAN_RTR_DATA;
    superCAP_tx_message.DLC = 0x02;
    superCAP_can_send_data[0] = powerLimit >> 8;
    superCAP_can_send_data[1] = powerLimit;

    HAL_CAN_AddTxMessage(&SUPERCAP_CAN, &superCAP_tx_message, superCAP_can_send_data, &send_mail_box);
}


/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
//	uint8_t data[8];  //发送的数据包

	uint32_t send_mail_box;//设置发送邮箱
	
    dm_tx_message.StdId = motor_id; //由于是MIT模式，所以mode_id = 0;  而其他模式就不是咯 
    dm_tx_message.IDE = CAN_ID_STD;//ID为标准帧
    dm_tx_message.RTR = CAN_RTR_DATA;
    dm_tx_message.DLC = 0x08;
	
	//看完MIT模式说明再来给参数
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	
	//请详细阅读MIT模式的说明，可以将这些参数理解为组合技，按照要求的搭配来进行赋值
	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);//注意限幅参数的大小
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);//扭矩不等于速度，实际二者是对立的关系，测试时不要给太大 0.5，1就行

	//这些需要参照MIT模式CAN通信协议，以及4310的发送帧格式 
	//（低位在后面哦！！）
	dm_can_send_data[0] = (pos_tmp >> 8);
	dm_can_send_data[1] = pos_tmp;
	dm_can_send_data[2] = (vel_tmp >> 4);
	dm_can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	dm_can_send_data[4] = kp_tmp;
	dm_can_send_data[5] = (kd_tmp >> 4);
	dm_can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	dm_can_send_data[7] = tor_tmp;
	
	
	//CAN的发送函数
    HAL_CAN_AddTxMessage(hcan, &dm_tx_message, dm_can_send_data, &send_mail_box);

}



/**
  * @brief          发送4310电机控制使能 位置速度
  * @param[in]      motor1: () 4310电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
//4310使能
void init_gimbalpitch()
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x01;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
	
    gimbal_can_send_data[0] = 0xFF;
    gimbal_can_send_data[1] = 0xFF;
	  gimbal_can_send_data[2] = 0xFF;
    gimbal_can_send_data[3] = 0xFF;
    gimbal_can_send_data[4] = 0xFF;
    gimbal_can_send_data[5] = 0xFF;
    gimbal_can_send_data[6] = 0xFF;
    gimbal_can_send_data[7] = 0xFC;
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}  





/**
  * @brief          发送4310电机控制电流 一拖四模式
  * @param[in]      motor1: () 4310电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_4310pitch(int16_t pitch)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_4310_CONTROL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (pitch >> 8);
    gimbal_can_send_data[1] = pitch;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


//位置速度4310
void CAN_cmd_4310pitch_pvmode(float pos,float vel)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x101;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
	
		uint8_t *pbuf,*vbuf;
		pbuf=(uint8_t*)&pos;
		vbuf=(uint8_t*)&vel;
    gimbal_can_send_data[0] = *pbuf;
    gimbal_can_send_data[1] = *(pbuf+1);
	  gimbal_can_send_data[2] = *(pbuf+2);
    gimbal_can_send_data[3] = *(pbuf+3);
    gimbal_can_send_data[4] = *vbuf;
    gimbal_can_send_data[5] = *(vbuf+1);
    gimbal_can_send_data[6] = *(vbuf+2);
    gimbal_can_send_data[7] = *(vbuf+3);
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}  



//  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
//  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
//  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
//  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
//  * @param[in]      rev: (0x208) 保留，电机控制电流
//  * @retval         none
//  */
void CAN_cmd_gimbal(int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
	void CAN_cmd_gimbalyaw(int16_t yaw)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}  

//    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
//    gimbal_tx_message.IDE = CAN_ID_STD;
//    gimbal_tx_message.RTR = CAN_RTR_DATA;
//    gimbal_tx_message.DLC = 0x08;
//    gimbal_can_send_data[0] = (yaw >> 8);
//    gimbal_can_send_data[1] = yaw;
//    gimbal_can_send_data[2] = (pitch >> 8);
//    gimbal_can_send_data[3] = pitch;
//    gimbal_can_send_data[4] = (shoot >> 8);
//    gimbal_can_send_data[5] = shoot;
//    gimbal_can_send_data[6] = (rev >> 8);
//    gimbal_can_send_data[7] = rev;
//    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
//}
/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          发送摩擦轮电机控制电流(0x201,0x202)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */


void CAN_cmd_fric(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  fric_tx_message.StdId = CAN_FRIC_ALL_ID;
  fric_tx_message.IDE = CAN_ID_STD;
  fric_tx_message.RTR = CAN_RTR_DATA;
  fric_tx_message.DLC = 0x08;
  fric_can_send_data[0] = motor1 >> 8;
  fric_can_send_data[1] = motor1;
  fric_can_send_data[2] = motor2 >> 8;
  fric_can_send_data[3] = motor2;
  fric_can_send_data[4] = motor3 >> 8;
  fric_can_send_data[5] = motor3;
  fric_can_send_data[6] = motor4 >> 8;
  fric_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&GIMBAL_CAN, &fric_tx_message, fric_can_send_data, &send_mail_box);
}


/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}
/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回摩擦轮电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_fricL_motor_measure_point(void)
{
    return &motor_chassis[7];
}
const motor_measure_t *get_fricR_motor_measure_point(void)
{
    return &motor_chassis[8];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
