/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receivea.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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
#include "CANdata_analysis.h"

#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
float PowerData[4];
//����������λ��
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
//motor data read
#define get_motor_measure(ptr , data)                                    \
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
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���; 6:������� 2006���*/
 motor_measure_t motor_chassis[9];
	
	
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
	
	
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
	
static CAN_TxHeaderTypeDef  yaw_tx_message;
static uint8_t              yaw_can_send_data[8];
	
static CAN_TxHeaderTypeDef  superCAP_tx_message;
static uint8_t              superCAP_can_send_data[8];
	
static CAN_TxHeaderTypeDef  fric_tx_message;
static uint8_t              fric_can_send_data[8];
	
uint32_t receive_data[8];
int board_receive_data[8];

extern c_fbpara_t  receive_chassis_data;

uint8_t test_data[8];

Yaw_Motor_t yaw_motor[2];

/**
************************************************************************
* @brief:      	float_to_uint: ������ת��Ϊ�޷�����������
* @param[in]:   x_float:	��ת���ĸ�����
* @param[in]:   x_min:		��Χ��Сֵ
* @param[in]:   x_max:		��Χ���ֵ
* @param[in]:   bits: 		Ŀ���޷���������λ��
* @retval:     	�޷����������
* @details:    	�������ĸ����� x ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��ָ��λ�����޷�������
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
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
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
		//���ͨ��
		case 0x10:   //0x10 ��C������C�巢�����ݵ�ID
	  {
		  gimbal_receive_chassis(&receive_chassis_data,rx_data,8); //��������������
			break;
		}					
						
        case CAN_YAW_MOTOR_ID:   //yaw6020 	CAN_YAW_MOTOR_ID = 0x205,
	  {
//            static uint8_t i = 0;

//            i = rx_header.StdId - CAN_3508_M1_ID;
//            get_motor_measure(&motor_chassis[i], rx_data);  //motor_chassis[4]
//		  
		  
		   dm4310_fbdata1(&yaw_motor[1], rx_data,8);
		   if(yaw_motor[1].para.pos>=6.25&&yaw_motor[1].para.pos<=12.5)
               yaw_motor[1].para.pos-=6.25;
           if(yaw_motor[1].para.pos<=0&&yaw_motor[1].para.pos>-6.25)
               yaw_motor[1].para.pos+=6.25;
           if(yaw_motor[1].para.pos<=-6.25&&yaw_motor[1].para.pos>=-12.5)
               yaw_motor[1].para.pos+=12.5;                             
              
		       motor_chassis[4].last_ecd=motor_chassis[4].ecd;
               motor_chassis[4].ecd=yaw_motor[1].para.pos*8192/6.25;

          //  detect_hook(CHASSIS_MOTOR1_TOE + i);            //
            break;
        }
			
//		  case CAN_SUPERCAP_ID:
//			{
//				
//				uint16_t* pPowerdata  = (uint16_t *) rx_data;//CAN�յ���8���ֽڵ�����
//				PowerData[0] = (float)pPowerdata[0] / 100.f;//�����ѹ
//				PowerData[1] = (float)pPowerdata[1] / 100.f;//���ݵ�ѹ
//				PowerData[2] = (float)pPowerdata[2] / 100.f;//�������
//				PowerData[3] = (float)pPowerdata[3] / 100.f;//�趨����
//				break;
//			}
		case CAN_TRIGGER_MOTOR_ID:   //������    CAN_TRIGGER_MOTOR_ID = 0x207,
		{
			get_motor_measure(&motor_chassis[6], rx_data);
			//detect_hook(PITCH_GIMBAL_MOTOR_TOE  + 1);
			break;
		}

        default:
        {
            break;
        }
    }
}

else if (hcan==&hcan2){
	switch (rx_header.StdId)
	{ 
		case CAN_PITCH_MOTOR_ID:
        {
            get_motor_measure(&motor_chassis[5], rx_data);
			motor_chassis[5].speed_rpm=(motor_chassis[5].speed_rpm);
			i = motor_chassis[5].ecd;
            //detect_hook(PITCH_GIMBAL_MOTOR_TOE);
			break;
        }
	//Ħ����
	    case CAN_FRIC_L_ID:			//CAN_FRIC_L_ID = 0x201,  //motor_chassis[7]
        case CAN_FRIC_R_ID:		    //CAN_FRIC_R_ID = 0x202,	//motor_chassis[8]
	  {
		  
        static uint8_t j = 0;
        j = rx_header.StdId - 0x200;
        get_motor_measure(&motor_chassis[j], rx_data);  
       // detect_hook(CAN_FRIC_L_ID + j - 7);
        break;
	 
	  }
	}
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


//λ���ٶ�4310
void CAN_cmd_4310pitch_pvmode(float pos,float vel)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x102;
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


/**
  * @brief          ����4310�������ʹ�� λ���ٶ�
  * @param[in]      motor1: () 4310������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
//4310ʹ��
void init_gimbalpitch(void)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x02+0x100;
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




void enable_motor_mode_yaw(void)
{
	uint8_t data[8];
//	uint16_t id = motor_id + mode_id;
//	
	uint32_t send_mail_box;
    yaw_tx_message.StdId = 0x07;
    yaw_tx_message.IDE = CAN_ID_STD;
    yaw_tx_message.RTR = CAN_RTR_DATA;
    yaw_tx_message.DLC = 0x08;
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;

    HAL_CAN_AddTxMessage(&hcan1, &yaw_tx_message, data, &send_mail_box);
	
}


/**
************************************************************************
* @brief:      	mit_ctrl: MITģʽ�µĵ�����ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   pos:			λ�ø���ֵ
* @param[in]:   vel:			�ٶȸ���ֵ
* @param[in]:   kp:				λ�ñ���ϵ��
* @param[in]:   kd:				λ��΢��ϵ��
* @param[in]:   torq:			ת�ظ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������MITģʽ�µĿ���֡��
************************************************************************
**/
void mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint32_t send_mail_box;
    yaw_tx_message.StdId = motor_id;
    yaw_tx_message.IDE = CAN_ID_STD;
    yaw_tx_message.RTR = CAN_RTR_DATA;
    yaw_tx_message.DLC = 0x08;
	
	
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	
	
	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
  HAL_CAN_AddTxMessage(hcan, &yaw_tx_message, data, &send_mail_box);
	
}




/**
************************************************************************
* @brief:      	dm4310_fbdata: ��ȡDM4310����������ݺ���
* @param[in]:   motor:    ָ��motor_t�ṹ��ָ�룬������������Ϣ�ͷ�������
* @param[in]:   rx_data:  ָ������������ݵ�����ָ��
* @param[in]:   data_len: ���ݳ���
* @retval:     	void
* @details:    	�ӽ��յ�����������ȡDM4310����ķ�����Ϣ���������ID��
*               ״̬��λ�á��ٶȡ�Ť������¶Ȳ������Ĵ������ݵ�
************************************************************************
**/


void dm4310_fbdata1(Yaw_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==8)
	{//���ص�������8���ֽ�
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
















//  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
//  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
//  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
//  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
//  * @param[in]      rev: (0x208) ������������Ƶ���
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

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
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
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
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
  * @brief          ����Ħ���ֵ�����Ƶ���(0x201,0x202)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
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
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
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
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
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
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
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
  * @brief          ����Ħ���ֵ�� 3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_fricL_motor_measure_point(void)
{
    return &motor_chassis[5];
}
const motor_measure_t *get_fricR_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
/**
  * @brief          �ϰ����ݷ��ͺ���  IDΪ 0x15
  * @retval          none
  */
void CAN_gimbal_send__to_chassis(CAN_HandleTypeDef *hcan,float UP_INS_YAW)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x15;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
	
	int16_t YAW_NAW = (int16_t)((UP_INS_YAW / FLOAT_MAX_INS_YAW) * 32767);
	chassis_can_send_data[0] = (YAW_NAW >> 8) & 0xFF;  // ���ֽ�
	chassis_can_send_data[1] = (YAW_NAW ) & 0xFF;      // ���ֽ�  
	HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	
}
