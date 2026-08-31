#include "dm_8009_drv.h"
#include "cmsis_os.h"
#include "main.h"
#include "CAN_receive.h"
#include "detect_task.h"
// IMU yaw角度最大最小值限制
#define FLOAT_MAX_INS_YAW 3.5f
#define FLOAT_MIN_INS_YAW -3.5f

// CAN发送消息头
static CAN_TxHeaderTypeDef  joint_tx_message;      // 关节电机发送消息头
static CAN_TxHeaderTypeDef  wheel_tx_message;      // 轮毂电机发送消息头
static CAN_TxHeaderTypeDef  chassis_transmit_message;  // 底盘数据发送消息头
uint8_t            chassis_send_data[6];            // 底盘发送数据缓冲区

/**
************************************************************************
* @brief:       浮点数转换为无符号整数
* @param[in]:   x_float: 待转换的浮点数
* @param[in]:   x_min:   范围最小值
* @param[in]:   x_max:   范围最大值
* @param[in]:   bits:    目标无符号整数的位数
* @retval:      无符号整数结果
* @details:     将输入的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	// 将浮点数按照给定范围和位数转换为无符号整数
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将输入的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	// 将无符号整数按照给定范围和位数转换为浮点数
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  关节电机初始化
 * @param  motor: 关节电机结构体指针
 * @param  id: 电机ID
 * @param  mode: 电机工作模式
 * @retval None
 */
void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode)
{
  motor->mode = mode;
  motor->para.id = id;
}

/**
 * @brief  轮毂电机初始化
 * @param  motor: 轮毂电机结构体指针
 * @param  id: 电机ID
 * @param  mode: 电机工作模式
 * @retval None
 */
void wheel_motor_init(Wheel_Motor_t *motor,uint16_t id,uint16_t mode)
{
  motor->mode = mode;
  motor->para.id = id;
}

/**
************************************************************************
* @brief:      	dm4310_fbdata: 解析DM4310电机反馈数据
* @param[in]:   motor:    关节电机结构体指针，用于存储电机信息和反馈数据
* @param[in]:   rx_data:  指向接收数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩、温度等参数，并存储到电机结构体中
************************************************************************
**/
void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len == 8)
	{  // 接收的数据长度为8个字节
	  motor->para.id = (rx_data[0]) & 0x0F;        // 提取电机ID（低4位）
	  motor->para.state = (rx_data[0]) >> 4;       // 提取电机状态（高4位）
	  motor->para.p_int = (rx_data[1] << 8) | rx_data[2];  // 位置原始数据（16位）
	  motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);  // 速度原始数据（12位）
	  motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];  // 扭矩原始数据（12位）
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16);  // 位置转换为浮点数 (-12.5, 12.5) rad
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12);  // 速度转换为浮点数 (-30.0, 30.0) rad/s
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // 扭矩转换为浮点数 (-10.0, 10.0) N·m
	  motor->para.Tmos = (float)(rx_data[6]);   // MOS管温度
	  motor->para.Tcoil = (float)(rx_data[7]);  // 线圈温度
	}
}

/**
************************************************************************
* @brief:      	底盘向上板发送数据
* @param[in]:   yaw_ctrl:   云台yaw轴偏差角度
* @param[in]:   pitch_ctrl: 云台pitch轴角度
* @param[in]:   mode:       模式标志位
* @param[in]:   reserve1:   保留字节1
* @param[in]:   reserve2:   保留字节2（未使用）
* @details:    	通过CAN总线向上板发送控制数据
************************************************************************
**/
int16_t yaw_value;    // yaw角度值（放大10000倍后的整数）
int16_t pitch_value;  // pitch角度值（放大10000倍后的整数）
void c_transmit_date(float yaw_ctrl, float pitch_ctrl,uint8_t mode,uint8_t reserve1, uint8_t reserve2)
{
	
	uint32_t send_mail_box;
	chassis_transmit_message.StdId = 0x10;           // CAN发送ID
	chassis_transmit_message.IDE = CAN_ID_STD;      // 标准帧
	chassis_transmit_message.RTR = CAN_RTR_DATA;    // 数据帧
	chassis_transmit_message.DLC = 0x08;            // 数据长度8字节

	yaw_value = (int16_t)(yaw_ctrl * 10000);      // yaw角度放大10000倍转为整数
	pitch_value = (int16_t)(pitch_ctrl * 10000);  // pitch角度放大10000倍转为整数
	
	chassis_send_data[0] = (uint8_t)(yaw_value >> 8);      // Yaw高字节
	chassis_send_data[1] = (uint8_t)(yaw_value & 0xFF);    // Yaw低字节
	chassis_send_data[2] = (uint8_t)(pitch_value >> 8);    // Pitch高字节
	chassis_send_data[3] = (uint8_t)(pitch_value & 0xFF);  // Pitch低字节
	chassis_send_data[4] = mode;        // 模式标志位
	chassis_send_data[5] = reserve1;    // 保留字节1

	HAL_CAN_AddTxMessage(&hcan1, &chassis_transmit_message, chassis_send_data, &send_mail_box);  // 发送CAN消息
}





/**
 * @brief  使能电机模式
 * @param  hcan: CAN总线句柄指针
 * @param  motor_id: 电机ID
 * @param  mode_id: 模式ID（MIT_MODE/POS_MODE/SPEED_MODE）
 * @retval None
 */
void enable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint32_t send_mail_box;
	joint_tx_message.StdId = motor_id + mode_id;  // 发送ID = 电机ID + 模式ID
	joint_tx_message.IDE = CAN_ID_STD;            // 标准帧
	joint_tx_message.RTR = CAN_RTR_DATA;          // 数据帧
	joint_tx_message.DLC = 0x08;                  // 数据长度8字节
	// 使能命令：0xFFFFFFFFFFFFFFFC
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;

    HAL_CAN_AddTxMessage(hcan, &joint_tx_message, data, &send_mail_box);
	
}


/**
 * @brief  失能电机模式
 * @param  hcan: CAN总线句柄指针
 * @param  motor_id: 电机ID
 * @param  mode_id: 模式ID（MIT_MODE/POS_MODE/SPEED_MODE）
 * @retval None
 */
void disable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint32_t send_mail_box;
	joint_tx_message.StdId = motor_id + mode_id;  // 发送ID = 电机ID + 模式ID
	joint_tx_message.IDE = CAN_ID_STD;            // 标准帧
	joint_tx_message.RTR = CAN_RTR_DATA;          // 数据帧
	joint_tx_message.DLC = 0x08;                  // 数据长度8字节
	// 失能命令：0xFFFFFFFFFFFFFFFD
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;

    HAL_CAN_AddTxMessage(hcan, &joint_tx_message, data, &send_mail_box);
	
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的关节电机控制函数
* @param[in]:   hcan:      CAN总线句柄指针
* @param[in]:   motor_id:  电机ID，指定目标电机
* @param[in]:   pos:       位置给定值 (rad)
* @param[in]:   vel:       速度给定值 (rad/s)
* @param[in]:   kp:        位置比例系数
* @param[in]:   kd:        位置微分系数
* @param[in]:   torq:      前馈扭矩给定值 (N·m)
* @retval:     	void
* @details:    	通过CAN总线发送MIT模式下的控制帧
*               MIT模式是一种力位混合控制模式，可以同时控制位置、速度和力矩
*               控制律：T = Kp*(pos_target - pos_actual) + Kd*(vel_target - vel_actual) + torq_feedforward
************************************************************************
**/
void mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
	uint8_t data[8];
	uint32_t send_mail_box;
	joint_tx_message.StdId = motor_id;       // 设置电机ID
	joint_tx_message.IDE = CAN_ID_STD;      // 标准帧
	joint_tx_message.RTR = CAN_RTR_DATA;    // 数据帧
	joint_tx_message.DLC = 0x08;            // 数据长度8字节
	
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	
	// 将浮点数参数转换为无符号整数
	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);  // 位置 16位
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);  // 速度 12位
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);  // Kp 12位
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);  // Kd 12位
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);  // 扭矩 12位

	// 数据打包：按照DM电机通信协议打包数据
	data[0] = (pos_tmp >> 8);                      // 位置高8位
	data[1] = pos_tmp;                             // 位置低8位
	data[2] = (vel_tmp >> 4);                      // 速度高8位
	data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);  // 速度低4位 + Kp高4位
	data[4] = kp_tmp;                              // Kp低8位
	data[5] = (kd_tmp >> 4);                       // Kd高8位
	data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);  // Kd低4位 + 扭矩高4位
	data[7] = tor_tmp;                             // 扭矩低8位
	
	HAL_CAN_AddTxMessage(hcan, &joint_tx_message, data, &send_mail_box);  // 发送控制命令
	
}

/**
 * @brief  轮毂电机MIT模式控制（速度范围更大）
 * @param  hcan: CAN总线句柄指针
 * @param  motor_id: 电机ID
 * @param  pos: 位置给定值 (rad)
 * @param  vel: 速度给定值 (rad/s)，范围 [-45, 45]
 * @param  kp: 位置比例系数
 * @param  kd: 位置微分系数
 * @param  torq: 前馈扭矩给定值 (N·m)
 * @retval None
 */
void mit_WHEEL(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
	uint8_t data[8];
	uint32_t send_mail_box;
	joint_tx_message.StdId = motor_id;       // 设置电机ID
	joint_tx_message.IDE = CAN_ID_STD;      // 标准帧
	joint_tx_message.RTR = CAN_RTR_DATA;    // 数据帧
	joint_tx_message.DLC = 0x08;            // 数据长度8字节
	
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

	// 将浮点数参数转换为无符号整数（使用轮毂电机参数范围）
	pos_tmp = float_to_uint(pos,  P_MIN2,  P_MAX2,  16);  // 位置 16位
	vel_tmp = float_to_uint(vel,  V_MIN2,  V_MAX2,  12);  // 速度 12位（范围更大）
	kp_tmp  = float_to_uint(kp,   KP_MIN2, KP_MAX2, 12);  // Kp 12位
	kd_tmp  = float_to_uint(kd,   KD_MIN2, KD_MAX2, 12);  // Kd 12位
	tor_tmp = float_to_uint(torq, T_MIN2,  T_MAX2,  12);  // 扭矩 12位

	// 数据打包
	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	data[7] = tor_tmp;
	
	HAL_CAN_AddTxMessage(hcan, &wheel_tx_message, data, &send_mail_box);  // 发送控制命令
	
}
