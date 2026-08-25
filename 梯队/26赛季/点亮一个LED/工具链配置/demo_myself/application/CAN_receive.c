#include "main.h"
#include "CAN_receive.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
//没太懂这里为什么要宏定义 这个函数吗get_motor_measure（） 不知道为什么要用宏定义 但是他们就是用了。。。 其使用void好像也可以
   static motor_measure_t motor_chassis[7];//每个电机参数的记录数组

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//接收CAN数据
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            break;
        }
        default:
        {
            break;
        }
    }
}

void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)//发送函数
{
    uint32_t send_mail_box;//一个CAN有0.1.2三个邮箱，用于查询数据
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;//广播形式发给所有电机,不清楚yaw(0x205)电机为什么会自动读取0.1数组的数据，我用实验室那个电机吧ID设置为0x205就行吗，拨成001的样子吗
    //                                            这里我发的送can的id为0x1ff，然后发送内容为四个int16_t的东西  当0x205的电机收到这一阵 一看是0x1ff的canid后 知道我需要接受这一帧代码 再根据点击是0x205
    // 知道控制电机的数据室第一个int16的控制内容 然后他自己再做一些处理最后出来我需要的电流值
    // 我用实验室那个电机吧ID设置为0x205就行吗，拨成001的样子吗：是的 实验室电机的id我目前设置的是电机id是1
    
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;//11bytes
    gimbal_tx_message.DLC = 0x08;//数据帧有8个字节
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;//传的电流值，(0x205) 6020电机控制电压, 范围 [-30000,30000]
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;//这是啥？（电机转速反馈）是这个吗？这个是人家写的代码，它的代码刚好七个电机，所以空出来了 rev没啥特殊的含义，如果你有电机can id是0x208的话 这一帧就是控制0x208的那个电机
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
//而且这里怎么没有那个什么滤波模式，掩码模式的代码
//你他大把的 掩码模式在我给你们的bsp的bsp_can里 真不要自己写了。。。。
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;//0x700 是底盘电机ID复位的标准命令ID（大疆等常见电调协议中，0x700 通常用于ID重置或广播命令
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;//8个字节
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
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)//用于拿到yaw（及6020）电机的数据指针
{
    return &motor_chassis[4];
}

/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}
