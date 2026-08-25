#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;//定义一个结构体
    can_filter_st.FilterActivation = ENABLE;//使能滤波器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;//滤波器模式：掩码模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//过滤器比例：32位
    can_filter_st.FilterIdHigh = 0x0000;//过滤器ID高位
    can_filter_st.FilterIdLow = 0x0000;//过滤器ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;//过滤器掩码ID高位
    can_filter_st.FilterMaskIdLow = 0x0000;//过滤器掩码ID低位
    can_filter_st.FilterBank = 0;//滤波器组号(CAN1和CAN2的滤波器组号不能相同,CAN1在0-13，CAN2在14-27)
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0; // 将过滤器分配给 FIFO0
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st); // 配置 CAN1 过滤器
    HAL_CAN_Start(&hcan1);// 启动 CAN1
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);// 开启 CAN1 FIFO0 消息通知 
    // 配置 CAN2 过滤器
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
