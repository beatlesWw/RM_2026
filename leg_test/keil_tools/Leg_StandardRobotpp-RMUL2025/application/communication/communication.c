/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       communication.h
  * @brief      这里是机器人通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-14-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================
板间通信的数据包结构如下：
    0x00 header
    0x01 data
    0x03 CRC16
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "communication.h"

#include "CRC8_CRC16.h"
#include "bsp_uart.h"
#include "bsp_usart.h"
#include "fifo.h"
#include "robot_param.h"
#include "usb_debug.h"
#include "signal_generator.h"
#include "detect_task.h"

#define USART_RX_BUF_LENGHT 512
#define USART1_FIFO_BUF_LENGTH 1024

// data send time
LastSendTime_t LastSendTime;
// send data
BoardCommunicateData_s BOARD_TX_DATA;
Uart1_Test_s Uart1_Test;
Rc_Data_s Rc_Data;
// receive data
uint8_t BOARD_RX_DATA[DATA_NUM][DATA_LEN + 1];  //第一位存放数据长度信息

// receive data buffer
uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t usart1_fifo;
uint8_t usart1_fifo_buf[USART1_FIFO_BUF_LENGTH];
UnpackData_t usart1_unpack_obj;

/**
 * @brief 数据包初始化宏定义
 */
#define UART1DataInit(data_name)                                                    \
{                                                                                   \
    memset (&(##data_name##) , 0 , sizeof(##data_name##_s));                        \
    ##data_name##.frame_header.sof = FRAME_HEADER_SOF;                              \
    ##data_name##.frame_header.len = sizeof(##data_name##.data);                              \
    ##data_name##.frame_header.id  = ##data_name##_ID;                                             \
    ##data_name##.frame_header.type  = 0;                                           \
    append_CRC8_check_sum((uint8_t*)(&(##data_name##.frame_header)) , sizeof(##data_name##.frame_header));                                             \
    LastSendTime.##data_name## = 0 ;                                                \
}                                                                                   \

/**
 * @brief 数据发送进行宏定义
 */
#define UART1DataPack(data_name)                                                    \
{                                                   \
    ##data_name##DataRenew();                       \
    UartSendTxMessage(&huart1, (uint8_t *)(&(##data_name##)), sizeof(##data_name##), ##data_name##_Duration);\
}    \

/**
 * @brief 对于发送以及取数据进行宏定义
 */
#define Uart1CheckDurationAndSend(data_name)                                       \
{                                                                                  \
    if (HAL_GetTick() - LastSendTime.##data_name## >= ##data_name##_Gap ) \
    {                                                                              \
        LastSendTime.##data_name## = HAL_GetTick();                                \
        UART1DataPack(##data_name##);                                                  \
    }                                                                              \
}                                                                                  \
                                        
/**
 * @brief 对于接收数据进行存储
 */
#define UART1DataSave(data_name)                                                   \
{                                                                                   \
    uint16_t crc_ok = verify_CRC16_check_sum(received, sizeof(##data_name##_s));    \
    if (crc_ok) {                                                                                     \
        memcpy(&##data_name##, received, sizeof(##data_name##_s));    \
    }                                                                                                                   \
}                                    \



void Uart1_TestDataRenew()
{
    Uart1_Test.data.test_data =HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)(&Uart1_Test), sizeof(Uart1_Test_s));
}

void Rc_DataDataRenew()
{
    if (toe_is_error(DBUS_TOE)){
      memset(&Rc_Data.data.rc_ctrl,0,sizeof(RC_ctrl_t));
      Rc_Data.data.rc_ctrl.rc.s[0] = RC_SW_DOWN;
      Rc_Data.data.rc_ctrl.rc.s[1] = RC_SW_UP;
      Rc_Data.data.rc_toe_error = true;
    }else{
      // Rc_Data.data.rc_ctrl.rc.ch[0] = GenerateSinWave(10,0,4);
      memcpy(&Rc_Data.data.rc_ctrl, get_remote_control_point(),sizeof(RC_ctrl_t));
      Rc_Data.data.rc_toe_error = false;
    }
    append_CRC16_check_sum((uint8_t *)(&Rc_Data), sizeof(Rc_Data_s));
}



// 4pin Uart串口初始化
void Usart1Init(void)
{
    fifo_s_init(&usart1_fifo, usart1_fifo_buf, USART1_FIFO_BUF_LENGTH);
    usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGHT);

    UART1DataInit(Uart1_Test);
    UART1DataInit(Rc_Data);
}

void UART1_task(void)
{
    if (__SELF_BOARD_ID == C_BOARD_BALANCE_CHASSIS)
    {
        Uart1CheckDurationAndSend(Rc_Data);
    }
    else if (__SELF_BOARD_ID == C_BOARD_BALANCE_GIMBAL)
    {
        Uart1CheckDurationAndSend(Uart1_Test);
    }
    DataUnpack();
}

// 4pin Uart口中断处理函数
void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if (USART1->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        static uint16_t this_time_rx_len = 0;

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&usart1_fifo, (char *)usart1_buf[0], this_time_rx_len);
            // detect_hook(REFEREE_TOE);
        } else {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&usart1_fifo, (char *)usart1_buf[1], this_time_rx_len);
            // detect_hook(REFEREE_TOE);
        }
    }
}

/**
 * @brief 将数据打包填入发送数据结构体中
 * @param data 数据段的数组指针
 * @param data_lenth 数据段长度
 * @param data_id 数据段id
 */
void DataPack(uint8_t * data, uint8_t data_lenth, uint8_t data_id)
{
    BOARD_TX_DATA.frame_header.sof = 0xA5;        // 数据帧起始字节，固定值为 0xA5
    BOARD_TX_DATA.frame_header.len = data_lenth;  // 数据帧中数据段的长度
    BOARD_TX_DATA.frame_header.id = data_id;
    // 添加帧头 CRC8 校验位
    append_CRC8_check_sum(
        (uint8_t *)(&BOARD_TX_DATA.frame_header), sizeof(BOARD_TX_DATA.frame_header));

    memcpy(BOARD_TX_DATA.data, data, data_lenth);  // 数据段

    // 帧尾CRC16，整包校验
    append_CRC16_check_sum((uint8_t *)(&BOARD_TX_DATA), sizeof(BOARD_TX_DATA));
}

void Uart2DataSolve(uint8_t * frame){
    uint32_t time_stamp = 0;

    uint8_t index = 0;

    FrameHeader_t frame_header;

    memcpy(&frame_header, frame, sizeof(FrameHeader_t));
    index += sizeof(FrameHeader_t);

    memcpy(&time_stamp, frame + index, sizeof(uint32_t));
    index += sizeof(uint32_t);

    switch (frame_header.id)
    {
    case Uart1_Test_ID:{
        memcpy(&Uart1_Test, frame, sizeof(Uart1_Test_s));
        } break;
    case Rc_Data_ID:{
        memcpy(&Rc_Data, frame, sizeof(Rc_Data_s));
        const RC_ctrl_t * rc_ctrl = get_remote_control_point();
        memcpy((RC_ctrl_t *)rc_ctrl, &Rc_Data.data.rc_ctrl, sizeof(RC_ctrl_t));
        
    
    } break;
    default:
        break;
    }
}

/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void DataUnpack(void)
{
    uint8_t byte = 0;
    UnpackData_t *p_obj = &usart1_unpack_obj;

    while (fifo_s_used(&usart1_fifo)) {
        byte = fifo_s_get(&usart1_fifo);
        switch(p_obj->unpack_step)
        {
          case STEP_HEADER_SOF:
          {
            if(byte == FRAME_HEADER_SOF)
            {
              p_obj->unpack_step = STEP_LENGTH;
              p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
              p_obj->index = 0;
            }
          }break;
          
          case STEP_LENGTH:
          {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_ID;
          }break;
          
          case STEP_ID:
          {
            p_obj->protocol_packet[p_obj->index++] = byte;
    
            if(p_obj->data_len < (UART2_FRAME_MAX_SIZE - UART2_HEADER_CRC_TIMESTAMP_LEN))
            {
              p_obj->unpack_step = STEP_TYPE;
            }
            else
            {
              p_obj->unpack_step = STEP_HEADER_SOF;
              p_obj->index = 0;
            }
          }break;
          case STEP_TYPE:
          {
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_HEADER_CRC8;
          }break;
    
          case STEP_HEADER_CRC8:
          {
            p_obj->protocol_packet[p_obj->index++] = byte;
    
            if (p_obj->index == UART2_FRAME_HEADER_SIZE)
            {
              if ( verify_CRC8_check_sum(p_obj->protocol_packet, UART2_FRAME_HEADER_SIZE) )
              {
                p_obj->unpack_step = STEP_DATA_CRC16;
              }
              else
              {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
              }
            }
          }break;  
          
          case STEP_DATA_CRC16:
          {
            if (p_obj->index < UART2_HEADER_CRC_TIMESTAMP_LEN + p_obj->data_len)
            {
               p_obj->protocol_packet[p_obj->index++] = byte;  
            }
            if (p_obj->index >= UART2_HEADER_CRC_TIMESTAMP_LEN + p_obj->data_len)
            {
              p_obj->unpack_step = STEP_HEADER_SOF;
              p_obj->index = 0;
    
              if ( verify_CRC16_check_sum(p_obj->protocol_packet, UART2_HEADER_CRC_TIMESTAMP_LEN + p_obj->data_len) )
              {
                Uart2DataSolve(p_obj->protocol_packet);
              }
            }
          }break;
    
          default:
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }break;
        }
    }
}

uint32_t GetUART1TestValue(void)
{
    return Uart1_Test.data.test_data;
}

bool GetUartRcToeError(void)
{
    return Rc_Data.data.rc_toe_error;
}
