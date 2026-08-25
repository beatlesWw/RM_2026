/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal.c/h
  * @brief      云台控制任务所需要的变量和函数
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-03-10      Harry_Wong      1.初始化uart2的传输结构体
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
**/
#ifndef UART1_TYPEDEF_H
#define UART1_TYPEDEF_H

#include "attribute_typedef.h"
#include "remote_control.h"
#include "struct_typedef.h"

#define COMMUNICATE_SOF ((uint8_t)0x5A)

// clang-format off
#define Uart1_Test_ID        ((uint8_t)0x01)
#define Rc_Data_ID           ((uint8_t)0x02)
#define Gimbal_Data_ID       ((uint8_t)0x03)
#define Gimbal_Motor_Data_ID ((uint8_t)0x04)
// clang-format on
#define Uart1_Test_Duration ((uint32_t)100)
#define Rc_Data_Duration    ((uint32_t)100)

#define Uart1_Test_Gap ((uint32_t)100)
#define Rc_Data_Gap    ((uint32_t)15)

#define UART2_FRAME_MAX_SIZE              ((uint8_t) 250)

#define UART2_FRAME_HEADER_SIZE           sizeof(FrameHeader_t)
#define UART2_FRAME_TIMESTAMP_SIZE        ((uint8_t)  4)
#define UART2_FRAME_CRC16_SIZE            ((uint8_t)  2)
#define UART2_HEADER_CRC_LEN              (UART2_FRAME_HEADER_SIZE + UART2_FRAME_CRC16_SIZE)
#define UART2_HEADER_CRC_TIMESTAMP_LEN    (UART2_FRAME_HEADER_SIZE + UART2_FRAME_CRC16_SIZE + UART2_FRAME_TIMESTAMP_SIZE)
#define UART2_HEADER_TIMESTAMP_LEN        (UART2_FRAME_HEADER_SIZE + UART2_FRAME_TIMESTAMP_SIZE)

/*-------------------- Send & Receive --------------------*/

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH      = 1,
  STEP_ID          = 2,
  STEP_TYPE        = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} UnpackStep_e;

typedef struct 
{
  uint32_t Uart1_Test;
  uint32_t Rc_Data;
}LastSendTime_t;

typedef  struct
{
  uint8_t sof;   // 数据帧起始字节，固定值为 0xA5
  uint8_t len;   // 数据段长度
  uint8_t id;    // 数据段id
  uint8_t type;  // 数据段类型
  uint8_t crc;   // 数据帧头的 CRC8 
}  __attribute__((packed)) FrameHeader_t;

typedef struct
{
  FrameHeader_t *p_header;
  uint8_t        data_len;
  uint8_t        protocol_packet[UART2_FRAME_MAX_SIZE];
  UnpackStep_e   unpack_step;
  uint16_t       index;
} UnpackData_t;

//测试用数据包
typedef struct
{
  FrameHeader_t frame_header;

  uint32_t time_stamp;  //数据段时间戳
  
  struct
  {
    uint32_t test_data;
  } __attribute__((packed)) data;

  uint16_t crc16;       //crc16校验
} __attribute__((packed)) Uart1_Test_s;

//遥控器数据包
typedef struct
{
  FrameHeader_t frame_header;

  uint32_t time_stamp;  //数据段时间戳
  
  struct
  {
    RC_ctrl_t rc_ctrl;
    bool rc_toe_error;
  } __attribute__((packed)) data;

  uint16_t crc16;       //crc16校验
} __attribute__((packed)) Rc_Data_s;
#endif
