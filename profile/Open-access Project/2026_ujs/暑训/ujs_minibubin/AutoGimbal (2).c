#include "AutoGimbal.h"
#include "referee.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "gimbal_task.h"
#define pi 3.1415926
#define ANGLE_TO_RAD 0.0174533
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
uint8_t sbus_rx_buffer[BUFLENGTH];   //缓存数组
uint8_t tx[2]={0,1};
BUF  RresPi;
TX_GAME game_date;//比赛时间数据
TX_DATE gimbaler_date;
TX_AUTO_AIM auto_to_nuc_data;

char txmessage[] = "";
void  AUTO_control_init(void)   //初始化是只在接收到nuc传来的数据后进行，所以调试要先让nuc发数据
{
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);								//使能串口空闲中断（两次消息的间隙会触发）
		HAL_UART_Receive_DMA(&huart1,sbus_rx_buffer,BUFLENGTH);  //把接收到的BUFLENGTH个数据存入sbus_rx_buffer这个缓存数组里
}

   

// 按字节顺序存储接收到的数据
void memory_from_buffer(uint8_t *buffer, CTRL *ctrl)
	{
		
		//////////////////////////////////////////////////////////////////
    ctrl->frame_header = buffer[0];
	//需要的部分
    memcpy(&ctrl->x, &buffer[1], 4);
    memcpy(&ctrl->y, &buffer[1+1 * 4], 4);
    memcpy(&ctrl->distance, &buffer[1+2 * 4], 4);
		memcpy(&ctrl->shoot_mode, &buffer[1+3*4], 4);
		memcpy(&ctrl->ahead, &buffer[1+4*4], 4);
		memcpy(&ctrl->ahead_y, &buffer[1+5*4], 4);
		memcpy(&ctrl->angle, &buffer[1+6*4], 4);
		memcpy(&ctrl->mode, &buffer[1+7*4], 4);
		memcpy(&ctrl->blank, &buffer[1+7*4], 4);
		memcpy(&ctrl->frame_tail, &buffer[1+7*4+1], 4);
		///////////////////////////////////////////////////////////////////

	 // 获取 yaw 电机的指针
    const gimbal_motor_t *yaw_motor = get_yaw_motor_point();
    const gimbal_motor_t *pitch_motor = get_pitch_motor_point();

  			ctrl->x=ctrl->x*ANGLE_TO_RAD;
	     ctrl->y=ctrl->y*ANGLE_TO_RAD;	
		

		

//		
//	if(ctrl->x !=0 && ctrl->y !=0)
//    {
//	ctrl->x=ctrl->x*pi/180+0.01;
//	ctrl->y=ctrl->y*pi/180-0.035;	
//	}
//	else
//	{
//		ctrl->x=0;
//		ctrl->y=0;
//	}

//    ctrl->blank = buffer[2 + 3 * sizeof(float)];
//    ctrl->FRAME_TAIL = buffer[2 + 3 * sizeof(float) + 1];
}



void Usart1Receive_IDLE(void)  //接收任务，是串口接收中断里的 ，接收到nuc的数据之后
{
	HAL_UART_DMAStop(&huart1);//关闭DMA
	uint8_t data_length  = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  //36-剩余空间 得到已经接收的数量
//	if( data_length == DATELENGTH )
//	{
//		memcpy(RresPi.buf,sbus_rx_buffer,data_length);//存入12字节数据，把缓存sbus_rx_buffer里的数据放在RresPi.buf里
//		//Latest_Remote_Control_Pack_Time = HAL_GetTick();//用于掉线检测
//	}
	
	 if (data_length == DATELENGTH) 
		 {
        // 检查帧头和帧尾是否正确
//        if (sbus_rx_buffer[0] == 0xFF && sbus_rx_buffer[DATELENGTH - 1] == 0x0B) {
            // 按字节顺序解析数据到RresPi.Rec
            memory_from_buffer(sbus_rx_buffer, &RresPi.Rec);
            //Latest_Remote_Control_Pack_Time = HAL_GetTick();//用于掉线检测
//        }
    }
	
    memset(sbus_rx_buffer,0,BUFLENGTH);//清除缓存数组

//		   //如果要帧头帧尾的话
//	if ((*RresPi.buf) >> 0 == 0xff && (*RresPi.buf) >> 16 == 0x0b)
//	{
//		RresPi.Rec.x =(*RresPi.buf) >> 4;
//		RresPi.Rec.y =(*RresPi.buf) >> 8;
//		RresPi.Rec.distance =(*RresPi.buf) >> 12;
//		//若加校验位对blank定义
//	}
	
	//老ttl串口
	//HAL_UART_Transmit_DMA(&huart1,game_date.rx_date,3); //把裁判系统传来的数据传给nuc
	HAL_UART_Transmit_DMA(&huart1,auto_to_nuc_data.board_tx_date,SEND_TO_NUC_DATA_LENGTH); 
	HAL_UART_Receive_DMA(&huart1,sbus_rx_buffer,BUFLENGTH);  //开启下一次接收中断 
}

void Virtual_uart_IDLE(uint32_t *Len)  //接收任务，是串口接收中断里的 ，接收到nuc的数据之后
{
	uint8_t data_length  = *Len;

	 if (data_length == DATELENGTH) 
		 {
        // 检查帧头和帧尾是否正确
//        if (sbus_rx_buffer[0] == 0xFF && sbus_rx_buffer[DATELENGTH - 1] == 0x0B) {
            // 按字节顺序解析数据到RresPi.Rec
            memory_from_buffer(sbus_rx_buffer, &RresPi.Rec);
            //Latest_Remote_Control_Pack_Time = HAL_GetTick();//用于掉线检测
//        }
    }
	
    memset(sbus_rx_buffer,0,BUFLENGTH);//清除缓存数组

//		   //如果要帧头帧尾的话
//	if ((*RresPi.buf) >> 0 == 0xff && (*RresPi.buf) >> 16 == 0x0b)
//	{
//		RresPi.Rec.x =(*RresPi.buf) >> 4;
//		RresPi.Rec.y =(*RresPi.buf) >> 8;
//		RresPi.Rec.distance =(*RresPi.buf) >> 12;
//		//若加校验位对blank定义
//	}
	
		//虚拟串口
		USBTransmit(auto_to_nuc_data.board_tx_date,30);
		
}

void USART1_IRQHandler(void)   //串口接收中断
{
	if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   //判断IDLE寄存器是否空闲
	{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);   //清除标志位                 
			Usart1Receive_IDLE();                          
	}
  HAL_UART_IRQHandler(&huart1);//判断中断类型并且清除中断标志位
}


CTRL *get_AUTO_control_point(void)
{ 
	return &RresPi.Rec;
}




void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)//发送回调函数
{
	if(huart->Instance==USART1)
	{
		memset(gimbaler_date.rx_date,0,9);
	}
}


