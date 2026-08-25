#ifndef AUTOGIMBAL_H
#define AUTOGIMBAL_H
#include "string.h"
#include "main.h"
#include "referee.h"
#define BUFLENGTH  		128//最大接收的数据
#define DATELENGTH		35//有效数据  接收的
#define SEND_TO_NUC_DATA_LENGTH 34  //发送的数据的字节数AUTO_SEND_TO_NUC_DATA_t

#define PITCH_AUTO_SEN    0.018f                            //
#define YAW_AUTO_SEN  0.029f                                //

typedef struct
{
	uint8_t frame_header;
  float x; 
  float y;
  float distance; 
	int shoot_mode;

	////////////导航/////////////
  float ahead;
  float ahead_y;
  float angle;
  int mode;
	//////////////////////////
	uint8_t blank;               //空白帧，视觉要不要校验由视觉决定
	uint8_t frame_tail ;         //帧尾
} CTRL;

typedef __packed struct
{
	uint8_t FRAME_HEADER ;       //帧头
	uint8_t mode;  //探测的颜色
	float roll;
	float pitch;
	float yaw;
	float big_pitch;
	float big_yaw;
	////////////////////////////////////22

		//////////////裁判/////////////////
	uint8_t game_progress; //比赛状态
	uint16_t remaining_time; //比赛剩余时间
	uint16_t sentry_hp;    //sentry血量self
	//uint8_t able_to_resurrection;  //是否可以免费买活 1可以 0不行
	//uint8_t center_gain_point;  //是否在中心增益点
	uint16_t self_outpost_HP;  //己方前哨战血量
	uint16_t projectile_allowance_17mm; //允许发弹量
	uint8_t self_support_point;  //己方与兑换区不重叠的补给区bool 0不在 1在
	////////////////////////////////////32
	
	
	uint8_t blank;               //空白帧，视觉要不要校验由视觉决定
	uint8_t FRAME_TAIL ;         //帧尾

}AUTO_SEND_TO_NUC_DATA_t;  //34


typedef union      //共用体
{
AUTO_SEND_TO_NUC_DATA_t   AUTO_SEND_TO_NUC_DATA;  
uint8_t board_tx_date[SEND_TO_NUC_DATA_LENGTH];  
} TX_AUTO_AIM;


typedef struct//发送数据
{
  float x;
  float y;
	uint8_t key_board;
} RX_DATE_t;

typedef union//接收数据
{
	CTRL Rec;
	uint8_t buf[DATELENGTH];
}BUF;

typedef struct//发送比赛状态
{
	 uint16_t game_time;
 uint8_t game_progress;
} GAME_DATE_t;


typedef union
{
//	RX_DATE_t RX_DATE;
	uint8_t rx_date[9];
}TX_DATE;

typedef union      
{
GAME_DATE_t GAME_DATE;  
	uint8_t rx_date[3];  //这里的rx_date是从裁判系统接收的数据
}TX_GAME;

extern void  AUTO_control_init(void);
extern CTRL *get_AUTO_control_point(void);
extern void memory_from_buffer(uint8_t *buffer, CTRL *ctrl);
extern void Virtual_uart_IDLE(uint32_t *Len);
#endif