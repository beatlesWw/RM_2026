#ifndef AUTOGIMBAL_H
#define AUTOGIMBAL_H
#include "string.h"
#include "main.h"

#define BUFLENGTH  		128//最大接收的数据
#define DATELENGTH		12//有效数据

#define YAW_AUTO_SEN    -0.0048f//俯仰0.002
#define PITCH_AUTO_SEN  -0.003f//0.007满足靠近中心的速度要求 x,y 是妙算传来的等值数据

typedef struct
{
  float x;
  float y;
	
	float distance;
} CTRL;

typedef union
{
	CTRL Rec;
	uint8_t buf[DATELENGTH];
}BUF;

extern void  AUTO_control_init(void);
extern CTRL *get_AUTO_control_point(void);



#endif
