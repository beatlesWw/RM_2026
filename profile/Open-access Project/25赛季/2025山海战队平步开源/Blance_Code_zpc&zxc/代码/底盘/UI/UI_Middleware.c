#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "ui_default_Group1_0.h"
#include "ui_default_Group2_0.h"
#include "ui_default_Group2_1.h"
#include "ui_default_Group2_2.h"
#include "ui_default_Group2_3.h"
#include "ui_default_Group2_4.h"
#include "ui_default_Change_0.h"
#include "ui_default_Update_0.h"
#include "ui_default_Update1_0.h"

/*
 *函数简介:UI初始化
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void UI_Middleware_Init(uint8_t Flag)
{
	switch(Flag)
	{
		case 0:_ui_init_default_Group1_0();break;
		case 1:_ui_init_default_Group2_0();break;
		case 2:_ui_init_default_Group2_1();break;
		case 3:_ui_init_default_Group2_2();break;
		case 4:_ui_init_default_Group2_3();break;
		case 5:_ui_init_default_Group2_4();break;
		case 6:_ui_init_default_Change_0();break;
		case 7:_ui_init_default_Update_0();break;
		case 8:_ui_init_default_Update1_0();break;
	}
}

/*
 *函数简介:UI更新
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void UI_Middleware_Updata(void)
{
	static uint8_t Count=0;
	Count=(Count+1)%2;
	
	if(Count==0)
	{
		_ui_update_default_Update_0();
	}
	if(Count==1)
	{
		_ui_update_default_Update1_0();
	}
}

void UI_Middleware_Warming(void)
{
	
}
