
#include "User_Task.h"
#include "CAN_Receive.h"            //加载摩擦轮发射闭环真实转速
#include "FreeRTOS.h"               //* pvParameters
#include "task.h"                   //vTaskDelay
#include "chassis_power_control.h"  //功率控制
#include "chassis_task.h"
#include "gimbal_task.h"
#include "communication.h"
#include "bsp_uart.h"
#include "chassis_omni.h"
#include <stdio.h>
#include "RM_Cilent_UI.h"
#include "string.h"
#define PI 3.1415936

//外部变量
// extern uint8_t fric_ui;
// extern uint8_t spin_ui;
// extern uint8_t auto_aim_ui;

//定义UI数据
Graph_Data G1, G2, G3, G4, G5, G6, G7, G8, G9, G10;
String_Data CH_SHOOT;
String_Data CH_SPIN;
String_Data CH_AUTO_AIM;

//定义UI字符
char CH_AUTO_AIM_ARR[8] = "auto_aim";
char shoot_arr[30] = "shoot";
char spin_arr[4] = "spin";

//定义其他UI变量
int time = 0;  //测试用
int flag_SHOOT = 0;//摩擦轮是否开启flag(0:关闭 1:开启)
int flag_SPIN = 0;//小陀螺是否开启flag(0:关闭 1:开启)
int flag_AUTO_AIM = 0;//自瞄是否开启flag(0:关闭 1:开启)
int flag_power = 0;//功率控制flag(0:关闭 1:开启)
int power_x = 280;//进度条的x坐标
int last_power_x = 280;//上一次进度条的x坐标

/*-------------------- Task --------------------*/

/**
 * @brief          UI任务
 * @param[in]      none
 * @retval         none
 */
void UserTask(void const * pvParameters)
{
	///////初始化UI////////
    memset(&G1, 0, sizeof(G1));                    //中心垂线
    memset(&G2, 0, sizeof(G2));                    //2m
    // memset(&G3, 0, sizeof(G3));                    //进度条条底
    memset(&G4, 0, sizeof(G4));                    //3m
    memset(&G5, 0, sizeof(G5));                    //5m
    memset(&G6, 0, sizeof(G6));                    //0m
    memset(&G7, 0, sizeof(G7));                    //射击指示圈
    memset(&G8, 0, sizeof(G8));                    //小陀螺指示圈
    memset(&G9, 0, sizeof(G9));                    //自瞄是否开启指示圈
    // memset(&G10, 0, sizeof(G10));                  //进度条
    memset(&CH_SHOOT, 0, sizeof(CH_SHOOT));        //射击字符
    memset(&CH_SPIN, 0, sizeof(CH_SPIN));          //小陀螺字符
    memset(&CH_AUTO_AIM, 0, sizeof(CH_AUTO_AIM));  //自瞄字符

	///////绘制非动态UI////////
    Line_Draw(&G1, "091", UI_Graph_ADD, 9, UI_Color_Purplish_red, 3, 980, 400, 980, 560);
    Line_Draw(&G2, "092", UI_Graph_ADD, 9, UI_Color_Purplish_red, 3, 910, 448, 1050, 448);
    // Line_Draw(&G3, "093", UI_Graph_ADD, 1, UI_Color_Black, 70, 80, 510, 280, 510);
    Line_Draw(&G4, "094", UI_Graph_ADD, 9, UI_Color_Purplish_red, 3, 935, 436, 1025, 436);
    Line_Draw(&G5, "095", UI_Graph_ADD, 9, UI_Color_Purplish_red, 3, 950, 429, 1005, 429);
    Line_Draw(&G6, "096", UI_Graph_ADD, 9, UI_Color_Purplish_red, 3, 910, 540, 1050, 540);
    Circle_Draw(&G8, "098", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 230, 700, 15);
    Circle_Draw(&G7, "097", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 230, 800, 15);
    UI_ReFresh(7, G1, G2, G4, G5, G6, G7, G8); 
    vTaskDelay(100);
    Char_Draw(&CH_SHOOT, "087", UI_Graph_ADD, 9, UI_Color_Yellow, 24, 5, 4, 80, 820, &shoot_arr[0]);
    Char_ReFresh(CH_SHOOT);
	vTaskDelay(100);

    Char_Draw(&CH_SPIN, "088", UI_Graph_ADD, 9, UI_Color_Yellow, 24, 4, 4, 80, 720, &spin_arr[0]);
    Char_ReFresh(CH_SPIN);
   	vTaskDelay(100);

    Circle_Draw(&G9, "099", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 300, 600, 15);
    Char_Draw(&CH_AUTO_AIM, "089", UI_Graph_ADD, 9, UI_Color_Yellow, 24, 8, 4, 80, 620,&CH_AUTO_AIM_ARR[0]);
    Char_ReFresh(CH_AUTO_AIM);
	vTaskDelay(100);

    UI_ReFresh(1,G9);
	vTaskDelay(100);

    // Line_Draw(&G10, "930", UI_Graph_ADD, 7, UI_Color_Green, 70, 80, 510, 280, 510);
    // UI_ReFresh(1, G10);
	///////绘制动态UI////////
    while (1) {
        ///////////////////////////////////////////////////摩擦轮是否开启UI///////////////////////////////////////////////////
        // if (flag_SHOOT == 0 && fric_ui == 0)  //关闭->关闭
        // {
        //     flag_SHOOT = 0;
        // } else if (flag_SHOOT == 0 && fric_ui == 1)  //关闭->开启
        // {
        //     flag_SHOOT = 1;
        //     Circle_Draw(&G7, "007", UI_Graph_Del, 9, UI_Color_Yellow, 15, 230, 800, 15);
        //     UI_ReFresh(1, G7);
        //     vTaskDelay(100);
        //     Circle_Draw(&G7, "007", UI_Graph_ADD, 9, UI_Color_Green, 15, 230, 800,15); 
        //     UI_ReFresh(1, G7);
        //     vTaskDelay(100);
        // } else if (flag_SHOOT == 1 && fric_ui == 1)  //开启->开启
        // {
        //     flag_SHOOT = 1;
        // } else if (flag_SHOOT == 1 && fric_ui == 0)  //开启->关闭
        // {
        //     flag_SHOOT = 0;
        //     Circle_Draw(&G7, "007", UI_Graph_Del, 9, UI_Color_Green, 15, 230, 800, 15);
        //     UI_ReFresh(1, G7);
        //     vTaskDelay(100);
        //     Circle_Draw(&G7, "007", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 230, 800, 15);
        //     UI_ReFresh(1, G7);
        //     vTaskDelay(100);
        // }

        /////////////////////////////////////////////////////小陀螺是否开启UI///////////////////////////////////////////////////
        // if (flag_SPIN == 0 && spin_ui == 0)  //关闭->关闭
        // {
        //     flag_SPIN = 0;
        // } else if (flag_SPIN == 0 && spin_ui == 1)  //关闭->开启
        // {
        //     flag_SPIN = 1;
        //     Circle_Draw(&G8, "098", UI_Graph_Del, 8, UI_Color_Yellow, 15, 230, 700, 15);
        //     UI_ReFresh(1, G8);
        //     vTaskDelay(100);
        //     Circle_Draw(&G8, "098", UI_Graph_ADD, 8, UI_Color_Green, 15, 230, 700, 15);
        //     UI_ReFresh(1, G8);
        //     vTaskDelay(100);
        // } else if (flag_SPIN == 1 && spin_ui == 1)  //开启->开启
        // {
        //     flag_SPIN = 1;
        // } else if (flag_SPIN == 1 && spin_ui == 0)  //开启->关闭
        // {
        //     flag_SPIN = 0;
        //     Circle_Draw(&G8, "098", UI_Graph_Del, 8, UI_Color_Green, 15, 230, 700, 15);
        //     UI_ReFresh(1, G8);
        //     vTaskDelay(100);
        //     Circle_Draw(&G8, "098", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 230, 700, 15);
        //     UI_ReFresh(1, G8);
        //     vTaskDelay(100);
        // }

        ///////////////////////////////////////////////////自瞄是否打开UI///////////////////////////////////////////////////
        // if (flag_AUTO_AIM == 0 && auto_aim_ui == 0)  //关闭->关闭
        // {
        //     flag_AUTO_AIM = 0;
        // } else if (flag_AUTO_AIM == 0 && auto_aim_ui == 1)  //关闭->开启
        // {
        //     flag_AUTO_AIM = 1;
        //     Circle_Draw(&G9, "099", UI_Graph_Del, 8, UI_Color_Yellow, 15, 300, 600, 15);
        //     UI_ReFresh(1, G9);
        //     vTaskDelay(100);
        //     Circle_Draw(&G9, "099", UI_Graph_ADD, 8, UI_Color_Green, 15, 300, 600, 15);
        //     UI_ReFresh(1, G9);
        //     vTaskDelay(100);
        // } else if (flag_AUTO_AIM == 1 && auto_aim_ui == 1)  //开启->开启
        // {
        //     flag_AUTO_AIM = 1;
        // } else if (flag_AUTO_AIM == 1 && auto_aim_ui == 0)  //开启->关闭
        // {
        //     flag_AUTO_AIM = 0;
        //     Circle_Draw(&G9, "099", UI_Graph_Del, 8, UI_Color_Green, 15, 300, 600, 15);
        //     UI_ReFresh(1, G9);
        //     vTaskDelay(100);
        //     Circle_Draw(&G9, "099", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 300, 600, 15);
        //     UI_ReFresh(1, G9);
        //     vTaskDelay(100);
        // }
        vTaskDelay(100);
        //osDelay(100);
    }
}
