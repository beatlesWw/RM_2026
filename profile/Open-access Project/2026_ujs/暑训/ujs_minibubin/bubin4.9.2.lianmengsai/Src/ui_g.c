//
// Created by RM UI Designer
// Dynamic Edition
//
#include <math.h>
#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "AutoGimbal.h"
#include "remote_control.h"
//////////////////////////////////////一直要有///////////////////////////////
#define UI_CENTER_X 960
#define UI_CENTER_Y 540


#define TOTAL_FIGURE 23
#define TOTAL_STRING 3

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

const gimbal_motor_t *ui_pitch;
const RC_ctrl_t *zimiao_open;
int rotated_x0;
int rotated_y0;
int rotated_x1;
int rotated_y1;
extern chassis_behaviour_e chassis_behaviour_mode;
//extern gimbal_motor_t gimbal_behaviour_mode;
#define BLINK_INTERVAL 200  // 闪烁间隔(ms)，可根据需要调整
static uint32_t last_blink_time = 0;
static uint8_t blink_state = 7;  



void ui_init_g() {
	ui_pitch = get_pitch_motor_point();

	
    ui_g_static_Line1->figure_type = 0;
    ui_g_static_Line1->operate_type = 1;
    ui_g_static_Line1->layer = 1;
    ui_g_static_Line1->color = 8;
    ui_g_static_Line1->start_x = 581;
    ui_g_static_Line1->start_y = 11;
    ui_g_static_Line1->width = 3;
    ui_g_static_Line1->end_x = 824;
    ui_g_static_Line1->end_y = 320;

    ui_g_static_Line2->figure_type = 0;
    ui_g_static_Line2->operate_type = 1;
    ui_g_static_Line2->layer = 1;
    ui_g_static_Line2->color = 8;
    ui_g_static_Line2->start_x = 1367;
    ui_g_static_Line2->start_y = 11;
    ui_g_static_Line2->width = 3;
    ui_g_static_Line2->end_x = 1122;
    ui_g_static_Line2->end_y = 320;

    ui_g_dynamic_number3->figure_type = 6;
    ui_g_dynamic_number3->operate_type = 2;
    ui_g_dynamic_number3->layer = 1;
    ui_g_dynamic_number3->color = 3;
    ui_g_dynamic_number3->start_x = 1318;
    ui_g_dynamic_number3->start_y = 801;
    ui_g_dynamic_number3->width = 3;
	ui_g_dynamic_number3->font_size =26;
    ui_g_dynamic_number3->number = -(ui_pitch->absolute_angle*57.3);

    ui_g_static1_midLine->figure_type = 0;
    ui_g_static1_midLine->operate_type = 1;
    ui_g_static1_midLine->layer = 2;
    ui_g_static1_midLine->color = 8;
    ui_g_static1_midLine->start_x = 1348;
    ui_g_static1_midLine->start_y = 538;
    ui_g_static1_midLine->width = 4;
    ui_g_static1_midLine->end_x = 1319;
    ui_g_static1_midLine->end_y = 538;

    ui_g_static1_upLine->figure_type = 0;
    ui_g_static1_upLine->operate_type = 1;
    ui_g_static1_upLine->layer = 0;
    ui_g_static1_upLine->color = 5;
    ui_g_static1_upLine->start_x = 1285;
    ui_g_static1_upLine->start_y = 755;
    ui_g_static1_upLine->width = 4;
    ui_g_static1_upLine->end_x = 1257;
    ui_g_static1_upLine->end_y = 739;

    ui_g_static1_downLine->figure_type = 0;
    ui_g_static1_downLine->operate_type = 1;
    ui_g_static1_downLine->layer = 0;
    ui_g_static1_downLine->color = 5;
    ui_g_static1_downLine->start_x = 1343;
    ui_g_static1_downLine->start_y = 476;
    ui_g_static1_downLine->width = 4;
    ui_g_static1_downLine->end_x = 1314;
    ui_g_static1_downLine->end_y = 481;

    ui_g_static1_NewNumber1->figure_type = 6;
    ui_g_static1_NewNumber1->operate_type = 1;
    ui_g_static1_NewNumber1->layer = 0;
    ui_g_static1_NewNumber1->color = 8;
    ui_g_static1_NewNumber1->start_x = 1236;
    ui_g_static1_NewNumber1->start_y = 708;
    ui_g_static1_NewNumber1->width = 1;
    ui_g_static1_NewNumber1->font_size = 12;
    ui_g_static1_NewNumber1->number = 30;

    ui_g_static1_NewNumber2->figure_type = 6;
    ui_g_static1_NewNumber2->operate_type = 1;
    ui_g_static1_NewNumber2->layer = 0;
    ui_g_static1_NewNumber2->color = 8;
    ui_g_static1_NewNumber2->start_x = 1271;
    ui_g_static1_NewNumber2->start_y = 477;
    ui_g_static1_NewNumber2->width = 1;
    ui_g_static1_NewNumber2->font_size = 12;
    ui_g_static1_NewNumber2->number = 10;

    ui_g_static1_NewLine3->figure_type = 0;
    ui_g_static1_NewLine3->operate_type = 1;
    ui_g_static1_NewLine3->layer = 0;
    ui_g_static1_NewLine3->color = 8;
    ui_g_static1_NewLine3->start_x = 1342;
    ui_g_static1_NewLine3->start_y = 605;
    ui_g_static1_NewLine3->width = 4;
    ui_g_static1_NewLine3->end_x = 1313;
    ui_g_static1_NewLine3->end_y = 600;

    ui_g_static1_NewLine4->figure_type = 0;
    ui_g_static1_NewLine4->operate_type = 1;
    ui_g_static1_NewLine4->layer = 0;
    ui_g_static1_NewLine4->color = 8;
    ui_g_static1_NewLine4->start_x = 1325;
    ui_g_static1_NewLine4->start_y = 671;
    ui_g_static1_NewLine4->width = 4;
    ui_g_static1_NewLine4->end_x = 1297;
    ui_g_static1_NewLine4->end_y = 662;

    ui_g_static2_NewLine5->figure_type = 0;
    ui_g_static2_NewLine5->operate_type = 1;
    ui_g_static2_NewLine5->layer = 1;
    ui_g_static2_NewLine5->color = 8;
    ui_g_static2_NewLine5->start_x = 1295;
    ui_g_static2_NewLine5->start_y = 735;
    ui_g_static2_NewLine5->width = 4;
    ui_g_static2_NewLine5->end_x = 1271;
    ui_g_static2_NewLine5->end_y = 719;

    ui_g_static2_NewNumber6->figure_type = 6;
    ui_g_static2_NewNumber6->operate_type = 1;
    ui_g_static2_NewNumber6->layer = 1;
    ui_g_static2_NewNumber6->color = 8;
    ui_g_static2_NewNumber6->start_x = 1280;
    ui_g_static2_NewNumber6->start_y = 594;
    ui_g_static2_NewNumber6->width = 1;
    ui_g_static2_NewNumber6->font_size = 12;
    ui_g_static2_NewNumber6->number = 10;

    ui_g_static_NewArc->figure_type = 4;
    ui_g_static_NewArc->operate_type = 2;
    ui_g_static_NewArc->layer = 1;
    ui_g_static_NewArc->color = 2;
    ui_g_static_NewArc->start_x = 1187;
    ui_g_static_NewArc->start_y = 534;
    ui_g_static_NewArc->width = 5;
    ui_g_static_NewArc->start_angle = 30;
    ui_g_static_NewArc->end_angle = 150;
    ui_g_static_NewArc->rx = 158;
    ui_g_static_NewArc->ry = 283;

    ui_g_dynamic_NewRound->figure_type = 2;
    ui_g_dynamic_NewRound->operate_type = 1;
    ui_g_dynamic_NewRound->layer = 0;
    ui_g_dynamic_NewRound->color = 5;
    ui_g_dynamic_NewRound->start_x = 473;
    ui_g_dynamic_NewRound->start_y = 300;
    ui_g_dynamic_NewRound->width = 16;
    ui_g_dynamic_NewRound->r = 19;

    ui_g_dynamic_midLine->figure_type = 0;
    ui_g_dynamic_midLine->operate_type = 2;
    ui_g_dynamic_midLine->layer = 2;
    ui_g_dynamic_midLine->color = 1;
    ui_g_dynamic_midLine->start_x = rotated_x0;
    ui_g_dynamic_midLine->start_y = rotated_y0;
    ui_g_dynamic_midLine->width = 12;
    ui_g_dynamic_midLine->end_x = rotated_x1;
    ui_g_dynamic_midLine->end_y = rotated_y1;

    ui_g_static_shu->figure_type = 0;
    ui_g_static_shu->operate_type = 1;
    ui_g_static_shu->layer = 0;
    ui_g_static_shu->color = 8;
    ui_g_static_shu->start_x = 960;
    ui_g_static_shu->start_y = 620;
    ui_g_static_shu->width = 3;
    ui_g_static_shu->end_x = 960;
    ui_g_static_shu->end_y = 460;

    ui_g_static2_heng1->figure_type = 0;
    ui_g_static2_heng1->operate_type = 1;
    ui_g_static2_heng1->layer = 0;
    ui_g_static2_heng1->color = 8;
    ui_g_static2_heng1->start_x = 880;
    ui_g_static2_heng1->start_y = 540;
    ui_g_static2_heng1->width = 3;
    ui_g_static2_heng1->end_x = 1040;
    ui_g_static2_heng1->end_y = 540;

    ui_g_static2_heng2->figure_type = 0;
    ui_g_static2_heng2->operate_type = 1;
    ui_g_static2_heng2->layer = 0;
    ui_g_static2_heng2->color = 8;
    ui_g_static2_heng2->start_x = 900;
    ui_g_static2_heng2->start_y = 520;
    ui_g_static2_heng2->width = 3;
    ui_g_static2_heng2->end_x = 1020;
    ui_g_static2_heng2->end_y = 520;

    ui_g_static2_heng3->figure_type = 0;
    ui_g_static2_heng3->operate_type = 1;
    ui_g_static2_heng3->layer = 0;
    ui_g_static2_heng3->color = 8;
    ui_g_static2_heng3->start_x = 930;
    ui_g_static2_heng3->start_y = 500;
    ui_g_static2_heng3->width = 3;
    ui_g_static2_heng3->end_x = 991;
    ui_g_static2_heng3->end_y = 500;

    ui_g_dynamic_number1->figure_type = 6;
    ui_g_dynamic_number1->operate_type = 2;
    ui_g_dynamic_number1->layer = 0;
    ui_g_dynamic_number1->color = 0;
    ui_g_dynamic_number1->start_x = 432;
    ui_g_dynamic_number1->start_y = 795;
    ui_g_dynamic_number1->width = 3;
    ui_g_dynamic_number1->font_size = 26;
    ui_g_dynamic_number1->number = 12345;

    ui_g_dynamic_number2->figure_type = 6;
    ui_g_dynamic_number2->operate_type = 2;
    ui_g_dynamic_number2->layer = 0;
    ui_g_dynamic_number2->color = 2;
    ui_g_dynamic_number2->start_x = 433;
    ui_g_dynamic_number2->start_y = 732;
    ui_g_dynamic_number2->width = 3;
    ui_g_dynamic_number2->font_size = 26;
    ui_g_dynamic_number2->number = 12345;

    ui_g_dynamic_zimiaojiaodu->figure_type = 4;
    ui_g_dynamic_zimiaojiaodu->operate_type = 2;
    ui_g_dynamic_zimiaojiaodu->layer = 0;
    ui_g_dynamic_zimiaojiaodu->color = 6;
    ui_g_dynamic_zimiaojiaodu->start_x = 724;
    ui_g_dynamic_zimiaojiaodu->start_y = 531;
    ui_g_dynamic_zimiaojiaodu->width = 5;
    ui_g_dynamic_zimiaojiaodu->start_angle = 210;
    ui_g_dynamic_zimiaojiaodu->end_angle = 250;
    ui_g_dynamic_zimiaojiaodu->rx = 157;
    ui_g_dynamic_zimiaojiaodu->ry = 283;

    ui_g_dynamic_zimiaohuokong->figure_type = 4;
    ui_g_dynamic_zimiaohuokong->operate_type = 2;
    ui_g_dynamic_zimiaohuokong->layer = 0;
    ui_g_dynamic_zimiaohuokong->color = 6;
    ui_g_dynamic_zimiaohuokong->start_x = 724;
    ui_g_dynamic_zimiaohuokong->start_y = 531;
    ui_g_dynamic_zimiaohuokong->width = 5;
    ui_g_dynamic_zimiaohuokong->start_angle = 290;
    ui_g_dynamic_zimiaohuokong->end_angle = 330;
    ui_g_dynamic_zimiaohuokong->rx = 157;
    ui_g_dynamic_zimiaohuokong->ry = 283;

    ui_g_static_Text3->figure_type = 7;
    ui_g_static_Text3->operate_type = 1;
    ui_g_static_Text3->layer = 1;
    ui_g_static_Text3->color = 5;
    ui_g_static_Text3->start_x = 304;
    ui_g_static_Text3->start_y = 327;
    ui_g_static_Text3->width = 3;
    ui_g_static_Text3->font_size = 33;
    ui_g_static_Text3->str_length = 4;
    strcpy(ui_g_static_Text3->string, "fric");

    ui_g_static_Text1->figure_type = 7;
    ui_g_static_Text1->operate_type = 1;
    ui_g_static_Text1->layer = 1;
    ui_g_static_Text1->color = 0;
    ui_g_static_Text1->start_x = 165;
    ui_g_static_Text1->start_y = 794;
    ui_g_static_Text1->width = 3;
    ui_g_static_Text1->font_size = 30;
    ui_g_static_Text1->str_length = 8;
    strcpy(ui_g_static_Text1->string, "distance");

    ui_g_static_Text2->figure_type = 7;
    ui_g_static_Text2->operate_type = 1;
    ui_g_static_Text2->layer = 1;
    ui_g_static_Text2->color = 2;
    ui_g_static_Text2->start_x = 165;
    ui_g_static_Text2->start_y = 735;
    ui_g_static_Text2->width = 3;
    ui_g_static_Text2->font_size = 30;
    ui_g_static_Text2->str_length = 5;
    strcpy(ui_g_static_Text2->string, "pitch");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
	// 更新 pitch 角度显示
    ui_g_dynamic_number3->number = (-ui_pitch->absolute_angle * 57.3); 
    ui_g_dirty_figure[2] = 1; 
	
	
	
	// 更新 distance 角度显示
    ui_g_dynamic_number1->number = 0;
    ui_g_dirty_figure[19] = 1;
    
	// 更新 自瞄pitch 角度显示
    ui_g_dynamic_number2->number = (-ui_pitch->absolute_angle * 57.3); 
    ui_g_dirty_figure[20] = 1;
	
	
	
	
    // 计算旋转角度（弧度）
    float angle_rad = -ui_pitch->absolute_angle; // 已经是弧度，无需再转换
    
    // 原始线段端点（相对于中心点的偏移）
    int x0_offset = 1348 - UI_CENTER_X;
    int y0_offset = 537 - UI_CENTER_Y;
    int x1_offset = 1319 - UI_CENTER_X;
    int y1_offset = 537 - UI_CENTER_Y;
    
    // 计算旋转矩阵
    float cos_theta = cosf(angle_rad);
    float sin_theta = sinf(angle_rad);
    
    // 直接更新线段坐标（不要使用中间变量）
    ui_g_dynamic_midLine->start_x = (int)(cos_theta * x0_offset - sin_theta * y0_offset) + UI_CENTER_X;
    ui_g_dynamic_midLine->start_y = (int)(sin_theta * x0_offset + cos_theta * y0_offset) + UI_CENTER_Y;
    ui_g_dynamic_midLine->end_x = (int)(cos_theta * x1_offset - sin_theta * y1_offset) + UI_CENTER_X;
    ui_g_dynamic_midLine->end_y = (int)(sin_theta * x1_offset + cos_theta * y1_offset) + UI_CENTER_Y;
    
    ui_g_dirty_figure[14] = 1;
	
	 // 小陀螺模式闪烁逻辑（针对 ui_g_dynamic_zimiaojiaodu）
    if(chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
		{
        // 检查是否到达闪烁间隔
        if(HAL_GetTick() - last_blink_time > BLINK_INTERVAL) {
            last_blink_time = HAL_GetTick();
            blink_state = !blink_state;  // 切换状态
            ui_g_dynamic_zimiaojiaodu->color = blink_state ? 7 : 8;
            ui_g_dirty_figure[21] = 1;
        }
    } 
	else {
            ui_g_dynamic_zimiaojiaodu->color = 6;
            ui_g_dirty_figure[21] = 1;
    }



      zimiao_open = get_remote_control_point();
	// 自瞄模式闪烁
    if(zimiao_open->mouse.press_r)
		{
        // 检查是否到达闪烁间隔
        if(HAL_GetTick() - last_blink_time > BLINK_INTERVAL)
		{
            last_blink_time = HAL_GetTick();
            blink_state = !blink_state;  // 切换状态
            
            // 根据状态设置颜色
            ui_g_dynamic_zimiaohuokong->color = blink_state ? 7 : 8; // 0:黑色，15:白色
            ui_g_dirty_figure[22] = 1;
        }
    } else
	{
            ui_g_dynamic_zimiaohuokong->color = 6; // 恢复默认颜色
            ui_g_dirty_figure[22] = 1;
        
    }

	
	if(zimiao_open->key.v & KEY_PRESSED_OFFSET_Q)
	{
		ui_g_static_NewArc->color = 6;
		ui_g_dirty_figure[12] = 1;
	}
	else if(zimiao_open->key.v & KEY_PRESSED_OFFSET_E)
	{
		ui_g_static_NewArc->color = 2;
		ui_g_dirty_figure[12] = 1;
	}
	
	
	
	
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = 1;
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = 1;
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}