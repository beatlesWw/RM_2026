//
// Created by RM UI Designer
//

#include "ui_default_Change_0.h"

#define ZhunXin_X			960
#define ZhunXin_Y			505
#define Jump_Y				325

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 0
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Change_0;
ui_interface_line_t *ui_default_Change_JumpLine = (ui_interface_line_t *)&(ui_default_Change_0.data[0]);
ui_interface_round_t *ui_default_Change_ZhunXinRound = (ui_interface_round_t *)&(ui_default_Change_0.data[1]);
ui_interface_line_t *ui_default_Change_ZhunXinLine1 = (ui_interface_line_t *)&(ui_default_Change_0.data[2]);
ui_interface_line_t *ui_default_Change_ZhunXinLine2 = (ui_interface_line_t *)&(ui_default_Change_0.data[3]);
ui_interface_line_t *ui_default_Change_ZhunXinLine3 = (ui_interface_line_t *)&(ui_default_Change_0.data[4]);
ui_interface_line_t *ui_default_Change_ZhunXinLine4 = (ui_interface_line_t *)&(ui_default_Change_0.data[5]);
ui_interface_round_t *ui_default_Change_Wheel = (ui_interface_round_t *)&(ui_default_Change_0.data[6]);

void _ui_init_default_Change_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Change_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Change_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Change_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Change_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Change_0.data[i].operate_tpyel = 0;
    }

    ui_default_Change_JumpLine->figure_tpye = 0;
    ui_default_Change_JumpLine->layer = 0;
    ui_default_Change_JumpLine->start_x = 700;
    ui_default_Change_JumpLine->start_y = Jump_Y;
    ui_default_Change_JumpLine->end_x = 1221;
    ui_default_Change_JumpLine->end_y = Jump_Y;
    ui_default_Change_JumpLine->color = 8;
    ui_default_Change_JumpLine->width = 2;

    ui_default_Change_ZhunXinRound->figure_tpye = 2;
    ui_default_Change_ZhunXinRound->layer = 0;
    ui_default_Change_ZhunXinRound->r = 1;
    ui_default_Change_ZhunXinRound->start_x = ZhunXin_X-1;
    ui_default_Change_ZhunXinRound->start_y = ZhunXin_Y-1;
    ui_default_Change_ZhunXinRound->color = 8;
    ui_default_Change_ZhunXinRound->width = 3;

    ui_default_Change_ZhunXinLine1->figure_tpye = 0;
    ui_default_Change_ZhunXinLine1->layer = 0;
    ui_default_Change_ZhunXinLine1->start_x = ZhunXin_X;
    ui_default_Change_ZhunXinLine1->start_y = ZhunXin_Y+10;
    ui_default_Change_ZhunXinLine1->end_x = ZhunXin_X;
    ui_default_Change_ZhunXinLine1->end_y = ZhunXin_Y+30;
    ui_default_Change_ZhunXinLine1->color = 8;
    ui_default_Change_ZhunXinLine1->width = 2;

    ui_default_Change_ZhunXinLine2->figure_tpye = 0;
    ui_default_Change_ZhunXinLine2->layer = 0;
    ui_default_Change_ZhunXinLine2->start_x = ZhunXin_X+10;
    ui_default_Change_ZhunXinLine2->start_y = ZhunXin_Y;
    ui_default_Change_ZhunXinLine2->end_x = ZhunXin_X+30;
    ui_default_Change_ZhunXinLine2->end_y = ZhunXin_Y;
    ui_default_Change_ZhunXinLine2->color = 8;
    ui_default_Change_ZhunXinLine2->width = 2;

    ui_default_Change_ZhunXinLine3->figure_tpye = 0;
    ui_default_Change_ZhunXinLine3->layer = 0;
    ui_default_Change_ZhunXinLine3->start_x = ZhunXin_X;
    ui_default_Change_ZhunXinLine3->start_y = ZhunXin_Y-30;
    ui_default_Change_ZhunXinLine3->end_x = ZhunXin_X;
    ui_default_Change_ZhunXinLine3->end_y = ZhunXin_Y-10;
    ui_default_Change_ZhunXinLine3->color = 8;
    ui_default_Change_ZhunXinLine3->width = 2;

    ui_default_Change_ZhunXinLine4->figure_tpye = 0;
    ui_default_Change_ZhunXinLine4->layer = 0;
    ui_default_Change_ZhunXinLine4->start_x = ZhunXin_X-30;
    ui_default_Change_ZhunXinLine4->start_y = ZhunXin_Y;
    ui_default_Change_ZhunXinLine4->end_x = ZhunXin_X-10;
    ui_default_Change_ZhunXinLine4->end_y = ZhunXin_Y;
    ui_default_Change_ZhunXinLine4->color = 8;
    ui_default_Change_ZhunXinLine4->width = 2;

    ui_default_Change_Wheel->figure_tpye = 2;
    ui_default_Change_Wheel->layer = 0;
    ui_default_Change_Wheel->r = 31;
    ui_default_Change_Wheel->start_x = 1600;
    ui_default_Change_Wheel->start_y = 675;
    ui_default_Change_Wheel->color = 0;
    ui_default_Change_Wheel->width = 5;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Change_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Change_0, sizeof(ui_default_Change_0));
}
