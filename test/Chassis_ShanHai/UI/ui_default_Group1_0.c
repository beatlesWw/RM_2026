//
// Created by RM UI Designer
//

#include "ui_default_Group1_0.h"

#define FRAME_ID 0
#define GROUP_ID 4
#define START_ID 0
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Group1_0;
ui_interface_line_t *ui_default_Group1_CAPLine1 = (ui_interface_line_t *)&(ui_default_Group1_0.data[0]);
ui_interface_line_t *ui_default_Group1_CAPLine2 = (ui_interface_line_t *)&(ui_default_Group1_0.data[1]);
ui_interface_line_t *ui_default_Group1_CAPLine3 = (ui_interface_line_t *)&(ui_default_Group1_0.data[2]);
ui_interface_line_t *ui_default_Group1_RollRange = (ui_interface_line_t *)&(ui_default_Group1_0.data[3]);
ui_interface_arc_t *ui_default_Group1_CAPRange1 = (ui_interface_arc_t *)&(ui_default_Group1_0.data[4]);
ui_interface_arc_t *ui_default_Group1_CAPRange2 = (ui_interface_arc_t *)&(ui_default_Group1_0.data[5]);
ui_interface_arc_t *ui_default_Group1_CAPRange3 = (ui_interface_arc_t *)&(ui_default_Group1_0.data[6]);

void _ui_init_default_Group1_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Group1_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Group1_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Group1_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Group1_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Group1_0.data[i].operate_tpyel = 0;
    }

    ui_default_Group1_CAPLine1->figure_tpye = 0;
    ui_default_Group1_CAPLine1->layer = 0;
    ui_default_Group1_CAPLine1->start_x = 595;
    ui_default_Group1_CAPLine1->start_y = 540;
    ui_default_Group1_CAPLine1->end_x = 615;
    ui_default_Group1_CAPLine1->end_y = 540;
    ui_default_Group1_CAPLine1->color = 8;
    ui_default_Group1_CAPLine1->width = 8;

    ui_default_Group1_CAPLine2->figure_tpye = 0;
    ui_default_Group1_CAPLine2->layer = 0;
    ui_default_Group1_CAPLine2->start_x = 603;
    ui_default_Group1_CAPLine2->start_y = 464;
    ui_default_Group1_CAPLine2->end_x = 622;
    ui_default_Group1_CAPLine2->end_y = 468;
    ui_default_Group1_CAPLine2->color = 8;
    ui_default_Group1_CAPLine2->width = 8;

    ui_default_Group1_CAPLine3->figure_tpye = 0;
    ui_default_Group1_CAPLine3->layer = 0;
    ui_default_Group1_CAPLine3->start_x = 626;
    ui_default_Group1_CAPLine3->start_y = 392;
    ui_default_Group1_CAPLine3->end_x = 644;
    ui_default_Group1_CAPLine3->end_y = 400;
    ui_default_Group1_CAPLine3->color = 8;
    ui_default_Group1_CAPLine3->width = 8;

    ui_default_Group1_RollRange->figure_tpye = 0;
    ui_default_Group1_RollRange->layer = 0;
    ui_default_Group1_RollRange->start_x = 725;
    ui_default_Group1_RollRange->start_y = 820;
    ui_default_Group1_RollRange->end_x = 1195;
    ui_default_Group1_RollRange->end_y = 820;
    ui_default_Group1_RollRange->color = 1;
    ui_default_Group1_RollRange->width = 15;

    ui_default_Group1_CAPRange1->figure_tpye = 4;
    ui_default_Group1_CAPRange1->layer = 0;
    ui_default_Group1_CAPRange1->rx = 390;
    ui_default_Group1_CAPRange1->ry = 390;
    ui_default_Group1_CAPRange1->start_x = 960;
    ui_default_Group1_CAPRange1->start_y = 540;
    ui_default_Group1_CAPRange1->color = 8;
    ui_default_Group1_CAPRange1->width = 6;
    ui_default_Group1_CAPRange1->start_angle = 240;
    ui_default_Group1_CAPRange1->end_angle = 300;

    ui_default_Group1_CAPRange2->figure_tpye = 4;
    ui_default_Group1_CAPRange2->layer = 0;
    ui_default_Group1_CAPRange2->rx = 373;
    ui_default_Group1_CAPRange2->ry = 373;
    ui_default_Group1_CAPRange2->start_x = 960;
    ui_default_Group1_CAPRange2->start_y = 540;
    ui_default_Group1_CAPRange2->color = 8;
    ui_default_Group1_CAPRange2->width = 40;
    ui_default_Group1_CAPRange2->start_angle = 300;
    ui_default_Group1_CAPRange2->end_angle = 303;

    ui_default_Group1_CAPRange3->figure_tpye = 4;
    ui_default_Group1_CAPRange3->layer = 0;
    ui_default_Group1_CAPRange3->rx = 373;
    ui_default_Group1_CAPRange3->ry = 373;
    ui_default_Group1_CAPRange3->start_x = 960;
    ui_default_Group1_CAPRange3->start_y = 540;
    ui_default_Group1_CAPRange3->color = 8;
    ui_default_Group1_CAPRange3->width = 40;
    ui_default_Group1_CAPRange3->start_angle = 237;
    ui_default_Group1_CAPRange3->end_angle = 240;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Group1_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Group1_0, sizeof(ui_default_Group1_0));
}
