//
// Created by RM UI Designer
//

#include "ui_default_Group2_4.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 4
#define OBJ_NUM 2
#define FRAME_OBJ_NUM 2

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Group2_4;
ui_interface_arc_t *ui_default_Group2_PitchRange = (ui_interface_arc_t *)&(ui_default_Group2_4.data[0]);
ui_interface_line_t *ui_default_Group2_PitchZero = (ui_interface_line_t *)&(ui_default_Group2_4.data[1]);

void _ui_init_default_Group2_4(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Group2_4.data[i].figure_name[0] = FRAME_ID;
        ui_default_Group2_4.data[i].figure_name[1] = GROUP_ID;
        ui_default_Group2_4.data[i].figure_name[2] = i + START_ID;
        ui_default_Group2_4.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Group2_4.data[i].operate_tpyel = 0;
    }

    ui_default_Group2_PitchRange->figure_tpye = 4;
    ui_default_Group2_PitchRange->layer = 0;
    ui_default_Group2_PitchRange->rx = 380;
    ui_default_Group2_PitchRange->ry = 380;
    ui_default_Group2_PitchRange->start_x = 960;
    ui_default_Group2_PitchRange->start_y = 540;
    ui_default_Group2_PitchRange->color = 2;
    ui_default_Group2_PitchRange->width = 20;
    ui_default_Group2_PitchRange->start_angle = 60;
    ui_default_Group2_PitchRange->end_angle = 150;

    ui_default_Group2_PitchZero->figure_tpye = 0;
    ui_default_Group2_PitchZero->layer = 0;
    ui_default_Group2_PitchZero->start_x = 1340;
    ui_default_Group2_PitchZero->start_y = 438;
    ui_default_Group2_PitchZero->end_x = 1359;
    ui_default_Group2_PitchZero->end_y = 433;
    ui_default_Group2_PitchZero->color = 8;
    ui_default_Group2_PitchZero->width = 6;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Group2_4);
    SEND_MESSAGE((uint8_t *) &ui_default_Group2_4, sizeof(ui_default_Group2_4));
}
