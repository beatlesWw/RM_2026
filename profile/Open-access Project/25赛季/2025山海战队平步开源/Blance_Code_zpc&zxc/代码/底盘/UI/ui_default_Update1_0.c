//
// Created by RM UI Designer
//

#include "ui_default_Update1_0.h"
#include "Remote.h"
#include "LinkCheck.h"
#include "Ultra_CAP.h"
#include "CToC.h"

#define FRAME_ID 0
#define GROUP_ID 3
#define START_ID 0
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Update1_0;
ui_interface_round_t *ui_default_Update1_Q_Round = (ui_interface_round_t *)&(ui_default_Update1_0.data[0]);
ui_interface_round_t *ui_default_Update1_VisualRound = (ui_interface_round_t *)&(ui_default_Update1_0.data[1]);
ui_interface_round_t *ui_default_Update1_B_Round = (ui_interface_round_t *)&(ui_default_Update1_0.data[2]);
ui_interface_round_t *ui_default_Update1_Z_Round = (ui_interface_round_t *)&(ui_default_Update1_0.data[3]);
ui_interface_rect_t *ui_default_Update1_SpeedRect = (ui_interface_rect_t *)&(ui_default_Update1_0.data[4]);
ui_interface_round_t *ui_default_Update1_RunRound = (ui_interface_round_t *)&(ui_default_Update1_0.data[5]);
ui_interface_arc_t *ui_default_Update1_CAP = (ui_interface_arc_t *)&(ui_default_Update1_0.data[6]);

void _ui_init_default_Update1_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Update1_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Update1_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Update1_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Update1_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Update1_0.data[i].operate_tpyel = 0;
    }

    ui_default_Update1_Q_Round->figure_tpye = 2;
    ui_default_Update1_Q_Round->layer = 0;
    ui_default_Update1_Q_Round->r = 15;
    ui_default_Update1_Q_Round->start_x = 1243;
    ui_default_Update1_Q_Round->start_y = 823;
    ui_default_Update1_Q_Round->color = 5;
    ui_default_Update1_Q_Round->width = 1;

    ui_default_Update1_VisualRound->figure_tpye = 2;
    ui_default_Update1_VisualRound->layer = 0;
    ui_default_Update1_VisualRound->r = 15;
    ui_default_Update1_VisualRound->start_x = 1280;
    ui_default_Update1_VisualRound->start_y = 781;
    ui_default_Update1_VisualRound->color = 5;
    ui_default_Update1_VisualRound->width = 1;

    ui_default_Update1_B_Round->figure_tpye = 2;
    ui_default_Update1_B_Round->layer = 0;
    ui_default_Update1_B_Round->r = 15;
    ui_default_Update1_B_Round->start_x = 640;
    ui_default_Update1_B_Round->start_y = 781;
    ui_default_Update1_B_Round->color = 5;
    ui_default_Update1_B_Round->width = 1;

    ui_default_Update1_Z_Round->figure_tpye = 2;
    ui_default_Update1_Z_Round->layer = 0;
    ui_default_Update1_Z_Round->r = 15;
    ui_default_Update1_Z_Round->start_x = 677;
    ui_default_Update1_Z_Round->start_y = 823;
    ui_default_Update1_Z_Round->color = 5;
    ui_default_Update1_Z_Round->width = 1;

    ui_default_Update1_SpeedRect->figure_tpye = 1;
    ui_default_Update1_SpeedRect->layer = 0;
    ui_default_Update1_SpeedRect->start_x = 922;
    ui_default_Update1_SpeedRect->start_y = 845;
    ui_default_Update1_SpeedRect->color = 5;
    ui_default_Update1_SpeedRect->width = 5;
    ui_default_Update1_SpeedRect->end_x = 992;
    ui_default_Update1_SpeedRect->end_y = 885;

    ui_default_Update1_RunRound->figure_tpye = 2;
    ui_default_Update1_RunRound->layer = 0;
    ui_default_Update1_RunRound->r = 15;
    ui_default_Update1_RunRound->start_x = 42;
    ui_default_Update1_RunRound->start_y = 842;
    ui_default_Update1_RunRound->color = 0;
    ui_default_Update1_RunRound->width = 1;

    ui_default_Update1_CAP->figure_tpye = 4;
    ui_default_Update1_CAP->layer = 0;
    ui_default_Update1_CAP->rx = 377;
    ui_default_Update1_CAP->ry = 377;
    ui_default_Update1_CAP->start_x = 960;
    ui_default_Update1_CAP->start_y = 540;
    ui_default_Update1_CAP->color = 6;
    ui_default_Update1_CAP->width = 20;
    ui_default_Update1_CAP->start_angle = 240;
    ui_default_Update1_CAP->end_angle = 300;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Update1_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Update1_0, sizeof(ui_default_Update1_0));
}

void _ui_update_default_Update1_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Update1_0.data[i].operate_tpyel = 2;
    }

	if(Remote_RxData.Remote_KeyPush_Q==0)
	{
		ui_default_Update1_Q_Round->r = 15;
		ui_default_Update1_Q_Round->width = 1;
	}
	else if(Remote_RxData.Remote_KeyPush_Q==1)
	{
		ui_default_Update1_Q_Round->r = 8;
		ui_default_Update1_Q_Round->width = 16;
	}

	if(Remote_RxData.Remote_Mouse_KeyR==0)
	{
		ui_default_Update1_VisualRound->r = 15;
		ui_default_Update1_VisualRound->width = 1;
	}
	else if(Visual_Find_Flag==1)
	{
		ui_default_Update1_VisualRound->r = 8;
		ui_default_Update1_VisualRound->width = 16;
	}

	if(Remote_RxData.Remote_KeyPush_B==0)
	{
		ui_default_Update1_B_Round->r = 15;
		ui_default_Update1_B_Round->width = 1;
	}
	else if(Remote_RxData.Remote_KeyPush_B==1)
	{
		ui_default_Update1_B_Round->r = 8;
		ui_default_Update1_B_Round->width = 16;
	}

	if(Remote_RxData.Remote_KeyPush_Z==0)
	{
		ui_default_Update1_Z_Round->r = 15;
		ui_default_Update1_Z_Round->width = 1;
	}
	else if(Remote_RxData.Remote_KeyPush_Z==1)
	{
		ui_default_Update1_Z_Round->r = 8;
		ui_default_Update1_Z_Round->width = 16;
	}

	if(Remote_RxData.Remote_LKnob>1600)//HIG
	{
		ui_default_Update1_SpeedRect->start_x = 1002;
		ui_default_Update1_SpeedRect->end_x = 1072;
	}
	else if(Remote_RxData.Remote_LKnob<-3200)//LOW
	{
		ui_default_Update1_SpeedRect->start_x = 842;
		ui_default_Update1_SpeedRect->end_x = 912;
	}
	else//MID
	{
		ui_default_Update1_SpeedRect->start_x = 922;
		ui_default_Update1_SpeedRect->end_x = 992;
	}

	if(LinkCheck_ErrorID==0)
	{
		ui_default_Update1_RunRound->r = 15;
		ui_default_Update1_RunRound->width = 1;
	}
	else if(LinkCheck_ErrorID==-1)
	{
		ui_default_Update1_RunRound->r = 8;
		ui_default_Update1_RunRound->width = 16;
	}

	float CAP_End=60.0f/255.0f*Ultra_CAP_Energy+240.0f;
    ui_default_Update1_CAP->end_angle = (int16_t)CAP_End;

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Update1_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Update1_0, sizeof(ui_default_Update1_0));
}
