//
// Created by RM UI Designer
//

#include "ui_default_Update_0.h"
#include "Observer.h"
#include "arm_math.h"
#include "CToC.h"
#include "Chassis.h"

#define FRAME_ID 0
#define GROUP_ID 2
#define START_ID 0
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Update_0;
ui_interface_arc_t *ui_default_Update_ChassisAngle = (ui_interface_arc_t *)&(ui_default_Update_0.data[0]);
ui_interface_line_t *ui_default_Update_Body = (ui_interface_line_t *)&(ui_default_Update_0.data[1]);
ui_interface_line_t *ui_default_Update_Leg = (ui_interface_line_t *)&(ui_default_Update_0.data[2]);
ui_interface_number_t *ui_default_Update_PitchFloat = (ui_interface_number_t *)&(ui_default_Update_0.data[3]);
ui_interface_number_t *ui_default_Update_L0Float = (ui_interface_number_t *)&(ui_default_Update_0.data[4]);
ui_interface_line_t *ui_default_Update_Roll = (ui_interface_line_t *)&(ui_default_Update_0.data[5]);
ui_interface_arc_t *ui_default_Update_Pitch = (ui_interface_arc_t *)&(ui_default_Update_0.data[6]);

void _ui_init_default_Update_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Update_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Update_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Update_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Update_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Update_0.data[i].operate_tpyel = 0;
    }

    ui_default_Update_ChassisAngle->figure_tpye = 4;
    ui_default_Update_ChassisAngle->layer = 0;
    ui_default_Update_ChassisAngle->rx = 87;
    ui_default_Update_ChassisAngle->ry = 87;
    ui_default_Update_ChassisAngle->start_x = 960;
    ui_default_Update_ChassisAngle->start_y = 540;
    ui_default_Update_ChassisAngle->color = 4;
    ui_default_Update_ChassisAngle->width = 10;
    ui_default_Update_ChassisAngle->start_angle = 330;
    ui_default_Update_ChassisAngle->end_angle = 30;

    ui_default_Update_Body->figure_tpye = 0;
    ui_default_Update_Body->layer = 0;
    ui_default_Update_Body->start_x = 1550;
    ui_default_Update_Body->start_y = 785;
    ui_default_Update_Body->end_x = 1650;
    ui_default_Update_Body->end_y = 785;
    ui_default_Update_Body->color = 0;
    ui_default_Update_Body->width = 5;

    ui_default_Update_Leg->figure_tpye = 0;
    ui_default_Update_Leg->layer = 0;
    ui_default_Update_Leg->start_x = 1600;
    ui_default_Update_Leg->start_y = 675;
    ui_default_Update_Leg->end_x = 1600;
    ui_default_Update_Leg->end_y = 785;
    ui_default_Update_Leg->color = 0;
    ui_default_Update_Leg->width = 5;

    ui_default_Update_PitchFloat->figure_tpye = 5;
    ui_default_Update_PitchFloat->layer = 0;
    ui_default_Update_PitchFloat->font_size = 15;
    ui_default_Update_PitchFloat->start_x = 1350;
    ui_default_Update_PitchFloat->start_y = 415;
    ui_default_Update_PitchFloat->color = 8;
    ui_default_Update_PitchFloat->number = 12345;
    ui_default_Update_PitchFloat->width = 2;

    ui_default_Update_L0Float->figure_tpye = 5;
    ui_default_Update_L0Float->layer = 0;
    ui_default_Update_L0Float->font_size = 15;
    ui_default_Update_L0Float->start_x = 1645;
    ui_default_Update_L0Float->start_y = 675;
    ui_default_Update_L0Float->color = 0;
    ui_default_Update_L0Float->number = 12345;
    ui_default_Update_L0Float->width = 2;

    ui_default_Update_Roll->figure_tpye = 0;
    ui_default_Update_Roll->layer = 0;
    ui_default_Update_Roll->start_x = 960;
    ui_default_Update_Roll->start_y = 840;
    ui_default_Update_Roll->end_x = 960;
    ui_default_Update_Roll->end_y = 800;
    ui_default_Update_Roll->color = 8;
    ui_default_Update_Roll->width = 10;

    ui_default_Update_Pitch->figure_tpye = 4;
    ui_default_Update_Pitch->layer = 0;
    ui_default_Update_Pitch->rx = 376;
    ui_default_Update_Pitch->ry = 376;
    ui_default_Update_Pitch->start_x = 960;
    ui_default_Update_Pitch->start_y = 540;
    ui_default_Update_Pitch->color = 8;
    ui_default_Update_Pitch->width = 30;
    ui_default_Update_Pitch->start_angle = 100;
    ui_default_Update_Pitch->end_angle = 110;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Update_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Update_0, sizeof(ui_default_Update_0));
}

void _ui_update_default_Update_0(void) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Update_0.data[i].operate_tpyel = 2;
    }
	
	float Chassis_theta=360.0f-Observer_BalanceStatus.Body.Yaw_Theta/(2.0f*PI)*360.0f;
	float Chassis_StartAngle=Chassis_theta-30.0f;
	if(Chassis_StartAngle<0)Chassis_StartAngle+=360.0f;
	float Chassis_EndAngle=Chassis_theta+30.0f;
	if(Chassis_EndAngle>360.0f)Chassis_EndAngle-=360.0f;
    ui_default_Update_ChassisAngle->start_angle = (int16_t)Chassis_StartAngle;
    ui_default_Update_ChassisAngle->end_angle = (int16_t)Chassis_EndAngle;

	float LegBody_X=1600.0f+110.0f*arm_sin_f32(Observer_BalanceStatus.RightLeg.theta);
	float LegBody_Y=675.0f+110.0f*arm_cos_f32(Observer_BalanceStatus.RightLeg.theta);
	float Body_StartX=LegBody_X-50.0f*arm_cos_f32(Observer_BalanceStatus.Body.Pitch);
	float Body_StartY=LegBody_Y-50.0f*arm_sin_f32(Observer_BalanceStatus.Body.Pitch);
	float Body_EndX=LegBody_X+50.0f*arm_cos_f32(Observer_BalanceStatus.Body.Pitch);
	float Body_EndY=LegBody_Y+50.0f*arm_sin_f32(Observer_BalanceStatus.Body.Pitch);
	
    ui_default_Update_Body->start_x = (int16_t)Body_StartX;
    ui_default_Update_Body->start_y = (int16_t)Body_StartY;
    ui_default_Update_Body->end_x = (int16_t)Body_EndX;
    ui_default_Update_Body->end_y = (int16_t)Body_EndY;

    ui_default_Update_Leg->end_x = (int16_t)LegBody_X;
    ui_default_Update_Leg->end_y = (int16_t)LegBody_Y;

    ui_default_Update_PitchFloat->font_size = 15;
    ui_default_Update_PitchFloat->number = -Gimbal_Pitch*10;

    ui_default_Update_L0Float->font_size = 15;
    ui_default_Update_L0Float->number = (int16_t)(Observer_BalanceStatus.RightLeg.L_0*1000);

	float Roll_Position=235.0f/RollThreshold*Observer_BalanceStatus.Body.Roll+960.0f;
    ui_default_Update_Roll->start_x = (int16_t)Roll_Position;
    ui_default_Update_Roll->start_y = 840;
    ui_default_Update_Roll->end_x = (int16_t)Roll_Position;
    ui_default_Update_Roll->end_y = 800;

	float Pitch_theta=0.5f*(Gimbal_Pitch/100.0f)+105.0f;
	float Pitch_StartAngle=Pitch_theta-5.0f;
	float Pitch_EndAngle=Pitch_theta+5.0f;
    ui_default_Update_Pitch->start_angle = (int16_t)Pitch_StartAngle;
    ui_default_Update_Pitch->end_angle = (int16_t)Pitch_EndAngle;

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Update_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Update_0, sizeof(ui_default_Update_0));
}
