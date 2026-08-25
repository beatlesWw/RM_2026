//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"
#include "gimbal_behaviour.h"

extern ui_interface_figure_t ui_g_now_figures[23];
extern uint8_t ui_g_dirty_figure[23];
extern ui_interface_string_t ui_g_now_strings[3];
extern uint8_t ui_g_dirty_string[3];

#define ui_g_static_Line1 ((ui_interface_line_t*)&(ui_g_now_figures[0]))
#define ui_g_static_Line2 ((ui_interface_line_t*)&(ui_g_now_figures[1]))
#define ui_g_dynamic_number3 ((ui_interface_number_t*)&(ui_g_now_figures[2]))
#define ui_g_static1_midLine ((ui_interface_line_t*)&(ui_g_now_figures[3]))
#define ui_g_static1_upLine ((ui_interface_line_t*)&(ui_g_now_figures[4]))
#define ui_g_static1_downLine ((ui_interface_line_t*)&(ui_g_now_figures[5]))
#define ui_g_static1_NewNumber1 ((ui_interface_number_t*)&(ui_g_now_figures[6]))
#define ui_g_static1_NewNumber2 ((ui_interface_number_t*)&(ui_g_now_figures[7]))
#define ui_g_static1_NewLine3 ((ui_interface_line_t*)&(ui_g_now_figures[8]))
#define ui_g_static1_NewLine4 ((ui_interface_line_t*)&(ui_g_now_figures[9]))
#define ui_g_static2_NewLine5 ((ui_interface_line_t*)&(ui_g_now_figures[10]))
#define ui_g_static2_NewNumber6 ((ui_interface_number_t*)&(ui_g_now_figures[11]))
#define ui_g_static_NewArc ((ui_interface_arc_t*)&(ui_g_now_figures[12]))
#define ui_g_dynamic_NewRound ((ui_interface_round_t*)&(ui_g_now_figures[13]))
#define ui_g_dynamic_midLine ((ui_interface_line_t*)&(ui_g_now_figures[14]))
#define ui_g_static_shu ((ui_interface_line_t*)&(ui_g_now_figures[15]))
#define ui_g_static2_heng1 ((ui_interface_line_t*)&(ui_g_now_figures[16]))
#define ui_g_static2_heng2 ((ui_interface_line_t*)&(ui_g_now_figures[17]))
#define ui_g_static2_heng3 ((ui_interface_line_t*)&(ui_g_now_figures[18]))
#define ui_g_dynamic_number1 ((ui_interface_number_t*)&(ui_g_now_figures[19]))
#define ui_g_dynamic_number2 ((ui_interface_number_t*)&(ui_g_now_figures[20]))
#define ui_g_dynamic_zimiaojiaodu ((ui_interface_arc_t*)&(ui_g_now_figures[21]))
#define ui_g_dynamic_zimiaohuokong ((ui_interface_arc_t*)&(ui_g_now_figures[22]))

#define ui_g_static_Text3 (&(ui_g_now_strings[0]))
#define ui_g_static_Text1 (&(ui_g_now_strings[1]))
#define ui_g_static_Text2 (&(ui_g_now_strings[2]))

#ifdef MANUAL_DIRTY
#define ui_g_static_Line1_dirty (ui_g_dirty_figure[0])
#define ui_g_static_Line2_dirty (ui_g_dirty_figure[1])
#define ui_g_dynamic_number3_dirty (ui_g_dirty_figure[2])
#define ui_g_static1_midLine_dirty (ui_g_dirty_figure[3])
#define ui_g_static1_upLine_dirty (ui_g_dirty_figure[4])
#define ui_g_static1_downLine_dirty (ui_g_dirty_figure[5])
#define ui_g_static1_NewNumber1_dirty (ui_g_dirty_figure[6])
#define ui_g_static1_NewNumber2_dirty (ui_g_dirty_figure[7])
#define ui_g_static1_NewLine3_dirty (ui_g_dirty_figure[8])
#define ui_g_static1_NewLine4_dirty (ui_g_dirty_figure[9])
#define ui_g_static2_NewLine5_dirty (ui_g_dirty_figure[10])
#define ui_g_static2_NewNumber6_dirty (ui_g_dirty_figure[11])
#define ui_g_static_NewArc_dirty (ui_g_dirty_figure[12])
#define ui_g_dynamic_NewRound_dirty (ui_g_dirty_figure[13])
#define ui_g_dynamic_midLine_dirty (ui_g_dirty_figure[14])
#define ui_g_static_shu_dirty (ui_g_dirty_figure[15])
#define ui_g_static2_heng1_dirty (ui_g_dirty_figure[16])
#define ui_g_static2_heng2_dirty (ui_g_dirty_figure[17])
#define ui_g_static2_heng3_dirty (ui_g_dirty_figure[18])
#define ui_g_dynamic_number1_dirty (ui_g_dirty_figure[19])
#define ui_g_dynamic_number2_dirty (ui_g_dirty_figure[20])
#define ui_g_dynamic_zimiaojiaodu_dirty (ui_g_dirty_figure[21])
#define ui_g_dynamic_zimiaohuokong_dirty (ui_g_dirty_figure[22])

#define ui_g_static_Text3_dirty (ui_g_dirty_string[0])
#define ui_g_static_Text1_dirty (ui_g_dirty_string[1])
#define ui_g_static_Text2_dirty (ui_g_dirty_string[2])
#endif

void ui_init_g();
void ui_update_g();

#endif // UI_g_H