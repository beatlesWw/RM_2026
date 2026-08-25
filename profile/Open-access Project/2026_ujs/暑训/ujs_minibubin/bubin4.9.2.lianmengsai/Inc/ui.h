//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h"
#include "ui_g.h"
#include "remote_control.h"
#include "user_lib.h"
#define ui_begin       KEY_PRESSED_OFFSET_F	
#ifdef __cplusplus
}
#endif
void user_task(void const * argument);
#endif //UI_H
