#include "string.h"
#include "cmsis_os.h"
#include "math.h"
#include "ui.h"
#include "gimbal_task.h"
#include "CAN_Receive.h"//加载摩擦轮发射闭环真实转速
#include "shoot.h"
#include "remote_control.h"
const RC_ctrl_t *mirror_rc1;
void user_task(void const *pvParameters)
{     
  while(1)
 {
     mirror_rc1 = get_remote_control_point();
    if(mirror_rc1->key.v & KEY_PRESSED_OFFSET_G)
     ui_init_g();    
    else
    ui_update_g();     
     osDelay(1);
  }
}