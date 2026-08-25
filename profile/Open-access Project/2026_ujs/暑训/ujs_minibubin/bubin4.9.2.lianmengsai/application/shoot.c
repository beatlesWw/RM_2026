/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "AutoGimbal.h"
#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮
#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin) //I7

shoot_control_t shoot_control;          //射击数据
extern power_heat_data_t power_heat_data_t1;
extern robot_status_t robot_state;
// 初始化射击模块
void shoot_init(void)
{
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 fric_speed_pid[2][3] = {{FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD},
                                              {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD}};

    shoot_control.shoot_mode = SHOOT_STOP;
    shoot_control.shoot_rc = get_remote_control_point();
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    shoot_control.fricL_motor_measure = get_fricL_motor_measure_point();
    shoot_control.fricR_motor_measure = get_fricR_motor_measure_point();

    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_motor_L_pid, PID_POSITION, fric_speed_pid[0], FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_motor_R_pid, PID_POSITION, fric_speed_pid[1], FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);

    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
    shoot_control.press_l_time = 0;
    shoot_control.rc_s_time = 0;
    shoot_control.block_time = 0;
    shoot_control.reverse_time = 0;
    shoot_control.key = 0;
    shoot_control.heat_limit = 0;
    shoot_control.heat = 0;

    shoot_control.fric_enabled = 0;  // 默认关闭摩擦轮
    SHOOT_ON_KEYBOARD;
}

// 射击数据更新
static void shoot_feedback_update(void)
{
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;
    shoot_control.fricL_speed = shoot_control.fricL_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
    shoot_control.fricR_speed = shoot_control.fricR_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;

    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }
    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    shoot_control.key = BUTTEN_TRIG_PIN;

    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;

  
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }
}

// 堵转倒转处理
static void trigger_motor_turn_back(void)
{
    if (shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

// 射击控制，控制拨弹电机角度，完成一次发射
static void shoot_bullet_control(void)
{
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
        shoot_control.move_flag = 1;
    }
    if (shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_DONE;
    }
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
    }
}

// 射击状态机设置
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;


     //上拨判断， 一次开启，再次关闭
//    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
//    {
//        
//        shoot_control.shoot_mode = SHOOT_STOP;
//    }

CTRL *ctrl = get_AUTO_control_point();  // 获取结构体指针

if (switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
{
    if (ctrl->mode == 1)  
    {
       shoot_control.shoot_mode = SHOOT_READY_BULLET;
        shoot_control.fric_set = BULLET_SPEED;
    }
    else if(ctrl->mode == 0)
    {
        shoot_control.given_current = 0;
        shoot_control.fric_set = 0;
    }
}
    
    // 中档，使用键盘V开启或关闭摩擦轮
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control.fric_set = BULLET_SPEED;

        if ( SHOOT_ON_KEYBOARD )
      {
         if( shoot_control.key == SWITCH_TRIGGER_ON)//拨弹开关开启
         {
             shoot_control.shoot_mode = SHOOT_READY;
         }
         else if(shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)//拨弹开关关闭
        {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
        }

        // 当鼠标左键按下时，直接设置射击模式为 SHOOT_READY_BULLET
        if (shoot_control.press_l)
        {
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
			 
        }
        else
        {
            shoot_control.shoot_mode =  SHOOT_STOP;
			
            shoot_control.fric_set = BULLET_SPEED;
        }
      }
       else if ( SHOOT_OFF_KEYBOARD )
       {
            shoot_control.shoot_mode =  SHOOT_STOP;
            shoot_control.fric_set = BULLET_SPEED;
       }
       shoot_control.fric_set = BULLET_SPEED;
    }
    
    if (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
		shoot_control.fric_set = BULLET_SPEED;
    }

    
    
    // 射击完成状态处理
    if (shoot_control.shoot_mode == SHOOT_DONE)
    {
        if (shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            shoot_control.key_time++;
            if (shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
            {
                shoot_control.key_time = 0;
                shoot_control.shoot_mode = SHOOT_READY_BULLET;
            }
        }
        else
        {
            shoot_control.key_time = 0;
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }
//    //热量检查直接退出射击循环
//    if (!toe_is_error(REFEREE_TOE) && (shoot_control.heat + BULLET_HEAT_BEST > shoot_control.heat_limit)) 
//	{
//        shoot_control.shoot_mode = SHOOT_STOP;
//    }
    
    
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}

extern int board_receive_data[8];
// 射击循环
int16_t shoot_control_loop(void)
{
    shoot_set_mode();
    shoot_feedback_update();
   

    switch (shoot_control.shoot_mode)
    {
        case SHOOT_STOP:
            shoot_control.speed_set = 0.0f;
            shoot_laser_off();
            shoot_control.given_current = 0;
            shoot_control.fric_set = 0;
            shoot_control.key = 0;
            break;
        case SHOOT_READY_BULLET:
            if (shoot_control.key == SWITCH_TRIGGER_OFF)
            {
                shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
                trigger_motor_turn_back();
            }
            else
            {
                shoot_control.trigger_speed_set = 0.0f;
                shoot_control.speed_set = 0.0f;
            }
            shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
            shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
            shoot_laser_on();
            shoot_control.fric_set = BULLET_SPEED;
            break;
        case SHOOT_READY:
            shoot_control.speed_set = 0.0f;
            shoot_laser_on();
            shoot_control.fric_set = BULLET_SPEED;
            break;
        case SHOOT_BULLET:
            shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
            shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
            shoot_bullet_control();
            shoot_laser_on();
            shoot_control.fric_set = BULLET_SPEED;
            break;
        case SHOOT_DONE:
            shoot_control.speed_set = 0.0f;
            shoot_control.fric_set = 0;
            shoot_laser_on();
            break;
        default:
            break;
    }
    PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
    shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
    
    if (shoot_control.shoot_mode < SHOOT_READY_BULLET)
    {
        shoot_control.given_current = 0;
    }
   
     if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
     {
        
	 //q键按下一下开启
	if ((shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && 
        !(shoot_control.last_key & SHOOT_ON_KEYBOARD)) 
    {
        shoot_control.fric_enabled = 1;
    }
    //e键按下一下关闭
    if ((shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && 
        !(shoot_control.last_key & SHOOT_OFF_KEYBOARD)) 
    {
        shoot_control.fric_enabled = 0;
    }
	 // 保存按键状态
    shoot_control.last_key = shoot_control.shoot_rc->key.v; 

    if (shoot_control.fric_enabled)
    {
        shoot_control.fric_set = BULLET_SPEED;
    }
    else
    {
        shoot_control.fric_set = 0;
    }
}
	 
	
    PID_calc(&shoot_control.fric_motor_L_pid, shoot_control.fricL_speed, -shoot_control.fric_set);
    PID_calc(&shoot_control.fric_motor_R_pid, shoot_control.fricR_speed, shoot_control.fric_set);
    shoot_control.fric_l_current = (int16_t)(shoot_control.fric_motor_L_pid.out);
    shoot_control.fric_r_current = (int16_t)(shoot_control.fric_motor_R_pid.out);
    

   
//    	//热量检查直接退出射击循环
    if ((power_heat_data_t1.shooter_17mm_1_barrel_heat + BULLET_HEAT_BEST > robot_state.shooter_barrel_heat_limit)) 
	{

		shoot_control.given_current=0;
    }
		

    if (board_receive_data[1] == 1)
             {
                CAN_cmd_fric(shoot_control.fric_l_current, shoot_control.fric_r_current, 0, 0);
             }
    else if(board_receive_data[1] == 0)
             {
                CAN_cmd_fric(0, 0, 0, 0);
             }

    return shoot_control.given_current;
}    