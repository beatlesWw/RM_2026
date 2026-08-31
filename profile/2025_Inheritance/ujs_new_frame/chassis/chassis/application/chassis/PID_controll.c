#include "PID_controll.h"
#include "controller.h"
#include <stdint.h>

Tumble_t tumble_ctrl = {0};
static PIDInstance Tumble_R_Inner;
static PIDInstance Tumble_L_Inner;
static PIDInstance Tumble_Sync_Outer;
static PIDInstance Tumble_Sync_Inner;

void TumbleRecover_Init(void)
{
    // 同步外环：对齐两腿phi0差值
    PID_Init_Config_s sync_outer_cfg = {
        .Kp = 50.5f, 
        .Ki = 0.0f, 
        .Kd = 0.0f,
        .MaxOut = 5.0f, 
        .IntegralLimit = 0.0f,
        .DeadBand = 0.0f, 
        .Improve = PID_IMPROVE_NONE
    };
    // 同步内环：输出同步修正力矩
    PID_Init_Config_s sync_inner_cfg = {
        .Kp = 1.5f, 
        .Ki = 0.1f, 
        .Kd = 0.0f,
        .MaxOut = 5.0f, 
        .IntegralLimit = 0.0f,
        .DeadBand = 0.0f, 
        .Improve = PID_IMPROVE_NONE
    };

    PIDInit(&Tumble_Sync_Outer, &sync_outer_cfg);
    PIDInit(&Tumble_Sync_Inner, &sync_inner_cfg);
}

//摆腿一致PID
float TumbleRecover_sync(vmc_leg_t *vmcr, vmc_leg_t *vmcl, float vel_r){
    tumble_ctrl.Tp_dphi0 = PID_calc(&Tumble_Sync_Outer, vmcl->phi0, vmcr->phi0);
    tumble_ctrl.Tp_sync  = PID_calc(&Tumble_Sync_Inner, vel_r, tumble_ctrl.Tp_dphi0);
    return  tumble_ctrl.Tp_sync;
}


uint8_t tumble_detect(attitude_t *INS,vmc_leg_t *leg)
{
    uint8_t tumble_ok=0;
    if(INS->Pitch<0.3&&INS->Pitch>-0.3&&leg->phi0<1.5&&leg->phi0>-1.5)
    {
    tumble_ok=1;
    }
    return tumble_ok;
}

// //TODO：后续换成检测摆杆倾角
// void TUMBLE_DETECT(attitude_t *ins,Balance_Chassis_e *chassis)
// {
//     // 处理pitch角度跳变（3.14 → -3.14）
//     float pitch_delta = ins->Pitch - tumble_ctrl.last_pitch_raw;
    
//     // 检测跳变：如果变化超过π，说明发生了-π到π的跳变
//     if (pitch_delta > 3.14159f) {
//         tumble_ctrl.pitch_continuous -= 6.28318f;  // 从π跳到-π，连续值减2π
//     } else if (pitch_delta < -3.14159f) {
//         tumble_ctrl.pitch_continuous += 6.28318f;  // 从-π跳到π，连续值加2π
//     }
    
//     // 更新连续pitch值
//     tumble_ctrl.pitch_continuous += pitch_delta;
//     tumble_ctrl.last_pitch_raw = ins->Pitch;
    
//     // 使用连续pitch的绝对值判断翻倒（|pitch| > 2.0 或 |pitch| 接近 π）
//     float abs_pitch = tumble_ctrl.pitch_continuous > 0 ? tumble_ctrl.pitch_continuous : -tumble_ctrl.pitch_continuous;
    
//     if (abs_pitch > 2.0f)  // 翻倒阈值：绝对值超过2.0 rad（约114度）
//     {
//         chassis->chassis_move_balance->recover_flag = 1;
//         chassis->chassis_move_balance->leg_set = 0.18;
//         chassis->chassis_move_balance->W_Strat = 1;
//     }
//     else 
//     {
//         chassis->chassis_move_balance->recover_flag = 0;
//     }   
// }