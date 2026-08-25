#include "spring_comp.h"
#include "VMC.h"
#include "arm_math.h"
#include "PID_controll.h"
#include <math.h>

SpringCompParam_t spring_comp_r = {
    .l1 = 0.21f,//m
    .l2 = 0.25f,
    .l3 = 0.085f,
    .l4 = 0.1155f,
    .as = 0.0f,
    .Fs = 300.0f,//N
    .Fv = 0.0f,
    .s2 = 0.05f,
    .s3 = 0.0f,
    .leg_theta =0.0f,
    .ls = 0.10f,

};

SpringCompParam_t spring_comp_l = {
    .l1 = 0.21f,//m
    .l2 = 0.25f,
    .l3 = 0.085f,
    .l4 = 0.1155f,
    .as = 0.0f,
    .Fs = 300.0f,//N
    .Fv = 0.0f,
    .s2 = 0.05f,
    .s3 = 0.0f,
    .leg_theta =0.0f,
    .ls = 0.10f,

};

//跨越逻辑
uint8_t LegFold_Override(vmc_leg_t *vmc, float *LQR_K, float leg_tp, PIDInstance *LegPid, SpringSide_e side)
{
//左右腿独立状态，以side为下标索引
    static uint8_t recording[2]   = {0, 0};
    static uint8_t active[2]      = {0, 0};
    static float   phi0_prev[2]   = {0.0f, 0.0f};
    static float   phi0_accum[2]  = {0.0f, 0.0f};
    static float   max_L0[2]      = {0.0f, 0.0f};  // 记录跨越过程中的最高腿长

    uint8_t idx   = (uint8_t)side;
    float   phi0  = vmc->phi0;

//记录阶段：L0>0.3 就开始（或继续）累积 phi0 连续角度
    if (vmc->L0 > 0.3f) {
        if (!recording[idx]) {
//首次满足，初始化记录起点
            recording[idx]   = 1;
            phi0_prev[idx]   = phi0;
            phi0_accum[idx]  = phi0;
        } else {
//跳变补偿，持续累积 phi0
            float d_phi0 = phi0 - phi0_prev[idx];
            if      (d_phi0 < -3.14159265f) d_phi0 += 6.28318530f;
            else if (d_phi0 >  3.14159265f) d_phi0 -= 6.28318530f;
            phi0_accum[idx] += d_phi0;
            phi0_prev[idx]   = phi0;
        }
    } else {
//L0 降到 0.3 以下，停止记录
        recording[idx]   = 0;
    }

//控制激活：phi0>2.0 且 L0>0.32
    if (!active[idx]) {
        if (phi0 > 2.0f && vmc->L0 > 0.32f) {
            active[idx] = 1;
            max_L0[idx] = vmc->L0;  // 初始化最高腿长
        } else {
            return 0;
        }
    }
    
    // 跨越过程中持续更新最高腿长
    if (vmc->L0 > max_L0[idx]) {
        max_L0[idx] = vmc->L0;
    }

//退出条件：theta_accum到达1.2
    if (vmc->theta_accum >= 1.2f) {
        recording[idx]   = 0;
        active[idx]      = 0;
        phi0_prev[idx]   = 0.0f;
        phi0_accum[idx]  = 0.0f;
        max_L0[idx]      = 0.0f;  // 重置最高腿长
        LegPid->Iout     = 0.0f;
        return 0;  // 返回0表示退出跨越模式
    }

    float tp_sign = (side == SPRING_SIDE_RIGHT) ?  1.0f : -1.0f;
    float f0_sign = (side == SPRING_SIDE_RIGHT) ? -1.0f :  1.0f;

    vmc->wheel_set = 0.0f;
    vmc->Tp = tp_sign * (LQR_K[6]*(vmc->theta_accum - 2.0f) + LQR_K[7]*(vmc->d_theta-0.0f) + leg_tp);
    // 跨越过程中维持最高腿长
    vmc->F0 = f0_sign * PID_calc(LegPid, vmc->L0, max_L0[idx]);

//限幅
    if (vmc->F0 >  500.0f) vmc->F0 =  500.0f;
    else if (vmc->F0 < -500.0f) vmc->F0 = -500.0f;

    VMC_calc_splitter(vmc);
    return 1;
}

float Fv_Gas_spring(vmc_leg_t *leg,SpringCompParam_t *spring){

    // 计算 leg_theta，限制 acos 参数在 [-1, 1] 范围内
    float cos_leg_theta = ((spring->l1*spring->l1) + (spring->l2*spring->l2) - leg->L0*leg->L0) / (2.0f*spring->l1*spring->l2);
    if (cos_leg_theta > 1.0f) cos_leg_theta = 1.0f;
    if (cos_leg_theta < -1.0f) cos_leg_theta = -1.0f;
    
    spring->leg_theta = acosf(cos_leg_theta);
    
    // 计算 s3
    spring->s3 = sqrtf((spring->l3*spring->l3) + (spring->l4*spring->l4) - 2.0f*arm_cos_f32(spring->leg_theta)*spring->l3*spring->l4);
    
    // 计算 as，限制 asin 参数在 [-1, 1] 范围内
    float angle_173_deg = 173.0f * 3.14159265f / 180.0f;
    float sin_arg = (spring->ls/spring->s3) * arm_sin_f32(angle_173_deg - spring->leg_theta);
    if (sin_arg > 1.0f) sin_arg = 1.0f;
    if (sin_arg < -1.0f) sin_arg = -1.0f;

    spring->as = asinf(sin_arg);
    
    // 计算 ls
    spring->ls = sqrtf(spring->s3*spring->s3 + spring->s2*spring->s2 - 2.0f*spring->s2*spring->s3*arm_cos_f32(spring->leg_theta - spring->as));
    
    // 计算 Fv，防止除零
    float sin_leg_theta = arm_sin_f32(spring->leg_theta);
    if (fabsf(sin_leg_theta) < 0.01f) sin_leg_theta = 0.01f;  
    if (spring->ls < 0.01f) spring->ls = 0.01f; 
    
    spring->Fv = spring->Fs * ((spring->s2*spring->s3*arm_sin_f32(spring->leg_theta - spring->as))
                          / (sin_leg_theta*spring->l1*spring->l2)) * (leg->L0/spring->ls);
    return spring->Fv;
}

