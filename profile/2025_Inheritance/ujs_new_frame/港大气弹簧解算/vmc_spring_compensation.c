/**
 * @file vmc_spring_compensation.c
 * @brief VMC气弹簧补偿力计算函数
 * @author Generated from Python VMC Visualizer
 * @date 2026-03-09
 * 
 * 功能：计算五连杆机构中气弹簧对虚拟腿的等效补偿力
 * 原理：基于虚功原理，将弹簧力通过雅可比矩阵映射到虚拟腿方向
 */

#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 五连杆机构参数结构体
 */
typedef struct {
    float L1;              // 大腿1长度 (m)
    float L2;              // 小腿1长度 (m)
    float L3;              // 小腿2长度 (m)
    float L4;              // 大腿2长度 (m)
    float spring_length;   // 弹簧力臂长度 (m)
    float spring_force;    // 弹簧力大小 (N)
} VMC_LinkageParams;

/**
 * @brief 关节状态结构体
 */
typedef struct {
    float Q1;              // 关节1角度 (rad)
    float Q4;              // 关节4角度 (rad)
} VMC_JointState;

/**
 * @brief 虚拟腿状态结构体
 */
typedef struct {
    float L0;              // 虚拟腿长度 (m)
    float Q0;              // 虚拟腿角度 (rad)
    float Cx;              // 足端C点x坐标 (m)
    float Cy;              // 足端C点y坐标 (m)
} VMC_VirtualLeg;

/**
 * @brief 弹簧补偿输出结构体
 */
typedef struct {
    float F_L0;            // 虚拟腿方向的等效力 (N)
    float F_Cx;            // 足端x方向的等效力 (N)
    float F_Cy;            // 足端y方向的等效力 (N)
    float tau_spring_DC;   // 弹簧对DC杆的力矩 (N*m)
    int valid;             // 计算是否有效 (1=有效, 0=无效)
} VMC_SpringCompensation;

/**
 * @brief 计算虚拟腿长度L0（辅助函数）
 * 
 * @param Q1 关节1角度 (rad)
 * @param Q4 关节4角度 (rad)
 * @param params 连杆参数
 * @param L0_out 输出虚拟腿长度 (m)
 * @return int 1=成功, 0=失败
 */
static int calc_L0(float Q1, float Q4, const VMC_LinkageParams *params, float *L0_out) {
    float Q4_adj = (Q1 >= Q4) ? Q4 : (Q4 - 2.0f * M_PI);
    
    float AE = params->L4 * cosf((Q1 - Q4_adj) / 2.0f);
    float sin_half = sinf((Q1 - Q4_adj) / 2.0f);
    float EC_sq = params->L3 * params->L3 - (sin_half * params->L4) * (sin_half * params->L4);
    
    if (EC_sq < 0.0f) {
        return 0;
    }
    
    float EC = sqrtf(EC_sq);
    *L0_out = AE + EC;
    return 1;
}

/**
 * @brief 计算弹簧作用点S在AD方向的投影（辅助函数）
 * 
 * @param Q1 关节1角度 (rad)
 * @param Q4 关节4角度 (rad)
 * @param params 连杆参数
 * @param S_proj_out 输出S点在AD方向的投影
 * @return int 1=成功, 0=失败
 */
static int calc_S_dot_AD(float Q1, float Q4, const VMC_LinkageParams *params, float *S_proj_out) {
    float Q4_adj = (Q1 >= Q4) ? Q4 : (Q4 - 2.0f * M_PI);
    
    // 计算C点位置
    float AE = params->L4 * cosf((Q1 - Q4_adj) / 2.0f);
    float sin_half = sinf((Q1 - Q4_adj) / 2.0f);
    float EC_sq = params->L3 * params->L3 - (sin_half * params->L4) * (sin_half * params->L4);
    
    if (EC_sq < 0.0f) {
        return 0;
    }
    
    float EC = sqrtf(EC_sq);
    float AC = AE + EC;
    
    float Cx = AC * cosf((Q1 + Q4_adj) / 2.0f);
    float Cy = AC * sinf((Q1 + Q4_adj) / 2.0f);
    
    // 计算D点位置
    float Dx = params->L4 * cosf(Q4_adj);
    float Dy = params->L4 * sinf(Q4_adj);
    
    // DC向量
    float DCx = Cx - Dx;
    float DCy = Cy - Dy;
    float DC_len = sqrtf(DCx * DCx + DCy * DCy);
    
    if (DC_len < 1e-6f) {
        return 0;
    }
    
    // S点位置（在DC上，距D点spring_length）
    float Sx = Dx + DCx / DC_len * params->spring_length;
    float Sy = Dy + DCy / DC_len * params->spring_length;
    
    // AD单位向量
    float AD_len = sqrtf(Dx * Dx + Dy * Dy);
    if (AD_len < 1e-6f) {
        return 0;
    }
    
    float AD_unit_x = Dx / AD_len;
    float AD_unit_y = Dy / AD_len;
    
    // S点在AD方向的投影
    *S_proj_out = Sx * AD_unit_x + Sy * AD_unit_y;
    return 1;
}

/**
 * @brief 数值计算偏导数 dL0/dQ4 和 d(S·AD)/dQ4
 * 
 * @param Q1 关节1角度 (rad)
 * @param Q4 关节4角度 (rad)
 * @param params 连杆参数
 * @param dL0_dQ4 输出 dL0/dQ4
 * @param dS_dQ4 输出 d(S·AD)/dQ4
 * @return int 1=成功, 0=失败
 */
static int calc_partials_Q4(float Q1, float Q4, const VMC_LinkageParams *params, 
                            float *dL0_dQ4, float *dS_dQ4) {
    const float delta = 1e-6f;
    
    // 计算 dL0/dQ4
    float L0_plus, L0_minus;
    if (!calc_L0(Q1, Q4 + delta, params, &L0_plus) || 
        !calc_L0(Q1, Q4 - delta, params, &L0_minus)) {
        return 0;
    }
    *dL0_dQ4 = (L0_plus - L0_minus) / (2.0f * delta);
    
    // 计算 d(S·AD)/dQ4
    float S_plus, S_minus;
    if (!calc_S_dot_AD(Q1, Q4 + delta, params, &S_plus) || 
        !calc_S_dot_AD(Q1, Q4 - delta, params, &S_minus)) {
        return 0;
    }
    *dS_dQ4 = (S_plus - S_minus) / (2.0f * delta);
    
    return 1;
}

/**
 * @brief 正运动学求解（计算虚拟腿状态）
 * 
 * @param joint 关节状态
 * @param params 连杆参数
 * @param vleg 输出虚拟腿状态
 * @return int 1=成功, 0=失败
 */
int VMC_ForwardKinematics(const VMC_JointState *joint, 
                          const VMC_LinkageParams *params,
                          VMC_VirtualLeg *vleg) {
    if (!joint || !params || !vleg) {
        return 0;
    }
    
    float Q4_adj = (joint->Q1 >= joint->Q4) ? joint->Q4 : (joint->Q4 - 2.0f * M_PI);
    
    // 计算虚拟腿长度
    float AE = params->L4 * cosf((joint->Q1 - Q4_adj) / 2.0f);
    float sin_half = sinf((joint->Q1 - Q4_adj) / 2.0f);
    float EC_sq = params->L3 * params->L3 - (sin_half * params->L4) * (sin_half * params->L4);
    
    if (EC_sq < 0.0f) {
        return 0;
    }
    
    float EC = sqrtf(EC_sq);
    float AC = AE + EC;
    
    vleg->L0 = AC;
    vleg->Q0 = (joint->Q1 + Q4_adj) / 2.0f;
    vleg->Cx = AC * cosf(vleg->Q0);
    vleg->Cy = AC * sinf(vleg->Q0);
    
    return 1;
}

/**
 * @brief 计算气弹簧补偿力（核心函数）
 * 
 * 基于虚功原理：F_spring × d(S·AD)/dQ4 = F_L0 × dL0/dQ4
 * 
 * @param joint 关节状态（输入）
 * @param params 连杆参数（输入）
 * @param compensation 补偿力输出
 * @return int 1=成功, 0=失败
 * 
 * @note 使用方法：
 *       1. 准备关节状态和连杆参数
 *       2. 调用此函数计算补偿力
 *       3. 使用 compensation->F_L0 作为虚拟腿方向的补偿力
 *       4. 或使用 compensation->F_Cx, F_Cy 作为笛卡尔坐标系的补偿力
 */
int VMC_CalcSpringCompensation(const VMC_JointState *joint,
                               const VMC_LinkageParams *params,
                               VMC_SpringCompensation *compensation) {
    if (!joint || !params || !compensation) {
        return 0;
    }
    
    // 初始化输出
    compensation->valid = 0;
    compensation->F_L0 = 0.0f;
    compensation->F_Cx = 0.0f;
    compensation->F_Cy = 0.0f;
    compensation->tau_spring_DC = 0.0f;
    
    // 计算虚拟腿状态
    VMC_VirtualLeg vleg;
    if (!VMC_ForwardKinematics(joint, params, &vleg)) {
        return 0;
    }
    
    // 计算偏导数
    float dL0_dQ4, dS_dQ4;
    if (!calc_partials_Q4(joint->Q1, joint->Q4, params, &dL0_dQ4, &dS_dQ4)) {
        return 0;
    }
    
    // 虚功原理计算等效力
    if (fabsf(dL0_dQ4) < 1e-10f) {
        return 0;
    }
    
    compensation->F_L0 = params->spring_force * dS_dQ4 / dL0_dQ4;
    
    // 转换为笛卡尔坐标系
    if (vleg.L0 > 1e-6f) {
        float AC_unit_x = vleg.Cx / vleg.L0;
        float AC_unit_y = vleg.Cy / vleg.L0;
        
        compensation->F_Cx = compensation->F_L0 * AC_unit_x;
        compensation->F_Cy = compensation->F_L0 * AC_unit_y;
    }
    
    // 计算弹簧对DC杆的力矩（可选，用于验证）
    float Q4_adj = (joint->Q1 >= joint->Q4) ? joint->Q4 : (joint->Q4 - 2.0f * M_PI);
    
    // 计算D点和C点
    float Dx = params->L4 * cosf(Q4_adj);
    float Dy = params->L4 * sinf(Q4_adj);
    
    float DCx = vleg.Cx - Dx;
    float DCy = vleg.Cy - Dy;
    float DC_len = sqrtf(DCx * DCx + DCy * DCy);
    
    if (DC_len > 1e-6f && params->L4 > 1e-6f) {
        // AD单位向量
        float AD_len = sqrtf(Dx * Dx + Dy * Dy);
        float AD_unit_x = Dx / AD_len;
        float AD_unit_y = Dy / AD_len;
        
        // DC单位向量
        float DC_unit_x = DCx / DC_len;
        float DC_unit_y = DCy / DC_len;
        
        // 叉乘 AD × DC = sin(Q3 - Q4)
        float AD_DC_cross = AD_unit_x * DC_unit_y - AD_unit_y * DC_unit_x;
        
        // 弹簧力矩
        compensation->tau_spring_DC = params->spring_force * params->spring_length * AD_DC_cross;
    }
    
    compensation->valid = 1;
    return 1;
}

/**
 * @brief 计算关节扭矩对应的虚拟腿等效力
 * 
 * 基于雅可比转置：τ = J^T × F
 * 求解：F_L0 = (τ1×dL0/dQ1 + τ4×dL0/dQ4) / (dL0/dQ1² + dL0/dQ4²)
 * 
 * @param joint 关节状态（输入）
 * @param params 连杆参数（输入）
 * @param tau1 关节1扭矩 (N*m)
 * @param tau4 关节4扭矩 (N*m)
 * @param F_L0_out 输出虚拟腿等效力 (N)
 * @param F_Cx_out 输出x方向力 (N)
 * @param F_Cy_out 输出y方向力 (N)
 * @return int 1=成功, 0=失败
 */
int VMC_TorqueToForce(const VMC_JointState *joint,
                      const VMC_LinkageParams *params,
                      float tau1, float tau4,
                      float *F_L0_out, float *F_Cx_out, float *F_Cy_out) {
    if (!joint || !params || !F_L0_out || !F_Cx_out || !F_Cy_out) {
        return 0;
    }
    
    // 计算虚拟腿状态
    VMC_VirtualLeg vleg;
    if (!VMC_ForwardKinematics(joint, params, &vleg)) {
        return 0;
    }
    
    // 计算偏导数
    const float delta = 1e-6f;
    
    // dL0/dQ1
    float L0_plus, L0_minus;
    if (!calc_L0(joint->Q1 + delta, joint->Q4, params, &L0_plus) || 
        !calc_L0(joint->Q1 - delta, joint->Q4, params, &L0_minus)) {
        return 0;
    }
    float dL0_dQ1 = (L0_plus - L0_minus) / (2.0f * delta);
    
    // dL0/dQ4
    if (!calc_L0(joint->Q1, joint->Q4 + delta, params, &L0_plus) || 
        !calc_L0(joint->Q1, joint->Q4 - delta, params, &L0_minus)) {
        return 0;
    }
    float dL0_dQ4 = (L0_plus - L0_minus) / (2.0f * delta);
    
    // 最小二乘解
    float denom = dL0_dQ1 * dL0_dQ1 + dL0_dQ4 * dL0_dQ4;
    
    if (fabsf(denom) < 1e-12f) {
        // 退化情况，尝试单关节解
        if (fabsf(dL0_dQ4) > 1e-8f) {
            *F_L0_out = tau4 / dL0_dQ4;
        } else if (fabsf(dL0_dQ1) > 1e-8f) {
            *F_L0_out = tau1 / dL0_dQ1;
        } else {
            return 0;
        }
    } else {
        *F_L0_out = (tau1 * dL0_dQ1 + tau4 * dL0_dQ4) / denom;
    }
    
    // 转换为笛卡尔坐标
    if (vleg.L0 > 1e-6f) {
        *F_Cx_out = (*F_L0_out) * vleg.Cx / vleg.L0;
        *F_Cy_out = (*F_L0_out) * vleg.Cy / vleg.L0;
    } else {
        *F_Cx_out = 0.0f;
        *F_Cy_out = 0.0f;
    }
    
    return 1;
}
