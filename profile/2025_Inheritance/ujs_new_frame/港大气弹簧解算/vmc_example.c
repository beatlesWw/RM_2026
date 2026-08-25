/**
 * @file vmc_example.c
 * @brief VMC气弹簧补偿函数使用示例
 * @date 2026-03-09
 */

#include <stdio.h>
#include <math.h>
#include "vmc_spring_compensation.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 示例1：基本的弹簧补偿力计算
 */
void example_basic_spring_compensation(void) {
    printf("\n========== 示例1：基本弹簧补偿力计算 ==========\n");
    
    // 1. 初始化连杆参数（与Python代码中的默认值一致）
    VMC_LinkageParams params = {
        .L1 = 0.21f,           // 大腿1长度 21cm
        .L2 = 0.25f,           // 小腿1长度 25cm
        .L3 = 0.25f,           // 小腿2长度 25cm
        .L4 = 0.21f,           // 大腿2长度 21cm
        .spring_length = 0.04863f,  // 弹簧力臂 48.63mm
        .spring_force = 400.0f      // 弹簧力 400N
    };
    
    // 2. 设置关节角度
    VMC_JointState joint = {
        .Q1 = 2.0f,  // 关节1角度 2.0 rad (约114.6度)
        .Q4 = 1.0f   // 关节4角度 1.0 rad (约57.3度)
    };
    
    // 3. 计算弹簧补偿力
    VMC_SpringCompensation compensation;
    int result = VMC_CalcSpringCompensation(&joint, &params, &compensation);
    
    if (result && compensation.valid) {
        printf("计算成功！\n");
        printf("虚拟腿方向等效力: F_L0 = %.2f N\n", compensation.F_L0);
        printf("足端x方向力: F_Cx = %.2f N\n", compensation.F_Cx);
        printf("足端y方向力: F_Cy = %.2f N\n", compensation.F_Cy);
        printf("弹簧对DC杆力矩: τ_spring = %.4f N·m\n", compensation.tau_spring_DC);
    } else {
        printf("计算失败！配置可能无效。\n");
    }
}

/**
 * @brief 示例2：正运动学计算
 */
void example_forward_kinematics(void) {
    printf("\n========== 示例2：正运动学计算 ==========\n");
    
    VMC_LinkageParams params = {
        .L1 = 0.21f, .L2 = 0.25f, .L3 = 0.25f, .L4 = 0.21f,
        .spring_length = 0.04863f, .spring_force = 400.0f
    };
    
    VMC_JointState joint = {
        .Q1 = 2.0f,
        .Q4 = 1.0f
    };
    
    VMC_VirtualLeg vleg;
    int result = VMC_ForwardKinematics(&joint, &params, &vleg);
    
    if (result) {
        printf("虚拟腿长度: L0 = %.4f m\n", vleg.L0);
        printf("虚拟腿角度: Q0 = %.4f rad (%.2f deg)\n", 
               vleg.Q0, vleg.Q0 * 180.0f / M_PI);
        printf("足端位置: C = (%.4f, %.4f) m\n", vleg.Cx, vleg.Cy);
    } else {
        printf("正运动学计算失败！\n");
    }
}

/**
 * @brief 示例3：关节扭矩转换为末端力
 */
void example_torque_to_force(void) {
    printf("\n========== 示例3：关节扭矩转末端力 ==========\n");
    
    VMC_LinkageParams params = {
        .L1 = 0.21f, .L2 = 0.25f, .L3 = 0.25f, .L4 = 0.21f,
        .spring_length = 0.04863f, .spring_force = 400.0f
    };
    
    VMC_JointState joint = {
        .Q1 = 2.0f,
        .Q4 = 1.0f
    };
    
    // 设置关节扭矩
    float tau1 = 5.0f;   // 关节1扭矩 5 N·m
    float tau4 = 3.0f;   // 关节4扭矩 3 N·m
    
    float F_L0, F_Cx, F_Cy;
    int result = VMC_TorqueToForce(&joint, &params, tau1, tau4, 
                                   &F_L0, &F_Cx, &F_Cy);
    
    if (result) {
        printf("输入扭矩: τ1 = %.2f N·m, τ4 = %.2f N·m\n", tau1, tau4);
        printf("等效虚拟腿力: F_L0 = %.2f N\n", F_L0);
        printf("等效末端力: F_C = (%.2f, %.2f) N\n", F_Cx, F_Cy);
        printf("末端力大小: |F_C| = %.2f N\n", 
               sqrtf(F_Cx * F_Cx + F_Cy * F_Cy));
    } else {
        printf("扭矩转换失败！\n");
    }
}

/**
 * @brief 示例4：实际控制应用（含弹簧补偿）
 */
void example_control_with_compensation(void) {
    printf("\n========== 示例4：实际控制应用 ==========\n");
    
    VMC_LinkageParams params = {
        .L1 = 0.21f, .L2 = 0.25f, .L3 = 0.25f, .L4 = 0.21f,
        .spring_length = 0.04863f, .spring_force = 400.0f
    };
    
    VMC_JointState joint = {
        .Q1 = 2.0f,
        .Q4 = 1.0f
    };
    
    // 期望的末端力（例如：支撑机器人重量）
    float F_desired_L0 = 100.0f;  // 期望虚拟腿方向100N的力
    
    // 1. 计算弹簧补偿力
    VMC_SpringCompensation spring_comp;
    VMC_CalcSpringCompensation(&joint, &params, &spring_comp);
    
    printf("弹簧补偿力: F_spring = %.2f N\n", spring_comp.F_L0);
    
    // 2. 计算需要的电机力（扣除弹簧补偿）
    float F_motor_needed = F_desired_L0 - spring_comp.F_L0;
    printf("电机需提供力: F_motor = %.2f N\n", F_motor_needed);
    
    // 3. 计算虚拟腿雅可比（数值方法）
    const float delta = 1e-6f;
    float L0_base, L0_plus, L0_minus;
    
    VMC_VirtualLeg vleg_base, vleg_plus, vleg_minus;
    VMC_ForwardKinematics(&joint, &params, &vleg_base);
    
    VMC_JointState joint_plus_Q1 = {.Q1 = joint.Q1 + delta, .Q4 = joint.Q4};
    VMC_ForwardKinematics(&joint_plus_Q1, &params, &vleg_plus);
    
    VMC_JointState joint_minus_Q1 = {.Q1 = joint.Q1 - delta, .Q4 = joint.Q4};
    VMC_ForwardKinematics(&joint_minus_Q1, &params, &vleg_minus);
    
    float dL0_dQ1 = (vleg_plus.L0 - vleg_minus.L0) / (2.0f * delta);
    
    VMC_JointState joint_plus_Q4 = {.Q1 = joint.Q1, .Q4 = joint.Q4 + delta};
    VMC_ForwardKinematics(&joint_plus_Q4, &params, &vleg_plus);
    
    VMC_JointState joint_minus_Q4 = {.Q1 = joint.Q1, .Q4 = joint.Q4 - delta};
    VMC_ForwardKinematics(&joint_minus_Q4, &params, &vleg_minus);
    
    float dL0_dQ4 = (vleg_plus.L0 - vleg_minus.L0) / (2.0f * delta);
    
    // 4. 力转扭矩（雅可比转置）
    float tau1_motor = F_motor_needed * dL0_dQ1;
    float tau4_motor = F_motor_needed * dL0_dQ4;
    
    printf("\n需要的关节扭矩:\n");
    printf("  τ1 = %.4f N·m\n", tau1_motor);
    printf("  τ4 = %.4f N·m\n", tau4_motor);
    
    // 5. 验证：总扭矩（电机+弹簧）
    float tau1_total = tau1_motor + spring_comp.F_L0 * dL0_dQ1;
    float tau4_total = tau4_motor + spring_comp.F_L0 * dL0_dQ4;
    
    printf("\n总扭矩（电机+弹簧）:\n");
    printf("  τ1_total = %.4f N·m\n", tau1_total);
    printf("  τ4_total = %.4f N·m\n", tau4_total);
    
    printf("\n节能效果:\n");
    printf("  弹簧贡献率: %.1f%%\n", 
           (spring_comp.F_L0 / F_desired_L0) * 100.0f);
}

/**
 * @brief 主函数
 */
int main(void) {
    printf("VMC气弹簧补偿函数使用示例\n");
    printf("====================================\n");
    
    // 运行所有示例
    example_basic_spring_compensation();
    example_forward_kinematics();
    example_torque_to_force();
    example_control_with_compensation();
    
    printf("\n====================================\n");
    printf("所有示例运行完成！\n");
    
    return 0;
}
