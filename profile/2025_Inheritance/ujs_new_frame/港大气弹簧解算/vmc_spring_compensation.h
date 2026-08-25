/**
 * @file vmc_spring_compensation.h
 * @brief VMC气弹簧补偿力计算函数头文件
 * @author Generated from Python VMC Visualizer
 * @date 2026-03-09
 */

#ifndef VMC_SPRING_COMPENSATION_H
#define VMC_SPRING_COMPENSATION_H

#ifdef __cplusplus
extern "C" {
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
 * @brief 正运动学求解（计算虚拟腿状态）
 * 
 * @param joint 关节状态
 * @param params 连杆参数
 * @param vleg 输出虚拟腿状态
 * @return int 1=成功, 0=失败
 */
int VMC_ForwardKinematics(const VMC_JointState *joint, 
                          const VMC_LinkageParams *params,
                          VMC_VirtualLeg *vleg);

/**
 * @brief 计算气弹簧补偿力（核心函数）
 * 
 * 基于虚功原理：F_spring × d(S·AD)/dQ4 = F_L0 × dL0/dQ4
 * 
 * @param joint 关节状态（输入）
 * @param params 连杆参数（输入）
 * @param compensation 补偿力输出
 * @return int 1=成功, 0=失败
 */
int VMC_CalcSpringCompensation(const VMC_JointState *joint,
                               const VMC_LinkageParams *params,
                               VMC_SpringCompensation *compensation);

/**
 * @brief 计算关节扭矩对应的虚拟腿等效力
 * 
 * 基于雅可比转置：τ = J^T × F
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
                      float *F_L0_out, float *F_Cx_out, float *F_Cy_out);

#ifdef __cplusplus
}
#endif

#endif // VMC_SPRING_COMPENSATION_H
