#include "LQR_gimbal_encapsulate.h"
#include "arm_math.h"

/*==============================================================================
 * LQR 控制器初始化函数
 *============================================================================*/

/**
 * @brief  初始化二阶LQR控制器
 * @param  lqr: LQR控制器结构体指针
 * @param  k1: 位置误差增益
 * @param  k2: 速度误差增益
 * @param  out_max: 输出上限
 * @param  out_min: 输出下限
 * @note   二阶LQR用于位置-速度双状态反馈控制
 *         状态向量 x = [position, velocity]^T
 *         控制律 u = -K * x = -K1*(θ_target - θ_actual) + K2*ω_actual
 *         即 u = K1*(θ_target - θ_actual) - K2*ω_actual
 */
void LQR_SecondOrder_Init(LQR_SecondOrder_TypeDef *lqr, float k1, float k2, float out_max, float out_min)
{
    lqr->K1 = k1;
    lqr->K2 = k2;
    lqr->output_max = out_max;
    lqr->output_min = out_min;
}

/**
 * @brief  初始化一阶LQR控制器
 * @param  lqr: LQR控制器结构体指针
 * @param  k: 速度误差增益
 * @param  out_max: 输出上限
 * @param  out_min: 输出下限
 * @note   一阶LQR用于单速度状态反馈控制
 *         状态向量 x = [velocity]
 *         控制律 u = K * (ω_target - ω_actual)
 */
void LQR_FirstOrder_Init(LQR_FirstOrder_TypeDef *lqr, float k, float out_max, float out_min)
{
    lqr->K = k;
    lqr->output_max = out_max;
    lqr->output_min = out_min;
}

/*==============================================================================
 * LQR 控制器通用计算函数
 *============================================================================*/

/**
 * @brief  限幅函数
 * @param  value: 输入值
 * @param  max: 上限
 * @param  min: 下限
 * @retval 限幅后的值
 * @note   确保输出值在安全范围内，防止电机过流或机械损坏
 */
float LQR_Constrain(float value, float max, float min)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/**
 * @brief  二阶LQR控制器计算 (位置-速度状态反馈)
 * @param  lqr: LQR控制器结构体指针
 * @param  target_pos: 目标位置 (rad)
 * @param  actual_pos: 实际位置 (rad)
 * @param  actual_vel: 实际速度 (rad/s)
 * @retval 控制输出 (力矩或电流)
 * @note   LQR状态反馈控制律: u = K1*(θ_ref - θ) - K2*ω
 *         - K1*(θ_ref - θ): 位置误差项，使系统趋向目标位置
 *         - K2*ω: 速度阻尼项，抑制振荡，提高稳定性
 */
float LQR_SecondOrder_Calc(LQR_SecondOrder_TypeDef *lqr, float target_pos, float actual_pos, float actual_vel)
{
    float pos_error = target_pos - actual_pos;
    float output = lqr->K1 * pos_error - lqr->K2 * actual_vel;
    return LQR_Constrain(output, lqr->output_max, lqr->output_min);
}

/**
 * @brief  一阶LQR控制器计算 (速度状态反馈)
 * @param  lqr: LQR控制器结构体指针
 * @param  target_vel: 目标速度 (rad/s)
 * @param  actual_vel: 实际速度 (rad/s)
 * @retval 控制输出 (力矩)
 * @note   一阶LQR控制律: u = K*(ω_ref - ω)
 *         本质上是比例控制，但增益K通过LQR最优控制理论计算得出
 */
float LQR_FirstOrder_Calc(LQR_FirstOrder_TypeDef *lqr, float target_vel, float actual_vel)
{
    float vel_error = target_vel - actual_vel;
    float output = lqr->K * vel_error;
    return LQR_Constrain(output, lqr->output_max, lqr->output_min);
}

/*==============================================================================
 * 云台专用LQR控制函数
 *============================================================================*/

/**
 * @brief  Pitch轴LQR力矩计算 (含重力前馈补偿)
 * @param  target_theta: 目标角度 (rad)
 * @param  actual_theta: 实际角度 (rad) - 来自陀螺仪
 * @param  actual_omega: 实际角速度 (rad/s) - 来自陀螺仪
 * @param  motor_pos: 电机位置 (rad) - 用于重力前馈计算
 * @retval 控制力矩 (Nm)
 * @note   Pitch轴控制包含两部分:
 *         1. LQR状态反馈: τ_lqr = K1*(θ_ref - θ) - K2*ω
 *         2. 重力前馈补偿: τ_ff = -m*g*L*cos(θ_motor)
 *            - 前馈补偿用于抵消云台重力力矩，减小稳态误差
 *            - 系数0.6f为重力力矩常数 (m*g*L)
 *         总控制力矩: τ = τ_lqr + τ_ff
 */
float LQR_Pitch_CalcTorque(float target_theta, float actual_theta, float actual_omega, float motor_pos)
{
    // LQR状态反馈计算
    float tau = LQR_PITCH_K1 * (target_theta - actual_theta) - LQR_PITCH_K2 * actual_omega;
    
    // 重力前馈补偿 (0.6f为重力力矩常数 m*g*L)
    tau -= 0.6f * arm_cos_f32(motor_pos);
    
    // 力矩限幅
    return LQR_Constrain(tau, LQR_PITCH_TAU_MAX, -LQR_PITCH_TAU_MAX);
}

/**
 * @brief  Yaw轴LQR电流计算
 * @param  target_theta: 目标角度 (rad)
 * @param  actual_theta: 实际角度 (rad) - 来自陀螺仪
 * @param  actual_omega: 实际角速度 (rad/s) - 来自陀螺仪
 * @retval 控制电流
 * @note   Yaw轴使用GM6020电机，控制流程:
 *         1. LQR计算力矩: τ = K3*(θ_ref - θ) - K4*ω
 *         2. 力矩转电流: I = τ / (Kt * ratio)
 *            - Kt = 0.741 Nm/A (GM6020力矩常数)
 *            - ratio = 3 (内部减速比)
 *         3. 电流映射: Current = I / I_max * 16384
 */
float LQR_Yaw_CalcCurrent(float target_theta, float actual_theta, float actual_omega)
{
    // LQR状态反馈计算力矩
    float tau = LQR_YAW_K3 * (target_theta - actual_theta) - LQR_YAW_K4 * actual_omega;
    
    // 力矩转电流 (Kt=0.741, ratio=3)
    float current = tau / 0.741f / 3.0f * 16384.0f;
    
    // 电流限幅
    return LQR_Constrain(current, LQR_YAW_CURRENT_MAX, -LQR_YAW_CURRENT_MAX);
}

/**
 * @brief  摩擦轮LQR电流计算
 * @param  target_speed: 目标速度 (rad/s)
 * @param  actual_rpm: 实际转速 (rpm)
 * @retval 控制电流
 * @note   摩擦轮使用M3508电机，采用一阶LQR速度控制:
 *         1. 转速单位转换: rpm -> rad/s
 *            ω = rpm / 60 * 2π
 *         2. LQR计算力矩: τ = K5 * (ω_ref - ω)
 *         3. 力矩转电流: I = τ / (Kt * ratio)
 *            - Kt = 0.3 Nm/A (M3508力矩常数)
 *            - ratio = 20 (减速比，实际为19:1)
 */
float LQR_Friction_CalcCurrent(float target_speed, float actual_rpm)
{
    // rpm转rad/s
    float actual_speed = actual_rpm / 60.0f * 2.0f * 3.1415926f;
    
    // LQR速度控制计算力矩
    float tau = LQR_FRICTION_K5 * (target_speed - actual_speed);
    
    // 力矩转电流 (Kt=0.3, ratio=20)
    float current = tau / 0.3f / 20.0f * 16384.0f;
    
    // 电流限幅
    return LQR_Constrain(current, LQR_FRICTION_CURRENT_MAX, -LQR_FRICTION_CURRENT_MAX);
}

/**
 * @brief  拨弹盘速度环LQR电流计算
 * @param  target_speed: 目标速度 (rad/s)
 * @param  actual_shaft_speed: 实际输出轴转速 (rpm)
 * @retval 控制电流
 * @note   拨弹盘使用M2006电机，采用一阶LQR速度控制:
 *         1. 转速单位转换: rpm -> rad/s
 *         2. LQR计算力矩: τ = K7 * (ω_ref - ω)
 *         3. 力矩转电流: I = (τ / ratio) / Kt / 10 * 10000
 *            - Kt = 0.18 Nm/A (M2006力矩常数)
 *            - ratio = 36 (减速比)
 *            - 10000为M2006电流满量程
 */
float LQR_Rammer_SpeedCalcCurrent(float target_speed, float actual_shaft_speed)
{
    // rpm转rad/s
    float actual_speed = actual_shaft_speed / 60.0f * 2.0f * 3.1415926f;
    
    // LQR速度控制计算力矩
    float tau = LQR_RAMMER_K7 * (target_speed - actual_speed);
    
    // 力矩转电流 (ratio=36, Kt=0.18)
    float current = (tau / 36.0f) / 0.18f / 10.0f * 10000.0f;
    
    // 电流限幅
    return LQR_Constrain(current, LQR_RAMMER_CURRENT_MAX, -LQR_RAMMER_CURRENT_MAX);
}

/**
 * @brief  拨弹盘位置环LQR电流计算 (单发模式)
 * @param  target_pos: 目标位置 (rad)
 * @param  actual_pos_deg: 实际位置 (degree)
 * @param  actual_shaft_speed: 实际输出轴转速 (rpm)
 * @retval 控制电流
 * @note   单发模式采用位置-速度双环LQR控制:
 *         1. 外环(位置环): 计算期望速度
 *            v_ref = K7 * (θ_ref - θ)，并限幅在±10 rad/s
 *         2. 内环(速度环): 计算控制力矩
 *            τ = K8 * (v_ref - v)
 *         3. 力矩转电流
 *         这种级联结构可以实现精确的位置控制，同时限制最大速度
 */
float LQR_Rammer_PosCalcCurrent(float target_pos, float actual_pos_deg, float actual_shaft_speed)
{
    // 角度单位转换: degree -> rad
    float actual_pos = actual_pos_deg / 180.0f * 3.1415926f;
    
    // 外环: 位置LQR计算期望速度
    float target_vel = LQR_DANFA_K7 * (target_pos - actual_pos);
    
    // 速度限幅
    target_vel = LQR_Constrain(target_vel, 10.0f, -10.0f);
    
    // 内环: 速度LQR计算力矩
    float actual_vel = actual_shaft_speed / 60.0f * 2.0f * 3.1415926f;
    float tau = LQR_DANFA_K8 * (target_vel - actual_vel);
    
    // 力矩转电流 (ratio=36, Kt=0.18)
    float current = (tau / 36.0f) / 0.18f / 10.0f * 10000.0f;
    
    // 电流限幅
    return LQR_Constrain(current, LQR_RAMMER_CURRENT_MAX, -LQR_RAMMER_CURRENT_MAX);
}

/*==============================================================================
 * 力矩-电流转换函数
 *============================================================================*/

/**
 * @brief  力矩到电流转换 (GM6020电机)
 * @param  tau: 力矩 (Nm)
 * @param  kt: 力矩常数 (Nm/A)，默认0.741
 * @param  ratio: 减速比，默认3
 * @retval 电流值 (范围-16384~16384)
 * @note   GM6020为直驱电机，内部有3:1减速
 *         电流计算: I = τ / (Kt * ratio) * 16384
 */
float LQR_TorqueToCurrent_GM6020(float tau, float kt, float ratio)
{
    float current = tau / kt / ratio * 16384.0f;
    return LQR_Constrain(current, 16384.0f, -16384.0f);
}

/**
 * @brief  力矩到电流转换 (M3508电机)
 * @param  tau: 力矩 (Nm)
 * @param  kt: 力矩常数 (Nm/A)，默认0.3
 * @param  ratio: 减速比，默认19 (实际使用20)
 * @retval 电流值 (范围-16384~16384)
 * @note   M3508配合C620电调，减速比19:1
 *         电流计算: I = τ / (Kt * ratio) * 16384
 */
float LQR_TorqueToCurrent_M3508(float tau, float kt, float ratio)
{
    float current = tau / kt / ratio * 16384.0f;
    return LQR_Constrain(current, 16384.0f, -16384.0f);
}

/**
 * @brief  力矩到电流转换 (M2006电机)
 * @param  tau: 力矩 (Nm)
 * @param  kt: 力矩常数 (Nm/A)，默认0.18
 * @param  ratio: 减速比，默认36
 * @retval 电流值 (范围-10000~10000)
 * @note   M2006配合C610电调，减速比36:1
 *         电流计算: I = (τ / ratio) / Kt / 10 * 10000
 */
float LQR_TorqueToCurrent_M2006(float tau, float kt, float ratio)
{
    float current = (tau / ratio) / kt / 10.0f * 10000.0f;
    return LQR_Constrain(current, 10000.0f, -10000.0f);
}
