#pragma once

#include "stdint.h"

// 底盘参数
#define CALF_LEN 0.24f              // 小腿
#define THIGH_LEN 0.14f             // 大腿
#define JOINT_DISTANCE 0.11f        // 关节间距
#define WHEEL_RADIUS 0.075f         // 轮子半径
#define LIMIT_LINK_RAD 0.220039368      // 初始限位角度,见ParamAssemble
#define BALANCE_GRAVITY_BIAS 0
#define ROLL_GRAVITY_BIAS 0
#define MAX_ACC_REF 1.2f

// 驱动轮质量
#define WHEEL_MASS 0.58f

// IMU距离中心的距离
#define CENTER_IMU_W -0.024f
#define CENTER_IMU_L 0.09f
#define CENTER_IMU_H -0.1f

#define VEL_PROCESS_NOISE 10  // 速度过程噪声
#define VEL_MEASURE_NOISE 2000 // 速度测量噪声
// 同时估计加速度和速度时对加速度的噪声
// 更好的方法是设置为动态,当有冲击时/加加速度大时更相信轮速
#define ACC_PROCESS_NOISE 2000 // 加速度过程噪声
#define ACC_MEASURE_NOISE 0.01 // 加速度测量噪声

// 用于循环枚举的宏,方便访问关节电机和驱动轮电机
#define JOINT_CNT 4u
#define LF 0u
#define LB 1u
#define RF 2u
#define RB 3u

#define DRIVEN_CNT 2u
#define LD 0u
#define RD 1u

typedef struct
{
    // joint
    float phi1_w, phi4_w, phi2_w, phi3_w, phi5_w; // phi2_w or phi3_w used for calc real wheel speed
    float T_back, T_front;

    // link angle, phi1-ph5, phi5 is pod angle
    float phi1, phi2, phi3, phi4, phi5;

    // wheel
    float w_ecd;      // 电机编码器速度
    float wheel_dist; // 单侧轮子的位移
    float wheel_w;    // 单侧轮子的速度
    float body_v;     // 髋关节速度
    float T_wheel;
    float zw_ddot;    // 驱动轮竖直方向加速度
    float normal_force;  // 支持力
    uint8_t fly_flag;    // 离地标志位

    // pod
    float theta, theta_w; // 杆和垂直方向的夹角,为控制状态之一
    float leg_len, legd;
    float height, height_v;
    float F_leg, T_hip;
    float target_len;

    float coord[6]; // xb yb xc yc xd yd

} LinkNPodParam;

typedef struct
{
    // 速度
    float vel, target_v;        // 底盘速度
    float vel_m;                // 底盘速度测量值
    float vel_predict;          // 底盘速度预测值
    float vel_cov;              // 速度方差
    float acc_m, acc_last;      // 水平方向加速度,用于计算速度预测值

    // 位移
    float dist, target_dist;   // 底盘位移距离

    // IMU
    float yaw, wz, target_yaw; // yaw角度和底盘角速度
    float pitch, pitch_w;      // 底盘俯仰角度和角速度
    float roll, roll_w;        // 底盘横滚角度和角速度
    
} ChassisParam;

/**
 * @brief 平衡底盘初始化
 *
 */
void BalanceInit();

/**
 * @brief 平衡底盘任务
 *
 */
void BalanceTask();
