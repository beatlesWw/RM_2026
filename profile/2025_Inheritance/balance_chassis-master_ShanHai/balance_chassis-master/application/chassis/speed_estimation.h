#include "balance.h"
#include "user_lib.h"
#include "ins_task.h"
#include "general_def.h"


/**
 * @brief 使用卡尔曼滤波估计底盘速度
 * @todo 增加w和dw的滤波,当w和dw均小于一定值时,不考虑dw导致的角加速度
 *
 * @param lp 左侧腿
 * @param rp 右侧腿
 * @param cp 底盘
 * @param imu imu数据
 * @param delta_t 更新间隔
 */
void SpeedEstimation(LinkNPodParam *lp, LinkNPodParam *rp, ChassisParam *cp, INS_t *imu, float delta_t)
{
    // 修正轮速和距离
    lp->wheel_w = lp->w_ecd + lp->phi3_w - cp->pitch_w; // 减去和定子固连的phi2_w
    rp->wheel_w = rp->w_ecd + rp->phi3_w - cp->pitch_w;

    // 以轮子为基点,计算机体两侧髋关节处的速度
    lp->body_v = lp->wheel_w * WHEEL_RADIUS + lp->leg_len * lp->theta_w + lp->legd * msin(lp->theta);
    rp->body_v = rp->wheel_w * WHEEL_RADIUS + rp->leg_len * rp->theta_w + rp->legd * msin(rp->theta);
    cp->vel_m = (lp->body_v + rp->body_v) / 2; // 机体速度(平动)为两侧速度的平均值
    
    // 扣除旋转导致的向心加速度和角加速度*R
    float *gyro = imu->Gyro, *dgyro = imu->dgyro;
    static float yaw_ddwrNwwr, yaw_p_ddwrNwwr, pitch_ddwrNwwr;
    yaw_ddwrNwwr = -powf(gyro[Z], 2) * CENTER_IMU_L + dgyro[Z] * CENTER_IMU_W;     // yaw旋转导致motion_acc[1]的额外加速度(机体前后方向)
    yaw_p_ddwrNwwr = -powf(gyro[X], 2) * CENTER_IMU_L - dgyro[X] * CENTER_IMU_H;   // pitch旋转导致motion_acc[1]的额外加速度(机体前后方向)
    pitch_ddwrNwwr = -powf(gyro[X], 2) * CENTER_IMU_H + dgyro[X] * CENTER_IMU_L;   // pitch旋转导致motion_acc[2]的额外加速度(机体竖直方向)

    // 补偿后的实际平动加速度,机体系前进方向和竖直方向
    static float macc_y, macc_z;
    macc_y = imu->MotionAccel_b[Y] - yaw_ddwrNwwr - yaw_p_ddwrNwwr;
    macc_z = imu->MotionAccel_b[Z] - pitch_ddwrNwwr;

    // 机体加速度投影到水平方向上
    static float pitch;
    pitch = imu->Pitch * DEGREE_2_RAD;
    cp->acc_last = cp->acc_m;
    cp->acc_m = macc_y * mcos(pitch) - macc_z * msin(pitch);

    // 融合加速度计的数据和机体速度
    static float u, k;   // 输入和卡尔曼增益
    static float vel_prior, vel_measure, vel_cov;     // 先验估计、测量、先验协方差

    // 预测
    u = (cp->acc_m + cp->acc_last) / 2;         // 速度梯形积分
    cp->vel_predict = vel_prior = cp->vel + delta_t * u;          // 先验估计
    vel_cov = cp->vel_cov + VEL_PROCESS_NOISE * delta_t;          // 先验协方差

    // 校正
    vel_measure = cp->vel_m;
    k = vel_cov / (vel_cov + VEL_MEASURE_NOISE);            // 卡尔曼增益
    cp->vel = vel_prior + k * (vel_measure - vel_prior);    // 后验估计
    cp->vel_cov = (1 - k) * vel_cov;                        // 后验协方差

    VAL_LIMIT(cp->vel_cov, 0.01, 100);       // 协方差限幅

    // 速度和位置分离，有速度输入时不进行位置闭环
    if(abs(cp->target_v) < 0.005)
    {
        cp->target_dist = 0;
        cp->dist += cp->vel * delta_t;
    }
    else
        cp->target_dist = cp->dist = 0;
}