#include "balance.h"
#include "stdint.h"
#include "arm_math.h"

/**
 * @brief 根据状态反馈计算当前腿长,查表获得LQR的反馈增益,并列式计算LQR的输出
 * @note 得到的腿部力矩输出还要经过综合运动控制系统补偿后映射为两个关节电机输出
 *
 */
static void CalcLQR(LinkNPodParam *p, ChassisParam *chassis)
{
    static float k[12][3] = {64.570926,-78.151551,-15.098827,
                            1.716153,-4.573289,-0.526836,
                            25.565621,-20.634320,-17.563574,
                            11.633020,-11.021911,-13.087704,
                            212.754066,-196.450645,56.227305,
                            12.051262,-11.941105,4.523285,
                            74.149148,-71.296364,23.673363,
                            4.449069,-4.214434,1.145555,
                            115.321993,-106.373546,29.626387,
                            82.459182,-74.915697,20.072512,
                            -163.793049,130.808796,114.419419,
                            -13.396998,11.357026,4.288949};
    float T[2] = {0};       // 0 T_wheel  1 T_hip
    float l = p->leg_len;
    float lsqr = l * l;

    uint8_t i, j;

    // 离地时轮子输出置0
    i = 0; j = i * 6;
    T[i] = p->fly_flag ? 0 :
           ((k[j + 0][0] * lsqr + k[j + 0][1] * l + k[j + 0][2]) * -p->theta +
            (k[j + 1][0] * lsqr + k[j + 1][1] * l + k[j + 1][2]) * -p->theta_w + 
            (k[j + 2][0] * lsqr + k[j + 2][1] * l + k[j + 2][2]) * (chassis->target_dist - chassis->dist) +
            (k[j + 3][0] * lsqr + k[j + 3][1] * l + k[j + 3][2]) * (chassis->target_v - chassis->vel) +
            (k[j + 4][0] * lsqr + k[j + 4][1] * l + k[j + 4][2]) * -chassis->pitch +
            (k[j + 5][0] * lsqr + k[j + 5][1] * l + k[j + 5][2]) * -chassis->pitch_w);
    
    // 离地时关节输出仅保留 theta 和 theta_dot，保证滞空时腿部竖直
    i = 1; j = i * 6;
    T[i] = (k[j + 0][0] * lsqr + k[j + 0][1] * l + k[j + 0][2]) * -p->theta +
           (k[j + 1][0] * lsqr + k[j + 1][1] * l + k[j + 1][2]) * -p->theta_w + (p->fly_flag ? 0 :
           ((k[j + 2][0] * lsqr + k[j + 2][1] * l + k[j + 2][2]) * (chassis->target_dist - chassis->dist) +
            (k[j + 3][0] * lsqr + k[j + 3][1] * l + k[j + 3][2]) * (chassis->target_v - chassis->vel) +
            (k[j + 4][0] * lsqr + k[j + 4][1] * l + k[j + 4][2]) * -chassis->pitch +
            (k[j + 5][0] * lsqr + k[j + 5][1] * l + k[j + 5][2]) * -chassis->pitch_w));
    
    p->T_wheel = T[0];
    p->T_hip = T[1];
}