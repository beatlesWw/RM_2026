#include "balance.h"
#include "arm_math.h"
#include "math.h"
#include "user_lib.h"

// 驱动轮支持力解算
void NormalForceSolve(LinkNPodParam *p, INS_t *imu)
{
    static float accx, accy, accz;
    accx = imu->MotionAccel_b[X];
    accy = imu->MotionAccel_b[Y];
    accz = imu->MotionAccel_b[Z];

    static float pitch, roll;
    pitch = imu->Pitch;
    roll = imu->Roll;

    // 机体竖直方向加速度
    p->zw_ddot = -msin(roll) * accx + mcos(roll) * msin(pitch) * accy + mcos(pitch) * mcos(roll) * accz;

    // 驱动轮支持力解算
    static float P;
    P = p->F_leg * mcos(p->theta) + p->T_hip * msin(p->theta) / p->leg_len;
    p->normal_force = P + WHEEL_MASS * (p->zw_ddot + 9.81f); 

    // 离地检测
    if(p->normal_force < 20.0f)
        p->fly_flag = 1;
    else
        p->fly_flag = 0;
}