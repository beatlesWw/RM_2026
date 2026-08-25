// app
#include "balance.h"
#include "linkNleg.h"
#include "robot_def.h"
#include "general_def.h"
#include "ins_task.h"
#include "HT04.h"
#include "LK9025.h"
#include "controller.h"
#include "can_comm.h"
#include "super_cap.h"
#include "user_lib.h"
#include "remote_control.h"
#include "referee_task.h"
#include "stdint.h"
#include "arm_math.h" // 需要用到较多三角函数
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "lqr_calc.h"
#include "speed_estimation.h"
#include "fly_detection.h"
#include "buzzer.h"
// 计时变量
static uint32_t balance_dwt_cnt;
static float del_t;

// 底盘拥有的实例模块
static INS_t *Chassis_IMU_data;
static RC_ctrl_t *rc_data; // 底盘单独调试用
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘反馈数据
static Chassis_Can_Comm chassis_can_recv;
// 四个关节电机和两个驱动轮电机
static HTMotorInstance *lf, *lb, *rf, *rb, *joint[4]; // 指针数组方便传参和调试
static LKMotorInstance *l_driven, *r_driven, *driven[2];

// 两个腿的参数,0为左腿,1为右腿
static LinkNPodParam l_side, r_side;
static ChassisParam chassis;

// 综合运动补偿的PID控制器
static PIDInstance leglen_pid_l, leglen_pid_r; // 用PD模拟弹簧, 不要积分(弹簧是无积分二阶系统), 增益不可过大否则抗外界冲击响应时太"硬"
static PIDInstance roll_compensate_pid;        // roll轴补偿,用于保持机体水平
static PIDInstance steer_p_pid, steer_v_pid;   // 转向PID,有转向指令时使用IMU的加速度反馈积分以获取速度和位置状态量
static PIDInstance anti_crash_pid;             // 抗劈叉,将输出以相反的方向叠加到左右腿的上

// 底盘状态
static Robot_Status_e chassis_status;

static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static CANCommInstance *cmd_can_comm; // 底盘CAN通信实例

static SuperCapInstance *cap; // 超级电容
static uint16_t DataSend2Cap[4] = {0, 0, 0, 0};

void BalanceInit()
{
    rc_data = RemoteControlInit(&huart3);
    Chassis_IMU_data = INS_Init();
    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI
    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // ww超级电容初始化
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .daemon_count = 100,
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);

    // 关节电机
    Motor_Init_Config_s joint_conf = {
        // 写一个,剩下的修改方向和id即可
        .can_init_config = {
            .can_handle = &hcan1},
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.1,
                .Kd = 0,
                .Ki = 0,
                .DeadBand = 0.0001,
                .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
                .MaxOut = 4,
                .Derivative_LPF_RC = 0.05,
            }, // 仅用于复位腿
        },
        .controller_setting_init_config = {
            .close_loop_type = ANGLE_LOOP,
            .outer_loop_type = OPEN_LOOP,
            .motor_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
        .motor_type = HT04};
    joint_conf.can_init_config.tx_id = 1;
    joint_conf.can_init_config.rx_id = 11;
    joint[LF] = lf = HTMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 2;
    joint_conf.can_init_config.rx_id = 12;
    joint[LB] = lb = HTMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 3;
    joint_conf.can_init_config.rx_id = 13;
    joint[RF] = rf = HTMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 4;
    joint_conf.can_init_config.rx_id = 14;
    joint[RB] = rb = HTMotorInit(&joint_conf);

    // 驱动轮电机
    Motor_Init_Config_s driven_conf = {
        // 写一个,剩下的修改方向和id即可
        .can_init_config.can_handle = &hcan2,
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = LK9025,

    };
    driven_conf.can_init_config.tx_id = 1;
    driven[LD] = l_driven = LKMotorInit(&driven_conf);
    driven_conf.can_init_config.tx_id = 2;
    driven[RD] = r_driven = LKMotorInit(&driven_conf);

    // 腿长控制
    PID_Init_Config_s leg_length_pid_conf = {
        .Kp = 1200,
        .Kd = 300,
        .Ki = 0,
        .MaxOut = 60,
        .DeadBand = 0.0001f,
        .Improve = PID_ChangingIntegrationRate | PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&leglen_pid_l, &leg_length_pid_conf);
    PIDInit(&leglen_pid_r, &leg_length_pid_conf);

    // roll轴补偿
    PID_Init_Config_s roll_compensate_pid_conf = {
        .Kp = 0.0008f,
        .Kd = 0.0002f,
        .Ki = 0.0f,
        .MaxOut = 0.05,
        .DeadBand = 0.001f,
        .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&roll_compensate_pid, &roll_compensate_pid_conf);

    // 航向控制
    // 角度环
    PID_Init_Config_s steer_p_pid_conf = {
        .Kp = 5,
        .Kd = 0,
        .Ki = 0.0f,
        .MaxOut = 3,
        .DeadBand = 0.001f,
        .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&steer_p_pid, &steer_p_pid_conf);
    // 速度环
    PID_Init_Config_s steer_v_pid_conf = {
        .Kp = 3,
        .Kd = 0.0f,
        .Ki = 0.0f,
        .MaxOut = 20,
        .DeadBand = 0.0f,
        .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&steer_v_pid, &steer_v_pid_conf);

    // 抗劈叉
    PID_Init_Config_s anti_crash_pid_conf = {
        .Kp = 15,
        .Kd = 2,
        .Ki = 0.0,
        .MaxOut = 30,
        .DeadBand = 0.001f,
        .Improve = PID_DerivativeFilter | PID_ChangingIntegrationRate | PID_Integral_Limit,
        .Derivative_LPF_RC = 0.01,
    };
    PIDInit(&anti_crash_pid, &anti_crash_pid_conf);

    // 状态初始化
    l_side.target_len = r_side.target_len = 0.12;
    chassis.vel_cov = 100; // 速度协方差初始化
    chassis_status = ROBOT_READY;
    DWT_GetDeltaT(&balance_dwt_cnt);
}

static void EnableAllMotor() /* 打开所有电机 */
{
    for (uint8_t i = 0; i < JOINT_CNT; i++) // 打开关节电机
        HTMotorEnable(joint[i]);
    for (uint8_t i = 0; i < DRIVEN_CNT; i++) // 打开驱动电机
        LKMotorEnable(driven[i]);
}

// 检查关节电机是否离线
static uint8_t JointMotorIsLost()
{
    for (uint8_t i = 0; i < JOINT_CNT; i++)
    {
        if (joint[i]->motor_daemon->temp_count == 0)
            return 1;
    }

    return 0;
}

// 检查驱动轮电机是否离线
static uint8_t DrivenMotorIsLost()
{
    for (uint8_t i = 0; i < DRIVEN_CNT; i++)
    {
        if (driven[i]->daemon->temp_count == 0)
            return 1;
    }

    return 0;
}

/* 切换底盘遥控器控制和云台双板控制 */
static void ControlSwitch()
{
    float chassis_vol = referee_data->PowerHeatData.chassis_voltage * 0.001;

    // 根据裁判系统底盘输出电压设定底盘状态
    if (chassis_vol < 15.0f || JointMotorIsLost() || DrivenMotorIsLost())
    {
        chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE; // 皆离线,急停
        return;
    }

    // // 右侧拨杆向下,进入遥控器底盘控制,此时不响应云台控制指令
    // if (switch_is_down(rc_data->rc.switch_right) && RemoteControlIsOnline())
    // {
    //     if (switch_is_up(rc_data->rc.switch_left))
    //     {
    //         chassis_cmd_recv.chassis_mode = CHASSIS_RESET;
    //         chassis_cmd_recv.vx = 0.5 * (float)rc_data[TEMP].rc.rocker_r1; // speed x, unit m/s
    //         chassis_cmd_recv.rotate_w = 0.5 * (float)rc_data[TEMP].rc.rocker_r_;
    //     }
    //     else
    //     {
    //         chassis_cmd_recv.chassis_mode = CHASSIS_FREE_DEBUG;              // 自由转动&前后
    //         chassis_cmd_recv.vx = 0.003 * (float)rc_data[TEMP].rc.rocker_r1; // speed x, unit m/s
    //         chassis_cmd_recv.delta_leglen = -0.0000005f * (float)rc_data[TEMP].rc.dial;
    //         chassis_cmd_recv.offset_angle -= 0.000005 * (float)rc_data[TEMP].rc.rocker_r_;
    //     }
    // }
    // else
    // {
    //     chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(cmd_can_comm);
    // }
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(cmd_can_comm);
    // if (abs(l_side.theta) > (30.0f * DEGREE_2_RAD) || abs(r_side.theta) > (30.0f * DEGREE_2_RAD))
    // {
    //     chassis_cmd_recv.chassis_mode = CHASSIS_RESET;
    // }
}

/* 腿缩回复位,只允许驱动轮电机移动 */
static void ResetChassis()
{
    EnableAllMotor(); // 打开全部电机,关节复位到起始角度,驱动电机响应速度输入以从墙角或固连中脱身

    // 目标速度置0
    chassis.target_v = 0;
    // 复位时清空距离和腿长积累量,保证顺利站起
    chassis.dist = chassis.target_dist = 0;
    l_side.target_len = r_side.target_len = 0.12;
    // 角度输入为当前角度
    // chassis_cmd_recv.offset_angle = chassis.target_yaw = chassis.yaw;

    // 撞墙时前后移动保证能重新站立,执行速度输入
    LKMotorSetRef(l_driven, chassis_cmd_recv.vx + (float)chassis_cmd_recv.rotate_w);
    LKMotorSetRef(r_driven, -chassis_cmd_recv.vx + (float)chassis_cmd_recv.rotate_w);

    // 若关节完成复位,进入ready态
    if (abs(lf->measure.total_angle) < 0.05 &&
        abs(lb->measure.total_angle) < 0.05 &&
        abs(rf->measure.total_angle) < 0.05 &&
        abs(rb->measure.total_angle) < 0.05)
    {
        chassis_status = ROBOT_READY; // 底盘已经准备好重新站立
    }
    else if (abs(lf->measure.total_angle) <= 0.03 &&
             abs(lb->measure.total_angle) <= 0.03 &&
             abs(rf->measure.total_angle) <= 0.03 &&
             abs(rb->measure.total_angle) <= 0.03)
    {                                 // 双阈值保证关节能够复位而不会进入死区
        chassis_status = ROBOT_READY; // 底盘已经准备好重新站立

        for (uint8_t i = 0; i < JOINT_CNT; i++)
            HTMotorOuterLoop(joint[i], OPEN_LOOP); // 改回直接开环扭矩输入,让电调对扭矩闭环

        return; // 退出函数不再执行关节指令
    }
    else
        chassis_status = ROBOT_STOP;

    // 还在复位中,关节改为位置环,执行复位
    for (uint8_t i = 0; i < JOINT_CNT; i++)
    {
        HTMotorOuterLoop(joint[i], ANGLE_LOOP);
        HTMotorSetRef(joint[i], 0);
    }
}

// 工作状态设定
static void WokingStateSet()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_RESET) // 复位模式
    {
        ResetChassis();
        return;
    }
    else if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) // 未收到遥控器和云台指令底盘进入急停
    {
        // 目标速度置0
        chassis.target_v = 0;
        // 清空腿长和距离
        l_side.target_len = r_side.target_len = 0.12;
        chassis.dist = chassis.target_dist = 0;
        // 角度输入为当前角度
        // chassis_cmd_recv.offset_angle = chassis.target_yaw = chassis.yaw;

        for (uint8_t i = 0; i < JOINT_CNT; i++)
            HTMotorStop(joint[i]);
        for (uint8_t i = 0; i < DRIVEN_CNT; i++)
            LKMotorStop(driven[i]);
        return; // 关闭所有电机,发送的指令为零
    }

    // 运动模式
    EnableAllMotor();
    // 保证关节电机为开环扭矩控制
    for (uint8_t i = 0; i < JOINT_CNT; i++)
        HTMotorOuterLoop(joint[i], OPEN_LOOP);

    // 设置目标速度/腿长/距离
    l_side.target_len += 0.00001f*(float)chassis_cmd_recv.delta_leglen;
    r_side.target_len += 0.00001f*(float)chassis_cmd_recv.delta_leglen;
    // 腿长限幅
    VAL_LIMIT(l_side.target_len, 0.12, 0.30);
    VAL_LIMIT(r_side.target_len, 0.12, 0.30);

    // 加速度限幅,防止键盘控制摔倒
    chassis.target_v += sign(chassis_cmd_recv.vx - chassis.target_v) * MAX_ACC_REF * del_t;
    // VAL_LIMIT(chassis.target_v, 0.002, 2.0);
    // 角度输入
    if (chassis_cmd_recv.chassis_mode == CHASSIS_FREE_DEBUG)
    {
        chassis.target_yaw = chassis_cmd_recv.offset_angle;
    }
    chassis.target_yaw = chassis.yaw + chassis_cmd_recv.offset_angle * DEGREE_2_RAD;

    // TODO 转向速度限幅

    // TODO 最大dist误差限幅

    // TODO 最大速度误差限幅
}

/**
 * @brief 将电机和imu的数据组装为LinkNPodParam结构体和chassisParam结构体
 *
 * @note HT04电机上电的编码器位置为零(校准过),请看Link2Pod()的note,以及HT04.c中的电机解码部分
 * @note 海泰04电机顺时针旋转为正; LK9025电机逆时针旋转为正,此处皆需要转换为模型中给定的正方向
 *
 */
static void ParamAssemble()
{
    // 机体参数,视为平面刚体
    chassis.pitch = Chassis_IMU_data->Pitch * DEGREE_2_RAD;
    chassis.pitch_w = Chassis_IMU_data->Gyro[0];
    chassis.yaw = Chassis_IMU_data->YawTotalAngle * DEGREE_2_RAD;
    chassis.wz = Chassis_IMU_data->Gyro[2];
    chassis.roll = Chassis_IMU_data->Roll * DEGREE_2_RAD;
    chassis.roll_w = Chassis_IMU_data->Gyro[1];

    // HT04电机的角度是顺时针为正,LK9025电机的角度是逆时针为正
    l_side.phi1 = PI + LIMIT_LINK_RAD - lb->measure.total_angle;
    l_side.phi1_w = -lb->measure.speed_rads;
    l_side.phi4 = -lf->measure.total_angle - LIMIT_LINK_RAD;
    l_side.phi4_w = -lf->measure.speed_rads;
    l_side.w_ecd = l_driven->measure.speed_rads;

    r_side.phi1 = PI + LIMIT_LINK_RAD + rb->measure.total_angle;
    r_side.phi1_w = rb->measure.speed_rads;
    r_side.phi4 = rf->measure.total_angle - LIMIT_LINK_RAD;
    r_side.phi4_w = rf->measure.speed_rads;
    r_side.w_ecd = -r_driven->measure.speed_rads;
}

static void SynthesizeMotion() /* 腿部控制:抗劈叉; 轮子控制:转向 */
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_FREE_DEBUG ||
        chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) // 底盘跟随
    {
        float p_ref = PIDCalculate(&steer_p_pid, chassis.yaw, chassis.target_yaw);
        PIDCalculate(&steer_v_pid, chassis.wz, p_ref);
    }
    else if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE) // 小陀螺
    {
        PIDCalculate(&steer_v_pid, chassis.wz, (float)chassis_cmd_recv.rotate_w);
    }
    else if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE_REVERSE)
    {
        PIDCalculate(&steer_v_pid, chassis.wz, (float)chassis_cmd_recv.rotate_w);
    }
    l_side.T_wheel -= steer_v_pid.Output;
    r_side.T_wheel += steer_v_pid.Output;

    // 抗劈叉
    static float swerving_speed_ff, ff_coef = 3;
    swerving_speed_ff = ff_coef * steer_v_pid.Output; // 用于抗劈叉的前馈
    PIDCalculate(&anti_crash_pid, l_side.phi5 - r_side.phi5, 0);
    l_side.T_hip += anti_crash_pid.Output - swerving_speed_ff;
    r_side.T_hip -= anti_crash_pid.Output - swerving_speed_ff;
}

static void LegControl() /* 腿长控制和Roll补偿 */
{
    PIDCalculate(&roll_compensate_pid, chassis.roll, 0);
    l_side.target_len += roll_compensate_pid.Output;
    r_side.target_len -= roll_compensate_pid.Output;

    static float gravity_ff = 60;
    static float roll_extra_comp_p = 400;
    float roll_comp = roll_extra_comp_p * chassis.roll;
    l_side.F_leg = PIDCalculate(&leglen_pid_l, l_side.height, l_side.target_len) + gravity_ff - roll_comp;
    r_side.F_leg = PIDCalculate(&leglen_pid_r, r_side.height, r_side.target_len) + gravity_ff + roll_comp;
}

static void WattLimitSet() /* 设定运动模态的输出 */
{
    HTMotorSetRef(lf, 0.2857f * -l_side.T_front); // 根据扭矩常数计算得到的系数
    HTMotorSetRef(lb, 0.2857f * -l_side.T_back);
    HTMotorSetRef(rf, 0.2857f * r_side.T_front);
    HTMotorSetRef(rb, 0.2857f * r_side.T_back);
    LKMotorSetRef(l_driven, 195.3125 * l_side.T_wheel);
    LKMotorSetRef(r_driven, 195.3125 * -r_side.T_wheel);
}

// 裁判系统,双板通信,电容功率控制等
static void CommNPower()
{
    //static uint8_t supercap_send_cnt = 0;
    // CANCommSend(cmd_can_comm, (void *)&chassis_feedback_data);
    // supercap_send_cnt++;
    // if (supercap_send_cnt % 5 == 0)
    // {
    //     DataSend2Cap[0] = referee_data->PowerHeatData.buffer_energy; // 200hz发送
    //     DataSend2Cap[1] = referee_data->GameRobotState.chassis_power_limit;
    //     SuperCapSend(cap, (uint8_t *)&DataSend2Cap);
    //     supercap_send_cnt = 0;
    // }
    /* 更新ui数据 */
    ui_data.direction = chassis_cmd_recv.direction;
    ui_data.friction_mode = chassis_cmd_recv.friction_mode;
    ui_data.loader_mode = chassis_cmd_recv.loader_mode;
    ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;
    ui_data.ui_mode = chassis_cmd_recv.ui_mode;
    ui_data.vision_mode = chassis_cmd_recv.vision_mode;
    memcpy(ui_data.coord, l_side.coord, sizeof(l_side.coord));
}

void BalanceTask()
{
    del_t = DWT_GetDeltaT(&balance_dwt_cnt);

    BuzzerOn();
    // 切换遥控器控制or云台板控制
    ControlSwitch();
    // 设置目标参数和工作模式
    WokingStateSet();
    // 裁判系统,双板通信,电容功率控制等
    CommNPower();
    // 参数组装
    ParamAssemble();
    // 将五连杆映射成单杆
    Link2Leg(&l_side, &chassis);
    Link2Leg(&r_side, &chassis);
    // 通过卡尔曼滤波估计机体速度
    SpeedEstimation(&l_side, &r_side, &chassis, Chassis_IMU_data, del_t);
    // 根据单杆计算处的角度和杆长,计算反馈增益
    CalcLQR(&l_side, &chassis);
    CalcLQR(&r_side, &chassis);
    // 转向和抗劈叉
    SynthesizeMotion();
    // 腿长控制,保持机体水平
    LegControl();
    // VMC映射成关节输出
    VMCProject(&l_side);
    VMCProject(&r_side);
    // 驱动轮支持力解算
    NormalForceSolve(&l_side, Chassis_IMU_data);
    NormalForceSolve(&r_side, Chassis_IMU_data);

    // stop表示复位尚未完成,reset表明还未切换到其他模式,故都不执行运动模态的代码
    if (chassis_status == ROBOT_STOP ||
        chassis_cmd_recv.chassis_mode == CHASSIS_RESET ||
        chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
        return; // 复位模态或急停,直接退出

    // 运动模态,电机输出映射和限幅
    WattLimitSet();
}