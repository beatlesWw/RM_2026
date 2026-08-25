#include "gimbal.h"
#include "motor_def.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "dmmotor.h"
#include "controller.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DMMotorInstance*yaw_motor, *pitch_motor;

static PIDInstance pitch_angle_pid; // pitch角度环PID
static PIDInstance pitch_speed_pid; // pitch速度环PID
static PIDInstance yaw_angle_pid;   // yaw角度环PID
static PIDInstance yaw_speed_pid;   // yaw速度环PID
static float Gimbal_dyaw;              // yaw角度环输出(速度环设定值)
static float yaw_T;                 // yaw速度环输出(力矩)

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU

void DMMotorSetMode_gimbal(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static uint8_t DMMotorIsOnline_gimbal(DMMotorInstance *motor)
{
    return DaemonIsOnline(motor->motor_daemon);
}

void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    
    // 初始化pitch串级PID控制器
    PID_Init_Config_s pitch_angle_pid_config = {
        .Kp = 20.0f,
        .Ki = 0.0f,
        .Kd = 2.5f,
        .MaxOut = 5.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 5.0f,
    };
    PIDInit(&pitch_angle_pid, &pitch_angle_pid_config);
    
    PID_Init_Config_s pitch_speed_pid_config = {
        .Kp = 0.5f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 5.0f,
        .DeadBand = 0.05f,  // 添加死区，过滤小噪声
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_OutputFilter,
        .IntegralLimit = 100.0f,
        .Derivative_LPF_RC = 0.05f,  // 微分低通滤波器，截止频率约3Hz
        .Output_LPF_RC = 0.02f,  // 输出低通滤波器，平滑输出
    };
    PIDInit(&pitch_speed_pid, &pitch_speed_pid_config);

    // 初始化yaw串级PID控制器
    PID_Init_Config_s yaw_angle_pid_config = {
        .Kp = 25.0f,
        .Ki = 0.0f,
        .Kd = 0.5f,
        .MaxOut = 8.5f,
        .IntegralLimit = 0.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    };
    PIDInit(&yaw_angle_pid, &yaw_angle_pid_config);

    PID_Init_Config_s yaw_speed_pid_config = {
        .Kp = 0.50f,
        .Ki = 0.08f,
        .Kd = 0.0f,
        .MaxOut = 5.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_OutputFilter,
        .IntegralLimit = 3.0f,
        .Derivative_LPF_RC = 0.05f,
        .Output_LPF_RC = 0.02f,
    };
    PIDInit(&yaw_speed_pid, &yaw_speed_pid_config);

    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .motor_type = DM4310};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .motor_type = DM4310,
    };

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    // yaw_motor = DJIMotorInit(&yaw_config);
    pitch_config.can_init_config.tx_id = 0x02,
    pitch_config.can_init_config.rx_id = 0x12,
    pitch_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    pitch_motor = DMMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    if (!DMMotorIsOnline_gimbal(pitch_motor))
    {
        DMMotorEnable(pitch_motor);
        DMMotorSetMode_gimbal(DM_CMD_MOTOR_MODE, pitch_motor);
        gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;
    }

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DMMotorSendPair(pitch_motor, 0);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式，底盘跟随云台

//我的上层板不挂Yaw，只负责计算并发给下层板     
        pitch_motor->gimbal_dpitch = PIDCalculate(&pitch_angle_pid, 0.0f-gimba_IMU_data->Pitch, gimbal_cmd_recv.pitch);    
        pitch_motor->gimbal_pitch_T = PIDCalculate(&pitch_speed_pid, 0.0f-gimba_IMU_data->Gyro[0], pitch_motor->gimbal_dpitch);      
        // 将力矩输出发送给电机
        // DMMotorSendPair(pitch_motor, 0);
        DMMotorSendPair(pitch_motor, pitch_motor->gimbal_pitch_T);
//前馈测试
        // DMMotorSendPair(pitch_motor, -0.8f);
        Gimbal_dyaw = PIDCalculate(&yaw_angle_pid, gimba_IMU_data->YawTotalAngle, gimbal_cmd_recv.yaw);
        yaw_T = PIDCalculate(&yaw_speed_pid, gimba_IMU_data->Gyro[2], Gimbal_dyaw);
        gimbal_feedback_data.T_Gimbal_Yaw = yaw_T;
        break;

// 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转
    case GIMBAL_FREE_MODE:   
        pitch_motor->gimbal_dpitch = PIDCalculate(&pitch_angle_pid, 0.0f-gimba_IMU_data->Pitch, gimbal_cmd_recv.pitch);
        gimbal_cmd_recv.pitch= 0.0f-PIDCalculate(&pitch_speed_pid, 0.0f-gimba_IMU_data->Gyro[0], pitch_motor->gimbal_dpitch);
        // DMMotorSendPair(pitch_motor, gimbal_cmd_recv.pitch);
        DMMotorSendPair(pitch_motor, pitch_motor->gimbal_pitch_T);
        
        // yaw串级PID控制
        Gimbal_dyaw = PIDCalculate(&yaw_angle_pid, gimba_IMU_data->YawTotalAngle, gimbal_cmd_recv.yaw);
        yaw_T = PIDCalculate(&yaw_speed_pid, gimba_IMU_data->Gyro[2], Gimbal_dyaw);
        gimbal_feedback_data.T_Gimbal_Yaw = yaw_T;
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    // gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}