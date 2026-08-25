/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *  Version    Date            Author       Modification
 *  V1.0.0     Dec-29-2025     刘安东        1. done
   @verbatim
  ==============================================================================

  ==============================================================================
 *
 */
#include "chassis.h"
#include "can.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "message_center.h"
#include "motor_def.h"
#include "power_control.h"
#include "referee_task.h"
#include "robot_def.h"
#include "super_cap.h"

#include "arm_math.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "referee_UI.h"
#include "string.h"

#include "ChassisR_task.h"
#include "ChassisL_task.h"
#include "VMC.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体
 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
static Publisher_t *chassis_pub;  // 用于发布底盘的数据
static Subscriber_t *chassis_sub; // 用于订阅底盘的控制命令
#endif                            // CHASSIS_BOARD
#ifdef ONE_BOARD
attitude_t *Chassis_IMU_data;
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static PIDInstance buffer_PID;       // 用于底盘的缓冲能量PID
static referee_info_t *referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap; // 超级电容
static DMMotorInstance *T1_l, *T2_l, *T1_r, *T2_r,*W_r,*W_l,*Yaw; // left right forward back
// static DJIMotorInstance *W_r,*W_l; // DJI电机实例

//用户自定义变量
extern vmc_leg_t right;//右腿
extern vmc_leg_t left; //左腿
/* 用于自旋变速策略的时间变量 */
// static float t;
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
//暂无用
static float J1_l, J2_l, J1_r, J2_r; // 底盘速度解算后的临时输出,待进行限幅
static float J_Wr,J_Wl;              // 轮子速度设定值
volatile uint8_t chassis_init_done = 0; // 底盘初始化完成标志

void ChassisInit() {
  // 移植需求
  // 四个DM电机初始化，VMC初始化,腿长PID初始化，四个补偿PID初始化
  //  四个轮子的参数一样,改tx_id和反转标志位即可
  // Chassis_IMU_data = INS_Init(); // 底盘IMU初始化
  Motor_Init_Config_s chassis_Tl_motor_config = {
      .can_init_config.can_handle = &hcan1,
      .motor_type = DM8009p,
  };
  Motor_Init_Config_s chassis_Tr_motor_config = {
      .can_init_config.can_handle = &hcan2,
      .motor_type = DM8009p,
  };
  Motor_Init_Config_s chassis_W_motor_config = {
      .motor_type = DM3519,
  };
  Motor_Init_Config_s chassis_Yaw_motor_config = {
    .controller_param_init_config = {
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
      .motor_type = DM4310,
      .cali_encoder_on_init = 1, // 使能前标定编码器零位
  };

  //  @todo:
  //  当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
  // 使用功率控制的电机需要使用PowerControlInit()函数初始化,因为电机的控制方式不同

  chassis_Tr_motor_config.can_init_config.tx_id = 0x01,
  chassis_Tr_motor_config.can_init_config.rx_id = 0x11,
  chassis_Tr_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
  T1_r = DMMotorInit(&chassis_Tr_motor_config);//右前

  chassis_Tr_motor_config.can_init_config.tx_id = 0x02,
  chassis_Tr_motor_config.can_init_config.rx_id = 0x12,
  chassis_Tr_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
  T2_r = DMMotorInit(&chassis_Tr_motor_config);//右后

  chassis_Tl_motor_config.can_init_config.tx_id = 0x03,
  chassis_Tl_motor_config.can_init_config.rx_id = 0x13,
  chassis_Tl_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
  T1_l = DMMotorInit(&chassis_Tl_motor_config);//左前

  chassis_Tl_motor_config.can_init_config.tx_id = 0x04,
  chassis_Tl_motor_config.can_init_config.rx_id = 0x14,
  chassis_Tl_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
  T2_l = DMMotorInit(&chassis_Tl_motor_config);//左后

//轮电机
  chassis_W_motor_config.can_init_config.tx_id = 0x06,
  chassis_W_motor_config.can_init_config.rx_id = 0x16,
  chassis_W_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
  chassis_W_motor_config.can_init_config.can_handle = &hcan2,
  W_r = DMMotorInit(&chassis_W_motor_config);
 
  chassis_W_motor_config.can_init_config.tx_id = 0x08,
  chassis_W_motor_config.can_init_config.rx_id = 0x18,
  chassis_W_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
  chassis_W_motor_config.can_init_config.can_handle = &hcan1,
  W_l = DMMotorInit(&chassis_W_motor_config);

//云台电机
  chassis_Yaw_motor_config.can_init_config.tx_id = 0x10,
  chassis_Yaw_motor_config.can_init_config.rx_id = 0x20,
  chassis_Yaw_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
  chassis_Yaw_motor_config.can_init_config.can_handle = &hcan2,
  Yaw = DMMotorInit(&chassis_Yaw_motor_config);

  	VMC_init(&right);
    VMC_init(&left);
  // referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI
  /* Buffer环暂未测试，逻辑是计算期望buffer与实际buffer的差值，转换为冗余的功率，todo：输入给功率控制部分，待完善
   */
  PID_Init_Config_s Buffer_pid_conf = {
      .Kp = 0.1,
      .Ki = 0,
      .Kd = 0,
      .IntegralLimit = 1000,
      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                 PID_Derivative_On_Measurement,
      .MaxOut = 1000,
  };
  PIDInit(&buffer_PID, &Buffer_pid_conf); // 缓冲能量PID初始化
  SuperCap_Init_Config_s cap_conf = {
      .can_config = {
          .can_handle = &hcan2,
          .tx_id = 0x302, // 超级电容默认接收id
          .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
      }};
  cap = SuperCapInit(&cap_conf); // 超级电容初始化
  // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
  Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

  CANComm_Init_Config_s comm_conf = {
      .can_config =
          {
              .can_handle = &hcan1,
              .tx_id = 0x25,
              .rx_id = 0x15,
          },
      .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
      .send_data_len = sizeof(Chassis_Upload_Data_s),
  };
  chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
  // 使用这个与用Sub-Pub的区别在于不需要额外的内存拷贝，直接在can
  // comm的buffer上操作
#endif // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
  chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
  chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
  chassis_init_done = 1; // 所有电机和外设初始化完成
}

/* ==================== 电机实例访问器函数实现 ==================== */
/**
 * @brief 获取右前关节电机实例(T1_r)
 * @note 用于ChassisR_task访问右腿电机
 */
DMMotorInstance* Chassis_GetMotor_T1R(void) {
    return T1_r;
}

/**
 * @brief 获取右后关节电机实例(T2_r)
 * @note 用于ChassisR_task访问右腿电机
 */
DMMotorInstance* Chassis_GetMotor_T2R(void) {
    return T2_r;
}

/**
 * @brief 获取左前关节电机实例(T1_l)
 * @note 用于ChassisL_task访问左腿电机
 */
DMMotorInstance* Chassis_GetMotor_T1L(void) {
    return T1_l;
}

/**
 * @brief 获取左后关节电机实例(T2_l)
 * @note 用于ChassisL_task访问左腿电机
 */
DMMotorInstance* Chassis_GetMotor_T2L(void) {
    return T2_l;
}

/**
 * @brief 获取右侧轮毂电机实例(W_r)
 * @note 用于ChassisR_task访问右侧轮毂电机
 */
DMMotorInstance* Chassis_GetMotor_WR(void) {
    return W_r;
}

/**
 * @brief 获取左侧轮毂电机实例(W_l)
 * @note 用于ChassisL_task访问左侧轮毂电机
 */
DMMotorInstance* Chassis_GetMotor_WL(void) {
    return W_l;
}

/**
 * @brief 获取Yaw电机的数据指针
 * @note 用于左右腿任务读取Yaw数据
 */
DMMotorInstance* Chassis_GetMotor_Yaw(void) {
    return Yaw ;
}

/**
 * @brief 获取底盘IMU数据指针
 * @note 用于左右腿任务获取姿态数据
 */
attitude_t* Chassis_GetIMUData(void) {
    return Chassis_IMU_data;
}

/**
 * @brief 获取板间通信数据指针
 * @note 用于左右腿任务读取遥控器数据
 */
CANCommInstance* Chassis_CAN_COMM(void) {
    return chasiss_can_comm ;
}

void Chassis_DMMotorEnable(DMMotorInstance *motor)
{
    if (motor == NULL || motor->motor_can_instace == NULL) {
        return;
    }

    DMMotorEnable(motor);
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);
    motor->motor_can_instace->tx_buff[7] = DM_CMD_MOTOR_MODE;
    CANTransmit(motor->motor_can_instace, 1);
}

/**
 * @brief 轮毂电机延时启动控制
 * @param first_startup 第一次启动标志指针
 * @param startup_time 启动时间戳指针
 * @param wheel_motor_enabled 轮毂电机使能标志指针
 * @param delay_ms 延时时间(毫秒)
 * @return uint8_t 返回当前轮毂电机是否可以启动 (1=可以启动, 0=延时中)
 * @note 第一次起身时先发关节电机，延时后再发轮毂电机
 *       在制动模式下调用 Chassis_ResetWheelStartup() 重置状态
 */
uint8_t Chassis_WheelMotorStartupDelay(uint8_t *first_startup, uint32_t *startup_time, 
                                        uint8_t *wheel_motor_enabled, float delay_ms)
{
    if (*first_startup) {
        if (*startup_time == 0) {
            *startup_time = DWT_GetTimeline_ms();  // 记录启动时间
        }
        // 检查是否已过延时时间
        if ((DWT_GetTimeline_ms() - *startup_time) >= delay_ms) {
            *wheel_motor_enabled = 1;
            *first_startup = 0;  // 标记已完成第一次启动
        }
    }
    
    return *wheel_motor_enabled;
}

/**
 * @brief 重置轮毂电机启动状态
 * @param first_startup 第一次启动标志指针
 * @param startup_time 启动时间戳指针
 * @param wheel_motor_enabled 轮毂电机使能标志指针
 * @note 在制动模式下调用，重置状态以便下次进入运行模式时重新计时
 */
void Chassis_ResetWheel(uint8_t *first_startup, uint32_t *startup_time, 
                                uint8_t *wheel_motor_enabled)
{
    *wheel_motor_enabled = 0;
    *first_startup = 1;
    *startup_time = 0;
}

float Chassis_WrapAngleToPi(float angle)
{
    const float full_turn = 2.0f * PI;

    while (angle > PI) {
        angle -= full_turn;
    }

    while (angle < -PI) {
        angle += full_turn;
    }

    return angle;
}

void Chassis_ResetMoveState(Balance_Chassis_e *chassis, float x_set)
{
    chassis->chassis_move_balance->x_set = x_set;
    chassis->chassis_move_balance->x_filter = 0.0f;
    chassis->chassis_move_balance->v_set = 0.0f;
    chassis->chassis_move_balance->v_filter = 0.0f;
    chassis->chassis_move_balance->v_target = 0.0f;
}

void Chassis_ResetRotateState(Balance_Chassis_e *chassis)
{
    Chassis_ResetMoveState(chassis, 0.0f);
    chassis->chassis_move_balance->theta_compensate = 0.0f;
}

void Chassis_UpdateTotalYaw(Balance_Chassis_e *chassis, float cmd_yaw, uint8_t rotate_start)
{
    static float last_total_yaw = 0.0f;
    float new_total_yaw = chassis->chassis_move_balance->total_yaw - cmd_yaw * 10.0f;
    float total_yaw_delta = new_total_yaw - last_total_yaw;

    if (total_yaw_delta < 0.0f) {
        total_yaw_delta = -total_yaw_delta;
    }

    if (rotate_start == 0 && total_yaw_delta > 0.01f) {
        Chassis_ResetMoveState(chassis, 0.1f);
    }

    chassis->chassis_move_balance->total_yaw = new_total_yaw;
    last_total_yaw = new_total_yaw;
}