#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include <stdint.h>

/* ====== 协议帧参数 (与上位机 auto_shoot/gimbal.hpp 完全一致) ====== */

#define VISION_FRAME_HEAD_0 0x5A  // 帧头字节0
#define VISION_FRAME_HEAD_1 0xA5  // 帧头字节1
#define VISION_FRAME_TAIL_0 0x7F  // 帧尾字节0
#define VISION_FRAME_TAIL_1 0xFE  // 帧尾字节1

/* 缓冲区大小 (与结构体大小严格对应) */
#define VISION_SEND_SIZE 43u  // GimbalToVision 帧总字节数
#define VISION_RECV_SIZE 29u  // VisionToGimbal 帧总字节数

/* ====== 模式枚举 (与上位机 GimbalMode / VisionToGimbal.mode 一致) ====== */

/* 下位机发给上位机的工作模式 (对应 GimbalMode) */
typedef enum {
    VISION_MODE_IDLE       = 0,  // 空闲
    VISION_MODE_AUTO_AIM   = 1,  // 自瞄
    VISION_MODE_SMALL_BUFF = 2,  // 小符
    VISION_MODE_BIG_BUFF   = 3,  // 大符
} Vision_Mode_e;

/* 上位机发来的控制模式 (对应 VisionToGimbal.mode) */
typedef enum {
    VISION_CTRL_NONE = 0,  // 不控制云台
    VISION_CTRL_AIM  = 1,  // 控制云台但不开火
    VISION_CTRL_FIRE = 2,  // 控制云台且开火
} Vision_Ctrl_e;

/* ====== 接收数据结构 (上位机→下位机, 对应 VisionToGimbal) ====== */

#pragma pack(1)
typedef struct {
    uint8_t control;    // 是否启用自瞄控制 (mode >= 1 时为 1)
    uint8_t shoot;      // 是否开火 (mode == 2 时为 1)
    float   yaw;        // yaw 目标偏移 (rad, 相对值)
    float   yaw_vel;    // yaw 角速度前馈 (rad/s)
    float   yaw_acc;    // yaw 角加速度前馈 (rad/s²)
    float   pitch;      // pitch 目标偏移 (rad, 相对值)
    float   pitch_vel;  // pitch 角速度前馈 (rad/s)
    float   pitch_acc;  // pitch 角加速度前馈 (rad/s²)
} Vision_Recv_s;
#pragma pack()

/* ====== 发送数据结构 (下位机→上位机, 对应 GimbalToVision) ====== */

#pragma pack(1)
typedef struct {
    float    q[4];          // IMU 四元数 (wxyz 顺序)
    float    yaw;           // yaw 角度 (rad)
    float    yaw_vel;       // yaw 角速度 (rad/s)
    float    pitch;         // pitch 角度 (rad)
    float    pitch_vel;     // pitch 角速度 (rad/s)
    float    bullet_speed;  // 实时弹速 (m/s)
    uint16_t bullet_count;  // 累计发弹计数
    uint8_t  mode;          // 工作模式 (Vision_Mode_e)
} Vision_Send_s;
#pragma pack()

/* ====== 外部接口 ====== */

/**
 * @brief 初始化视觉通信模块
 * @param _handle 串口句柄 (VISION_USE_UART 时传入句柄, 其余模式传 NULL)
 * @return 接收数据结构体指针
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 向上位机发送一帧数据 (GimbalToVision, 43字节)
 * @note  在 INS 任务或 Robot 任务中定期调用
 */
void VisionSend(void);

/**
 * @brief 设置 IMU 四元数 (wxyz 顺序, 与上位机 GimbalToVision.q 一致)
 * @param q 四元数数组 [w, x, y, z]
 */
void VisionSetQuaternion(const float *q);

/**
 * @brief 设置云台姿态角和角速度
 * @param yaw       yaw 角度 (rad)
 * @param pitch     pitch 角度 (rad)
 * @param yaw_vel   yaw 角速度 (rad/s)
 * @param pitch_vel pitch 角速度 (rad/s)
 */
void VisionSetAltitude(float yaw, float pitch, float yaw_vel, float pitch_vel);

/**
 * @brief 设置机器人状态信息
 * @param mode         当前工作模式 (Vision_Mode_e)
 * @param bullet_speed 实时弹速 (m/s)
 * @param bullet_count 累计发弹计数
 */
void VisionSetStatus(Vision_Mode_e mode, float bullet_speed, uint16_t bullet_count);

/**
 * @brief 检查视觉通信是否在线
 * @return 1=在线, 0=离线
 */
uint8_t VisionIsOnline(void);

#endif // !MASTER_PROCESS_H