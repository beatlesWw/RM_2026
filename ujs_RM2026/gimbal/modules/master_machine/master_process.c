/**
 * @file master_process.c
 * @brief 视觉上位机通信模块
 * @details 适配上位机 auto_shoot (gimbal.hpp) 协议:
 *          - 发送: GimbalToVision 帧 (43字节, 帧头{0x5A,0xA5} + 帧尾{0x7F,0xFE})
 *          - 接收: VisionToGimbal 帧 (29字节, 帧头{0x5A,0xA5} + 帧尾{0x7F,0xFE})
 *          通过 robot_def.h 中的宏选择通信方式:
 *          - VISION_USE_UART: 硬件串口 (推荐, 115200bps)
 *          - VISION_USE_VCP:  USB 虚拟串口
 */
#include "master_process.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include <string.h>

/* ====== 模块内部数据 ====== */

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;

/* ====== Setter 函数 (由应用层在任务中调用) ====== */

void VisionSetQuaternion(const float *q)
{
    send_data.q[0] = q[0];
    send_data.q[1] = q[1];
    send_data.q[2] = q[2];
    send_data.q[3] = q[3];
}

void VisionSetAltitude(float yaw, float pitch, float yaw_vel, float pitch_vel)
{
    send_data.yaw       = yaw;
    send_data.pitch     = pitch;
    send_data.yaw_vel   = yaw_vel;
    send_data.pitch_vel = pitch_vel;
}

void VisionSetStatus(Vision_Mode_e mode, float bullet_speed, uint16_t bullet_count)
{
    send_data.mode         = (uint8_t)mode;
    send_data.bullet_speed = bullet_speed;
    send_data.bullet_count = bullet_count;
}

uint8_t VisionIsOnline(void)
{
    return DaemonIsOnline(vision_daemon_instance);
}

/* ====== 帧打包 (内部共用, 由 VisionSend 调用) ====== */

/**
 * @brief 将 send_data 打包为 GimbalToVision 帧写入 buf
 * @note  帧布局 (43字节):
 *        [0-1]  帧头  {0x5A, 0xA5}
 *        [2]    mode
 *        [3-18] q[4]  (wxyz, 16字节)
 *        [19-22] yaw
 *        [23-26] yaw_vel
 *        [27-30] pitch
 *        [31-34] pitch_vel
 *        [35-38] bullet_speed
 *        [39-40] bullet_count (小端序)
 *        [41-42] 帧尾 {0x7F, 0xFE}
 */
static void PackGimbalToVision(uint8_t *buf)
{
    buf[0] = VISION_FRAME_HEAD_0;
    buf[1] = VISION_FRAME_HEAD_1;
    buf[2] = send_data.mode;
    memcpy(&buf[3],  send_data.q,            16);
    memcpy(&buf[19], &send_data.yaw,           4);
    memcpy(&buf[23], &send_data.yaw_vel,       4);
    memcpy(&buf[27], &send_data.pitch,         4);
    memcpy(&buf[31], &send_data.pitch_vel,     4);
    memcpy(&buf[35], &send_data.bullet_speed,  4);
    buf[39] = (uint8_t)(send_data.bullet_count & 0xFF);
    buf[40] = (uint8_t)(send_data.bullet_count >> 8);
    buf[41] = VISION_FRAME_TAIL_0;
    buf[42] = VISION_FRAME_TAIL_1;
}

/* ====== 帧解包 (内部共用, 由 DecodeVision 调用) ====== */

/**
 * @brief 从 buf 解析 VisionToGimbal 帧到 recv_data
 * @note  帧布局 (29字节):
 *        [0-1]  帧头  {0x5A, 0xA5}
 *        [2]    mode (0=不控制, 1=控制不开火, 2=控制开火)
 *        [3-6]  yaw
 *        [7-10] yaw_vel
 *        [11-14] yaw_acc
 *        [15-18] pitch
 *        [19-22] pitch_vel
 *        [23-26] pitch_acc
 *        [27-28] 帧尾 {0x7F, 0xFE}
 * @return 1=解析成功, 0=帧头或帧尾错误
 */
static uint8_t UnpackVisionToGimbal(const uint8_t *buf)
{
    // 检查帧头
    if (buf[0] != VISION_FRAME_HEAD_0 || buf[1] != VISION_FRAME_HEAD_1) {
        LOGWARNING("[Vision] header fail: %02X %02X", buf[0], buf[1]);
        return 0;
    }
    // 检查帧尾
    if (buf[VISION_RECV_SIZE - 2] != VISION_FRAME_TAIL_0 ||
        buf[VISION_RECV_SIZE - 1] != VISION_FRAME_TAIL_1) {
        LOGWARNING("[Vision] tail fail: %02X %02X",
                   buf[VISION_RECV_SIZE - 2], buf[VISION_RECV_SIZE - 1]);
        return 0;
    }

    uint8_t mode       = buf[2];
    recv_data.control  = (mode >= 1) ? 1 : 0;
    recv_data.shoot    = (mode == 2) ? 1 : 0;
    memcpy(&recv_data.yaw,       &buf[3],  4);
    memcpy(&recv_data.yaw_vel,   &buf[7],  4);
    memcpy(&recv_data.yaw_acc,   &buf[11], 4);
    memcpy(&recv_data.pitch,     &buf[15], 4);
    memcpy(&recv_data.pitch_vel, &buf[19], 4);
    memcpy(&recv_data.pitch_acc, &buf[23], 4);
    return 1;
}

/* ====== 离线回调 (UART/VCP 共用) ====== */

#ifdef VISION_USE_UART
#include "bsp_usart.h"
static USARTInstance *vision_usart_instance;
#endif

static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif
    LOGWARNING("[Vision] offline, restart communication.");
}

/* ====== UART 通信实现 ====== */

#ifdef VISION_USE_UART

static void DecodeVision(void)
{
    if (UnpackVisionToGimbal(vision_usart_instance->recv_buff)) {
        DaemonReload(vision_daemon_instance);
    }
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf = {
        .module_callback = DecodeVision,
        .recv_buff_size  = VISION_RECV_SIZE,
        .usart_handle    = _handle,
    };
    vision_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback,
        .owner_id     = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend(void)
{
    // buff 必须为 static, 保证 DMA 发送期间不被释放 (析构陷阱!)
    static uint8_t send_buff[VISION_SEND_SIZE];
    PackGimbalToVision(send_buff);
    USARTSend(vision_usart_instance, send_buff, VISION_SEND_SIZE, USART_TRANSFER_DMA);
}

#endif // VISION_USE_UART

/* ====== VCP 通信实现 ====== */

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVisionVCP(uint16_t recv_len)
{
    if (recv_len < VISION_RECV_SIZE) return;
    if (UnpackVisionToGimbal(vis_recv_buff)) {
        DaemonReload(vision_daemon_instance);
    }
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle);
    USB_Init_Config_s conf = {.rx_cbk = DecodeVisionVCP};
    vis_recv_buff = USBInit(conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback,
        .owner_id     = NULL,
        .reload_count = 5,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend(void)
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    PackGimbalToVision(send_buff);
    USBTransmit(send_buff, VISION_SEND_SIZE);
}

#endif // VISION_USE_VCP
