#ifndef MINIPC_H
#define MINIPC_H
/*	include	*/
#include "usbd_cdc_if.h"
#include "RMLibHead.h"
#include "robot.h"
#include "RMQueue.h"
#include "bsp_dwt.h"
#include "Task_Init.h"
/* Define----------------------------------------------------------------------*/
#define USB_MAX_PACKET_SIZE 64      // usb一包最大64字节
#define SEND_SOF ((uint8_t)0x5A)    // 发送包头
#define RECEIVE_SOF ((uint8_t)0x5A) // 接收包头
#define SEND_EOF ((uint8_t)0xA5)    // 发送帧尾
#define RECEIVE_EOF ((uint8_t)0xA5) // 接收帧尾
#define SEND_IMU_DATA_ID ((uint8_t)0x01)
#define RECEIVE_VISION_DATA_ID ((uint8_t)0x02)
/* struct */
// 帧头结构体
typedef struct
{
    uint8_t sof; // 数据帧起始字节，固定值为 0x5A
    uint8_t len; // 数据段长度
    uint8_t id;  // 数据段id
} __attribute__((packed)) FrameHeader_t;
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x02
    uint32_t time_stamp;
    struct
    {
        uint8_t self_color; // robot_color
        float bullet_speed;
        float yaw;   // rad
        float pitch; // rad
        float roll;  // rad

        float yaw_vel;   // rad/s
        float pitch_vel; // rad/s
        float roll_vel;  // rad/s
    } __attribute__((packed)) data;

    uint8_t eof; // 数据帧末尾字节，固定值为 0xA5
} __attribute__((packed)) SendData_t;
typedef struct
{
    FrameHeader_t frame_header;

    struct
    {
        uint8_t FireFlag;
        float Ref_yaw;
        float Ref_pitch;
        float Ref_Vyaw;
        float Ref_Vpitch;
        float Ref_aYaw;
        float Ref_aPitch;
        float dis;
    } __attribute__((packed)) data;

    uint8_t eof;
} __attribute__((packed)) ReceiveData_t;
// MiniPC实例
typedef struct
{
    SendData_t send_data_imu;            // 发送给MiniPC的IMU数据
    ReceiveData_t receive_data;          // 从MiniPC接收的数据
    uint32_t rx_buff_len;                // 接收数据长度
    volatile uint8_t MiniPC_Online_Flag; // MiniPC在线标志位
    volatile uint32_t MiniPC_Last_rx_ms; // MiniPC最后接收时间
    RMQueue_Handle MIniPC_queue;         // MiniPC数据队列
} MiniPC_instance_t;

/*marco -----------------------------------------------------------------------*/
void MiniPC_Init(void);
void MiniPC_AliveCheck(void);
void DecodeMiniPC(uint8_t *Buf, uint32_t *Len);
void MiniPC_Send(uint8_t *Buf, uint16_t Len);
void Usb_Process(void);
/*extern------------------------------------------------------------------------*/
extern MiniPC_instance_t MiniPC_instance;
#endif // CAN_COMM_H
