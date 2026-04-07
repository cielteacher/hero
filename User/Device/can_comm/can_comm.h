#ifndef CAN_COMM_H
#define CAN_COMM_H
/*	include	*/
#include "bsp_fdcan.h"
#include "RMLibHead.h"
#include "robot.h"
#include <stdint.h>
/* Define----------------------------------------------------------------------*/
#define CAN_COMM_MAX_BUFFSIZE 60 // 最大发送/接收字节数,如果不够可以增加此数值
#define CAN_COMM_HEADER 's'      // 帧头
#define CAN_COMM_TAIL 'e'        // 帧尾
#define CAN_COMM_OFFSET_BYTES 3		// 's'+ datalen(8) + 'e'



/***  板间通信定义  ***/
// 云台->底盘数据包
#pragma pack(1) // 压缩结构体,取消字节对齐
typedef struct  {
    int16_t vx;            	// 单位 基准速度的倍率（基准速度由底盘模块根据功率自动计算）
    int16_t vy;            	// 单位 基准速度的倍率
    int16_t rotate;        	// 单位 旋转速度度每秒
	  uint8_t Close_flag;	   	//!< @brief 底盘关闭标志位
    uint8_t Unlimit_flag;	   	//!< @brief 底盘突破功率限制加速
} Gimbal_board_send_t;
typedef struct{
  uint8_t Gimbal_Online;         
  uint8_t Shoot_Online ;          
  uint8_t Move_Status ;     
  uint8_t Vision_Online;          
  uint8_t Shoot_Mode;				           
	uint8_t Break_Limitation; 
	uint16_t Reserve; 
}Gimbal_action_t;
// 云台<-底盘数据包
typedef struct {
  int16_t chassis_gyro;  // 将底盘主控的imu数据发到云台
  int16_t chassis_slop;  // 底盘所在斜坡倾角
  uint8_t game_progress; // 比赛状态
uint16_t heat_limit_remain;   // 剩余热量
uint8_t shoot_barrel_cooling; // 枪口热量冷却值
} Chassis_board_send_t;
#pragma pack()
typedef struct {
  CANInstance *can_comm;                // can实例
  Chassis_board_send_t can_comm_rx_data; // 上次接收的数据
  uint32_t can_comm_last_rx_ms;         // 上次接收数据的时间戳
  uint8_t can_comm_online_flag;
} Can_Comm_Typedef;

void Can_Comm_Init();
void Can_Comm_AliveCheck(void);

extern Can_Comm_Typedef can_comm_instance;
#endif // CAN_COMM_H


