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
/* Data Structure -----------------------------------------------------------*/
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


