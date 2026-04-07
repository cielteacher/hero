#include "can_comm.h"
#include <string.h>

Can_Comm_Typedef can_comm_instance = {0};
static void DecodeCancomm(CANInstance *_instance) 
{
  if(_instance == NULL || _instance->rx_buff == NULL)
    return;
  uint8_t *rx_buff = _instance->rx_buff;
  can_comm_instance.can_comm_last_rx_ms = DWT_GetTime_ms(); // 更新接收时间戳
  can_comm_instance.can_comm_online_flag = 1;               // 标记在线
  memcpy(&can_comm_instance.can_comm_rx_data, rx_buff, 8); // 假设数据长度为8字节
}
void Can_Comm_Init()
{
  CAN_Init_Config_s can_comm_config = {
      .can_handle = &hfdcan2,
      .tx_id = 0x110,
      .rx_id = 0x101,
      .rx_len = 8, // 接收长度为8字节
	};
  /* 注册CAN实例 */
  can_comm_instance.can_comm = CANRegister(&can_comm_config);
  
  can_comm_instance.can_comm_last_rx_ms = DWT_GetTime_ms();
  can_comm_instance.can_comm->id = &can_comm_instance; // 将实例指针传入CAN实例的id字段，供回调函数使用
  can_comm_instance.can_comm->can_module_callback = DecodeCancomm; // 设置CAN回调函数
}
void Can_Comm_AliveCheck(void)
{
  uint32_t now = DWT_GetTime_ms();
  if (now - can_comm_instance.can_comm_last_rx_ms > 500) // 超过500ms未收到数据
    can_comm_instance.can_comm_online_flag = 0;           // 标记离线
  else
    can_comm_instance.can_comm_online_flag = 1;           // 标记在线
}
