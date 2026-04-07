#ifndef BSP_UART_H
#define BSP_UART_H
#include "RMQueue.h"
#include "usart.h"
#include "TASK_Init.h"
#define DEVICE_USART_CNT 1 // 串口数量
#define USART_PORT_CNT (DEVICE_USART_CNT * 3) // 消息端口数量 

/* 发送模式枚举 */
typedef enum {
  UASRT_TRANSFER_NONE = 0,
  USART_TRANSFER_BLOCKING,
  USART_TRANSFER_IT,
  USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;
/* 接收模式枚举 */
typedef enum {
  UASRT_RECEIVE_NONE = 0,
  USART_RECEIVE_BLOCKING,
  USART_RECEIVE_IT,
  USART_RECEIVE_DMA,
  USART_RECEIVE_IDLE_DMA,
} USART_RECEIVE_MODE;

/* uasrt 初始化配置结构体 */
typedef struct {
  USART_TRANSFER_MODE transmit_mode; // 发送模式
  USART_RECEIVE_MODE receive_mode;   // 接收模式
  UART_HandleTypeDef *usart_handle;  // 实例对应的uart_handle
  uint16_t data_len;                 // 单包数据长度
  uint8_t temp_depth;                // 数据深度
} USART_Init_Config_s;

// 串口实例结构体,每个module都要包含一个实例.
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况,因此不用加入id;当然也可以选择加入,这样在bsp层可以访问到module的其他信息
typedef struct _UARTnstance {
  RMQueue_Handle rx_queue;           // 循环队列存放数据
  USART_TRANSFER_MODE transmit_mode; // 发送模式
  USART_RECEIVE_MODE receive_mode;   // 接收模式
  UART_HandleTypeDef *usart_handle;  // 实例对应的uart_handle
  uint8_t temp_depth;
  void (*uart_module_callback)(struct _UARTnstance *); // 解析函数
  void (*uart_error_callback)(struct _UARTnstance *);  // 错误回调函数
  uint16_t data_len;                                   // 单包数据长度
  volatile uint16_t buffer_len;                        // 接收包长度（主要用于不定长数据接收）
  void *id;                                            // 通用用户数据指针，可存放任意类型数据
} USARTInstance;
// uart端口
typedef struct
{
  USARTInstance *instance; // 对应的串口实例
  uint16_t length;         // 接收长度
} USART_Port;
/**
 * @brief  注册一个串口实例,返回串口实例
 * @param _config 传入串口初始化结构体
 **/
USARTInstance *USARTRegister(const USART_Init_Config_s *_config);

RM_Status USART_Transmit(USARTInstance *instance, uint8_t *send_buff, uint16_t data_len);
RM_Status USART_Receive(USARTInstance *instance, uint8_t *recv_buff, uint16_t data_len);
void USART_Process(void);
#endif
