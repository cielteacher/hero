#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H
#include "stm32h7xx.h"
#include "bsp_dwt.h"
#include "fdcan.h"
#include "RMlibHead.h"
#define CAN_REGISTER_CNT 9 //can线上的设备数
#define CAN_PORT_CNT (CAN_REGISTER_CNT * 3) // 消息端口数量
//CAN实例结构体
typedef struct CANInstance
{
    FDCAN_HandleTypeDef *can_handle;        // can句柄
    FDCAN_TxHeaderTypeDef header_type_def;         // CAN报文发送配置
    uint32_t tx_id;                         // 发送id
    uint32_t FIFO_idx;                    // CAN消息填入的FIFO号
    uint8_t *rx_buff;                     // 接收缓存,最大消息长度为8
    uint32_t rx_id;                         // 接收id
    uint8_t rx_len;                         // 接收长度,可能为0-8
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct CANInstance *); // 回调函数指针,指向使用该CAN实例的模块的更新函数,在接收到数据后调用
    void *id; // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;
//消息端口
typedef struct
{
    FDCAN_HandleTypeDef *handle;
    uint32_t fifo;
    uint32_t id;
    uint8_t data[8]; // 接收数据缓冲区
} CAN_Port;
//CAN传入设置
typedef struct
{
    FDCAN_HandleTypeDef *can_handle;            // can句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    uint8_t rx_len;                         // 接收长度,可能为0-8
} CAN_Init_Config_s;
/**
 * @brief  CAN注册函数
 * @param  config: CAN初始化配置
 * @retval CAN实例指针
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

HAL_StatusTypeDef CAN_Send(CANInstance *_instance,  uint8_t *tx_buff,float timeout);

/**
 * @brief  CAN初始化，需要在全部注册完成后调用
 */
void Can_Init(void);
/**
 * @brief  CAN消息处理函数，需要在主循环中调用
 */
void CAN_Process(void);
/**
    * @brief  CAN重启函数，重启后会重新进入在线状态
    * @param  hfdcan: 需要重启的CAN句柄
    */

void FDCAN_Restart(void);
#endif //SINCERELY_BSP_FDCAN_H

