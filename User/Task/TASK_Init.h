//fdcan
// 2.28，更新
// 将H7频率改为480MHZ，发现中断然后任务通知再更新数据有点臃肿，借鉴香港科技大学的思路修改
// 改成定时访问FIFO
// 轮询有点不太优雅，但是反正现在也没有什么性能要求（而且电机大都都是1ms？）
// 3.4，更新,难绷，想起来还有达妙电机。最近学了freertos任务通知的函数，改为中断+任务通知
// 幸好之前备份了...
// 3.5 嘻嘻，单电机调试好了

// char buf[64];
// sprintf(buf, "Check: RX 0x%lX  want 0x%lX\r\n",
//         id,
//         inst->rx_id);

// CDC_Transmit_HS((uint8_t *)buf, strlen(buf));
#ifndef __TASK_INIT_H
#define __TASK_INIT_H

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"


/**
* @brief 看门狗任务
*/
void Task_Alive(void *pvParameters);
/**
* @brief USB任务
*/
void    Task_Usb(void *pvParameters);
/**
* @brief 陀螺仪任务
*/
void   Task_Ins(void *pvParameters);
/**
 * @brief CAN处理任务
 */
void  Task_Can(void *pvParameters);
/**
 * @brief USART处理任务
 */
void  Task_Usart(void *pvParameters);
/**
 * @brief USART处理任务
 */
void    Task_Minipc(void *pvParameters);
/**
	*@brief 整车控制任务
	*/
void    Task_Robot_Control(void *pvParameters);


extern TaskHandle_t Task_Robot_Control_Handle;
extern TaskHandle_t Task_Alive_Handle;
extern TaskHandle_t Task_Gimbal_Handle;
extern TaskHandle_t Task_Ins_Handle;
extern TaskHandle_t Task_Can_Handle;
extern TaskHandle_t Task_Minipc_Handle;
extern TaskHandle_t Task_Usart_Handle;
extern TaskHandle_t Task_Usb_Handle;
#endif
