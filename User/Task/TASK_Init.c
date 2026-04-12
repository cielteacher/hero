#include "TASK_INIT.h"
#include "string.h"
#include "remote.h"
#include "Gimbal.h"
#include "Shoot.h"
#include "Gimbal_Cmd.h"
TaskHandle_t Task_Alive_Handle;
TaskHandle_t Task_Minipc_Handle;
TaskHandle_t Task_Usb_Handle;
TaskHandle_t Task_Ins_Handle;
TaskHandle_t Task_Can_Handle;
TaskHandle_t Task_Usart_Handle;
TaskHandle_t Task_Robot_Control_Handle;
void Task_init(void) 
{
  taskENTER_CRITICAL(); // 进入临界区
  xTaskCreate((TaskFunction_t)Task_Usb, "Task_Remote", 128, NULL, 8, &Task_Usb_Handle);
  xTaskCreate((TaskFunction_t)Task_Ins, "Task_Ins", 512, NULL, 9, &Task_Ins_Handle);
  xTaskCreate((TaskFunction_t)Task_Can, "Task_Can", 128, NULL, 8, &Task_Can_Handle);
  xTaskCreate((TaskFunction_t)Task_Usart, "Task_Usart", 512, NULL, 8, &Task_Usart_Handle);
  xTaskCreate((TaskFunction_t)Task_Alive, "Task_Alive", 128, NULL, 8, &Task_Alive_Handle);
	 xTaskCreate((TaskFunction_t)Task_Minipc, "Task_Minipc", 128, NULL, 8, &Task_Minipc_Handle);
  xTaskCreate((TaskFunction_t)Task_Robot_Control, "Task_Robot_Control", 512, NULL, 9, &Task_Robot_Control_Handle);
  taskEXIT_CRITICAL(); // 退出临界区
  /*  初始化  */
	DR16_Init(); // 遥控器初始化
	Robot_Init();
	Can_Init();    // can初始化
	BMI088_Init(); // 陀螺仪初始化
  vTaskDelete(NULL);   // 删除开始空闲任务
}
