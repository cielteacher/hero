#include "Task_Gimbal.h"
#include "Gimbal.h"
#include "PID.h"
#include "RMLibHead.h"
#include "Gimbal_Cmd.h"
#include "Shoot.h"
#include "pid.h"

void Task_Robot_Control(void *pvParameters)
{
	TickType_t Task_Robot_Control_SysTick = 0;
	Task_Robot_Control_SysTick = xTaskGetTickCount();
	int16_t can_send[4] = {0};
	can_send[1] = 500;
	for(;;)
	{
		Robot_Update();
		// can_send[1] = PID_Control(shoot.Pluck_motor->Data.CurrentFilter, 50.0f, &test);
		DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, can_send);
		// Shoot_Control();
		// Gimbal_Control();
    	vTaskDelayUntil(&Task_Robot_Control_SysTick, 1);
	}
}



