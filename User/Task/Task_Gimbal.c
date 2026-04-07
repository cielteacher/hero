#include "Task_Gimbal.h"
#include "RMLibHead.h"
#include "Gimbal_Cmd.h"
void Task_Robot_Control(void *pvParameters)
{

	for(;;)
	{
		Robot_Update();
		Shoot_Control();
		Gimbal_Control();
    vTaskDelay(1);
	}
}



