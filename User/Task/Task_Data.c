#include "Task_Data.h"
#include "DJIMOTOR.h"
#include "DMmotor.h"
#include "Kalman_Filter.h"
#include "PID.h"
#include "remote.h"
#include "MiniPC.h"
#include "can_comm.h"
#include "bmi088driver.h"
#include "quaternion.h"
#include "LPF.h"
#include "bsp_pwm.h"
#include "Quaternion.h"
#include "bsp_dwt.h"

//目前数据更新任务全部加着临界区
void Task_Alive(void *pvParameters)
{
  for (;;)
  {//
    FDCAN_Restart();
    DM_Motor_AliveCheck();
    DJI_Motor_AliveCheck();
    BMI088_AliveCheck();
    INS_AliveCheck();
    DR16_AliveCheck();
    MiniPC_AliveCheck();
    Can_Comm_AliveCheck();
    vTaskDelay(1);
  }
}


void   Task_Minipc(void *pvParameters)
{    
	for(;;){
				SendData_t INS_Data = {
        .frame_header.id = SEND_IMU_DATA_ID,
        .frame_header.len = sizeof(SendData_t),
        .frame_header.sof = SEND_SOF,

        .data = {
            .self_color = 0, // robot_color
            .bullet_speed = 0, // 子弹速度（如果有的话
            .yaw = INS_Info.Yaw_Angle ,   
            .pitch = INS_Info.Pitch_Angle ,        
            .roll = INS_Info.Roll_Angle ,

            .yaw_vel = INS_Info.Yaw_Gyro ,   // rad/s
            .pitch_vel = INS_Info.Pitch_Gyro , // rad/s
            .roll_vel = INS_Info.Roll_Gyro ,  // rad/s
        }};
			INS_Data.data.bullet_speed = 0; // 子弹速度（如果有的话
        MiniPC_Send((uint8_t*)&INS_Data,sizeof(INS_Data));
					vTaskDelay(1);
	}
	

}
  
void  Task_Ins(void *pvParameters) {
  TickType_t Task_INS_SysTick = 0;
	Task_INS_SysTick = xTaskGetTickCount();
/**
 * @brief the Initialize data of state transition matrix.
 */
	
  static float QuaternionEKF_A_Data[36] = {1, 0, 0, 0, 0, 0,
                                         0, 1, 0, 0, 0, 0,
                                         0, 0, 1, 0, 0, 0,
                                         0, 0, 0, 1, 0, 0,
                                         0, 0, 0, 0, 1, 0,
                                         0, 0, 0, 0, 0, 1};
static float INS_LPF2p_Alpha[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
LowPassFilter2p_Info_TypeDef INS_AccelPF2p[3];
/**
 * @brief the Initialize data of posteriori covariance matrix.
 */
static float QuaternionEKF_P_Data[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                         0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                         0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                         0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                         0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                         0.1, 0.1, 0.1, 0.1, 0.1, 100};

  PID IMU_temperature_control_pid = {.Kp = 10.0f,
                                     .Ki = 5.0f,
                                     .Kd = 0.0f,
                                     .interlimit = 3000,
                                     .outlimit = 10000,
                                     .DeadBand = 0.125f,
                                     .inter_threLow = 500,
                                     .inter_threUp = 1000};
  LowPassFilter2p_Init(&INS_AccelPF2p[0], INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[1], INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[2], INS_LPF2p_Alpha);
  QuaternionEKF_Init(&Quaternion_Info, 10.f, 0.001f, 1000000.f, QuaternionEKF_A_Data, QuaternionEKF_P_Data);

  // 主循环
  for (;;) 
	{		



		/* Update the BMI088 measurement */
    BMI088_Info_Update(&BMI088_Info);

    if (BMI088_Info.imu_online_flag)
    {
      INS_Info.INS_last_rx_time_ms = DWT_GetTime_ms();
    }

    /* Accel measurement LPF2p */
    INS_Info.Accel[0]   =   LowPassFilter2p_Update(&INS_AccelPF2p[0],BMI088_Info.Accel[0]);
    INS_Info.Accel[1]   =   LowPassFilter2p_Update(&INS_AccelPF2p[1],BMI088_Info.Accel[1]);
    INS_Info.Accel[2]   =   LowPassFilter2p_Update(&INS_AccelPF2p[2],BMI088_Info.Accel[2]);
		
    /* Update the INS gyro in radians */
		INS_Info.Gyro[0]   =   BMI088_Info.Gyro[0];
    INS_Info.Gyro[1]   =   BMI088_Info.Gyro[1];
    INS_Info.Gyro[2]   =   BMI088_Info.Gyro[2];
		
		/* Update the QuaternionEKF */
    QuaternionEKF_Update(&Quaternion_Info,INS_Info.Gyro,INS_Info.Accel,0.001f);
		
    memcpy(INS_Info.Angle,Quaternion_Info.EulerAngle,sizeof(INS_Info.Angle));

		/* Update the Euler angle in degrees. */
    INS_Info.Pitch_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH]*57.295779513f;
    INS_Info.Yaw_Angle   = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]   *57.295779513f;
    INS_Info.Roll_Angle  = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL]*57.295779513f;
		
		/* Update the yaw total angle */
		if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.f)
		{
			INS_Info.YawRoundCount++;
		}
		else if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.f)
		{
			INS_Info.YawRoundCount--;
		}
		INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;
		
		INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount*360.f;
		
    /* Update the INS gyro in degrees */
    INS_Info.Pitch_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH]*RadiansToDegrees;
    INS_Info.Yaw_Gyro   = INS_Info.Gyro[IMU_GYRO_INDEX_YAW]*RadiansToDegrees;
    INS_Info.Roll_Gyro  = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]*RadiansToDegrees;
    /* Update the yaw total angle */
    if (INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.f) {
      INS_Info.YawRoundCount++;
    } else if (INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.f) {
      INS_Info.YawRoundCount--;
    }
    INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;

    INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount * 360.f;

    /* Update the INS gyro in degrees */
    INS_Info.Pitch_Gyro =
        INS_Info.Gyro[IMU_GYRO_INDEX_PITCH] * RadiansToDegrees;
    INS_Info.Yaw_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_YAW] * RadiansToDegrees;
    INS_Info.Roll_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL] * RadiansToDegrees;

    // 固定周期延时
	vTaskDelayUntil(&Task_INS_SysTick,1);
  }
}
void Task_Can(void *pvParameters)
{
  for (;;)
  {
    // 阻塞等待通知
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    CAN_Process();
		vTaskDelay(1);
  }
}
void Task_Usart(void *pvParameters)
{
  for (;;)
  {
    // 阻塞等待通知
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    USART_Process();
		vTaskDelay(1);
  }
}
void Task_Usb(void *pvParameters) {
  // 主循环
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Usb_Process();
		vTaskDelay(1);
  }
}
