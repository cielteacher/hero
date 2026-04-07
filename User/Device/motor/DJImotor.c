#include "DJIMOTOR.h"
#include "string.h"
#include <stdlib.h>

// 全局索引
static uint8_t DJI_idx = 0;
// 静态创建电机实例
DJI_Motor_Instance dji_motor_instance[DJI_MOTOR_CNT];
/**
 * @brief  更新大疆（DJI）电机信息
 * @param  _instance: CAN实例指针
 */
/*被中断调用*/
static void DecodeDJIMotor(CANInstance *_instance)
{
  DJI_Motor_Instance *motor = (DJI_Motor_Instance *)_instance->id; // 注册时已将电机实例存入
  uint8_t *rx_buff = _instance->rx_buff;                           // 接收数据缓冲区
  motor->djimotor_last_rx_ms = DWT_GetTime_ms();                   
  motor->dji_motor_online_flag = 1;
  // 数据移位
  motor->Data.Last_MechanicalAngle = motor->Data.MechanicalAngle;
  motor->Data.Temperature = rx_buff[6];
  motor->Data.MechanicalAngle = ((int16_t)rx_buff[0] << 8 | (int16_t)rx_buff[1]);
  motor->Data.Velocity = ((int16_t)rx_buff[2] << 8 | (int16_t)rx_buff[3]);
  motor->Data.Current = ((int16_t)rx_buff[4] << 8 | (int16_t)rx_buff[5]);
  // 数据处理
  int16_t diff = motor->Data.MechanicalAngle - motor->Data.Last_MechanicalAngle;
  if (diff > 4000)
    motor->Data.round--;
  if (diff < -4000)
    motor->Data.round++;
  // 计算连续机械角度
  motor->Data.Continuous_Mechanical_angle =
      motor->Data.round * 8192 + motor->Data.MechanicalAngle;

  // 一阶低通滤波（平滑数据）
  motor->Data.SpeedFilter =
      (1.0f - DJI_SPEED_SMOOTH_COEF) * motor->Data.SpeedFilter +
      (float)(DJI_SPEED_SMOOTH_COEF * motor->Data.Velocity);

  motor->Data.CurrentFilter =
      (1.0f - DJI_CURRENT_SMOOTH_COEF) * motor->Data.CurrentFilter +
      DJI_CURRENT_SMOOTH_COEF * (float)(motor->Data.Current);

  // 角度转换（转成°）
  motor->Data.Angle = motor->Data.MechanicalAngle * ECD_ANGLE_COEF;
  motor->Data.Angle_DEG = motor->Data.Continuous_Mechanical_angle * ECD_ANGLE_COEF;

  if (motor->Data.Current > 30000 || motor->Data.Current < -30000)
    motor->dji_motor_online_flag = 0;
    //过流强制掉线
}

/**
 * @brief  停止电机
 * @param  motor: 电机实例指针
 */
void DJIMotorStop(DJI_Motor_Instance *motor)
{
  uint8_t Data[8] = {0};
  memset(Data, 0, sizeof(Data));
  CAN_Send(motor->CANFrame, Data, 10);
}



/**
 * @brief  电机初始化（注册一个电机实例）
 * @param  _config: 电机配置结构体
 * @retval 电机实例指针
 * @attention
 * 必须在freertos开始前全部初始化完成！！或者加临界区（好像也不是很行）
 */
DJI_Motor_Instance *DJI_Motor_Init(DJI_Motor_Config *_config)
{
  if (DJI_idx >= DJI_MOTOR_CNT)
    return NULL;
  if (!_config)
    return NULL;
  dji_motor_instance[DJI_idx].djimotor_last_rx_ms = DWT_GetTime_ms();
  dji_motor_instance[DJI_idx].dji_motor_online_flag = 0; // 初始化停止
  dji_motor_instance[DJI_idx].Type = _config->Type;
  dji_motor_instance[DJI_idx].CANFrame = CANRegister(&(_config->Can_Config));
  /*传入can实例的持有者*/
  dji_motor_instance[DJI_idx].CANFrame->id = &dji_motor_instance[DJI_idx];
  dji_motor_instance[DJI_idx].CANFrame->can_module_callback = DecodeDJIMotor;

  return &dji_motor_instance[DJI_idx++];
}

/**
 * @brief  大疆电机发送函数
 * @param  DJI_Motor: 电机实例指针
 * @param  txbuffer: 发送缓冲区
 */
void DJI_Motor_CAN_TxMessage(DJI_Motor_Instance *DJI_Motor, int16_t *txbuffer)
{
  uint8_t Data[8] = {0};

  if (DJI_Motor->dji_motor_online_flag == 1)
  {
    Data[0] = txbuffer[0] >> 8;
    Data[1] = txbuffer[0];
    Data[2] = txbuffer[1] >> 8;
    Data[3] = txbuffer[1];
    Data[4] = txbuffer[2] >> 8;
    Data[5] = txbuffer[2];
    Data[6] = txbuffer[3] >> 8;
    Data[7] = txbuffer[3];

    CAN_Send(DJI_Motor->CANFrame, Data, 10);
  }
  else
    DJIMotorStop(DJI_Motor);
}
void DJI_Motor_AliveCheck(void)
{

  for (uint8_t i = 0; i < DJI_idx; i++)
  {
    uint32_t lost_time = DWT_GetTime_ms() - dji_motor_instance[i].djimotor_last_rx_ms;
    if (lost_time > 50)
    {
      dji_motor_instance[i].dji_motor_online_flag = 0;

    }
    else
      dji_motor_instance[i].dji_motor_online_flag = 1;
  }
}
