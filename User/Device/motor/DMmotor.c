#include "DMMotor.h"
#include "bsp_fdcan.h"
#include "string.h"
// 全局检索
static uint8_t DM_idx = 0;
// 静态创建电机实例
DM_Motor_Instance dm_motor_instance[DM_MOTOR_CNT];

/**
 * @brief  Update the DM_Motor Information
 */
/*被中断调用*/
static void DecodeDMMotor(CANInstance *_instance)
{
	DM_Motor_Instance *DM_Motor = (DM_Motor_Instance *)_instance->id;
	uint8_t *Rx_Buf = _instance->rx_buff;
	DM_Motor->dmmotor_last_rx_time_ms = DWT_GetTime_ms(); // 存活标志清零
	DM_Motor->dm_motor_online_flag = 1;
	/*大疆模式*/
	if (DM_Motor->Control_Mode == DJI_MODEL)
	{// 数据移位
		DM_Motor->Data.DJI_data.Last_MechanicalAngle = DM_Motor->Data.DJI_data.MechanicalAngle;
		DM_Motor->Data.DJI_data.MechanicalAngle = ((uint16_t)Rx_Buf[0]) << 8 | Rx_Buf[1];
		DM_Motor->Data.DJI_data.Velocity = (int16_t)(Rx_Buf[2] << 8 | Rx_Buf[3]) * 0.01f;
		DM_Motor->Data.DJI_data.Current = (int16_t)(Rx_Buf[4] << 8 | Rx_Buf[5]) / 1000.0f;
		DM_Motor->Data.DJI_data.Temperature = (int8_t)(Rx_Buf[6]);
		//数据处理
		// 跨圈处理（round增减）
		int16_t diff = DM_Motor->Data.DJI_data.MechanicalAngle - DM_Motor->Data.DJI_data.Last_MechanicalAngle;
		if (diff > 4000)
			DM_Motor->Data.DJI_data.round--;
		if (diff < -4000)
			DM_Motor->Data.DJI_data.round++;
		// 计算连续机械角度
		DM_Motor->Data.DJI_data.Continuous_Mechanical_angle = DM_Motor->Data.DJI_data.round * 8192 + DM_Motor->Data.DJI_data.MechanicalAngle;

		// 一阶低通滤波
		DM_Motor->Data.DJI_data.SpeedFilter = (1.0f - DJI_SPEED_SMOOTH_COEF) * DM_Motor->Data.DJI_data.SpeedFilter + (float)(DJI_SPEED_SMOOTH_COEF * DM_Motor->Data.DJI_data.Velocity);

		DM_Motor->Data.DJI_data.CurrentFilter = (1.0f - DJI_CURRENT_SMOOTH_COEF) * DM_Motor->Data.DJI_data.CurrentFilter +
											 DJI_CURRENT_SMOOTH_COEF * (float)(DM_Motor->Data.DJI_data.Current);

		// 角度转换（转成°）
		DM_Motor->Data.DJI_data.Angle = DM_Motor->Data.DJI_data.MechanicalAngle * ECD_ANGLE_COEF;
        DM_Motor->Data.DJI_data.Angle_DEG = DM_Motor->Data.DJI_data.Continuous_Mechanical_angle *
                    ECD_ANGLE_COEF;
                if (DM_Motor->Data.DJI_data.Current > 30000 || DM_Motor->Data.DJI_data.Current < -30000)
			DM_Motor->dm_motor_online_flag = 0;
	} /*非大疆模式*/
	if (DM_Motor->Control_Mode != DJI_MODEL) // 其他模式的反馈帧格式一样，在这里统一处理
	{
		DM_Motor->Data.Normal_Data.id = (Rx_Buf[0]) & 0x0F; // 电机id
		DM_Motor->Data.Normal_Data.State = (Rx_Buf[0]) >> 4;
		DM_Motor->Data.Normal_Data.P_int = (Rx_Buf[1] << 8) | Rx_Buf[2];						// 位置（int）rad
		DM_Motor->Data.Normal_Data.V_int = (Rx_Buf[3] << 4) | (Rx_Buf[4] >> 4);					// 速度rad/s
		DM_Motor->Data.Normal_Data.T_int = ((Rx_Buf[4] & 0xF) << 8) | Rx_Buf[5];				// 扭矩Nm
		DM_Motor->Data.Normal_Data.Temperature_MOS = (float)(Rx_Buf[6]);					 // mos管温度
		DM_Motor->Data.Normal_Data.Temperature_Rotor = (float)(Rx_Buf[7]);			 // 电机绕组温度
		DM_Motor->Data.Normal_Data.Position = uint_to_float( DM_Motor->Data.Normal_Data.P_int, -DM_Motor->Param_Range.P_MAX, DM_Motor->Param_Range.P_MAX, 16); // (-12.5,12.5)
		DM_Motor->Data.Normal_Data.Velocity = uint_to_float( DM_Motor->Data.Normal_Data.V_int, -DM_Motor->Param_Range.V_MAX, DM_Motor->Param_Range.V_MAX, 12); // (-45.0,45.0)
		DM_Motor->Data.Normal_Data.Torque = uint_to_float( DM_Motor->Data.Normal_Data.T_int, -DM_Motor->Param_Range.T_MAX, DM_Motor->Param_Range.T_MAX, 12);	  // (-18.0,18.0)
	}
}

/**
************************************************************************
* @brief:      	 启用电机模式函数
* @param[in]:   instance: 电机实例指针
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
void DMMotorEnable(DM_Motor_Instance *instance)
{
	if (instance->Control_Mode != DJI_MODEL)
	{
		uint8_t data[8];
		data[0] = 0xFF;
		data[1] = 0xFF;
		data[2] = 0xFF;
		data[3] = 0xFF;
		data[4] = 0xFF;
		data[5] = 0xFF;
		data[6] = 0xFF;
		data[7] = 0xFC;
		CAN_Send(instance->CAN_instance, data, 10) ;
	}
	
	else
	{
	}
}

/**
************************************************************************
* @brief:      	 停止电机模式函数
* @param[in]:   instance: 电机实例指针
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
void DMMotordisable(DM_Motor_Instance *instance)
{
	uint8_t data[8] = {0,0,0,0,0,0,0,0};
	CAN_Send(instance->CAN_instance, data, 10);
	//这里是发失能帧还是发全为0的数据?
}

/**
************************************************************************
* @brief:      	 		清除电机错误代码
* @param[in]:   instance: 电机实例指针
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
void DMMotorClear(DM_Motor_Instance *instance)
{
	uint8_t data[8];

	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;

	CAN_Send(instance->CAN_instance, data, 10);
}

/**
 * @brief  电机初始化（注册一个电机实例）
 * @param  _config: 电机配置结构体
 * @retval 电机实例指针
 * @attention 必须在freertos开始前全部初始化完成！！或者加临界区（好像也不是很行）
 */
DM_Motor_Instance *DM_Motor_Init(DM_Motor_Info_Typedef *_config)
{
	if (DM_idx >= DM_MOTOR_CNT) return NULL;
	if (!_config) return NULL;  
	
	dm_motor_instance[DM_idx].type = _config->type;
	dm_motor_instance[DM_idx].dmmotor_last_rx_time_ms = DWT_GetTime_ms();
	dm_motor_instance[DM_idx].Control_Mode = _config->Control_Mode;
	dm_motor_instance[DM_idx].CAN_instance = CANRegister(&(_config->CANFrame));
		/*传入can实例的持有者*/
	dm_motor_instance[DM_idx].CAN_instance->id = &dm_motor_instance[DM_idx];
	dm_motor_instance[DM_idx].CAN_instance->can_module_callback = DecodeDMMotor;
	dm_motor_instance[DM_idx].dm_motor_online_flag = 0;
	return &dm_motor_instance[DM_idx++];
}

/**
 * @brief  发送电机指令
 * @attention 电机发送指令，mit模式
 */
void DM_Motor_MIT_CAN_TxMessage(DM_Motor_Instance *DM_Motor, float pos, float vel, float kp, float kd, float tor)
{
	if (DM_Motor->Control_Mode != MIT_MODEL)
		return;
	uint8_t data[8];
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	// 转换类型
	pos_tmp = float_to_uint(pos, -12.5, 12.5, 16);
	vel_tmp = float_to_uint(vel, -12.5, 12.5, 12);
	tor_tmp = float_to_uint(tor, -12.5, 12.5, 12);
	kp_tmp = float_to_uint(kp, -50, 50, 12);
	kd_tmp = float_to_uint(kd, -50, 50, 12);
	if (DM_Motor->dm_motor_online_flag == 1)
	{
		// 数据移位
		data[0] = (pos_tmp >> 8);
		data[1] = pos_tmp;
		data[2] = (vel_tmp >> 4);
		data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
		data[4] = kp_tmp;
		data[5] = (kd_tmp >> 4);
		data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
		data[7] = tor_tmp;
		CAN_Send(DM_Motor->CAN_instance, data, 1);
	}	else
		DMMotordisable(DM_Motor);
}

/**
 * @brief  发送电机指令
 * @attention 电机发送指令，大疆模式
 */
void DM_Motor_DJI_CAN_TxMessage(DM_Motor_Instance *DM_Motor, int16_t *txbuffer)
{
	if (DM_Motor->Control_Mode != DJI_MODEL)
		return;
	uint8_t Data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	if (DM_Motor->dm_motor_online_flag == 1)
	{
		Data[0] = txbuffer[0] >> 8;
		Data[1] = txbuffer[0];
		Data[2] = txbuffer[1] >> 8;
		Data[3] = txbuffer[1];
		Data[4] = txbuffer[2] >> 8;
		Data[5] = txbuffer[2];
		Data[6] = txbuffer[3] >> 8;
		Data[7] = txbuffer[3];
		CAN_Send(DM_Motor->CAN_instance, Data, 10);
	}	else
		DMMotordisable(DM_Motor);
}
void DM_Motor_AliveCheck(void) {
	for (uint8_t i = 0; i < DM_idx; i++)
	{
		uint32_t lost_time = DWT_GetTime_ms() - dm_motor_instance[i].dmmotor_last_rx_time_ms;
		if (lost_time > 50)
		{
            dm_motor_instance[i].dm_motor_online_flag = 0;

		}
		else
			dm_motor_instance[i].dm_motor_online_flag = 1;
	}
}