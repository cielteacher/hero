#ifndef SINCERELY_MOTOR_H
#define SINCERELY_MOTOR_H
/*	include	*/
#include "bsp_fdcan.h"
#include "RMLibHead.h"
#include <stdint.h>

/* Define----------------------------------------------------------------------*/
/*给电机的输出限幅*/
#define RM3508_LIMIT 16000  //!<@brief RM3508的输出限幅
#define GM6020_LIMIT 29000  //!<@brief GM6020的输出限幅
#define RM3510_LIMIT 32767  //!<@brief RM3510的输出限幅
#define GM3510_LIMIT 29000  //!<@brief GM3510的输出限幅
#define M2006_LIMIT  10000  //!<@brief M2006 的输出限幅
/*单位转换*/
#define RAD_TO_DEGREE 57.2957795f		// 弧度~角度
#define DEGREE_TO_RAD 0.01745329252f	// 角度~弧度
#define RPM_TO_ANGLE_PER_SEC 6.0f		// 转速转化 圈数每分钟 ~ 角度每秒
#define RPM_TO_RAD_PER_SEC 0.104719755f //  转速转化 圈数每分钟 ~ 弧度每秒
#define ECD_ANGLE_COEF 0.0439453125f	// (360/8192),将编码器值转化为角度制
/*编码器值*/
#define MAX_EC_VALUE 8192
/*一节低通滤波参数*/
/* 滤波系数设置为1的时候即关闭滤波 */
// TODO 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
#define DJI_SPEED_SMOOTH_COEF 0.90f	  // 最好大于0.85
#define DJI_CURRENT_SMOOTH_COEF 0.95f // 必须大于0.9
#define DM_SPEED_SMOOTH_COEF 0.90f
#define DM_CURRENT_SMOOTH_COEF 0.95f
/*电机状态枚举*/
typedef enum
{
	MOTOR_STOP = 0,
	MOTOR_ENABLE = 1, // 修正拼写错误：MOTOR_ENALBE -> MOTOR_ENABLE
	MOTOR_DATALOST = 2,
} Motor_Working_Type;

/*控制方式*/
typedef enum
{
	/*大疆*/
	NONE_LOOP = 0,	  // 开环
	CURRENT_LOOP = 1, // 电流环
	SPEED_LOOP = 2,	  // 速度环
	ANGLE_LOOP = 4,	  // 位置环
} DJIMOTOR_CONTROL_TYPE;

typedef enum
{
	/*达妙*/
	DJI_MODEL = 0,		   // 一拖四
	MIT_MODEL = 1,		   // MIT
	SPEED_ANGLE_MODEL = 2, // 速位环
	SPEED_MODEL = 4,	   // 位置环
	EMIT_MODEL = 8,		   // EMIT
} DMMOTOR_CONTROL_TYPE;

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int X_int, float X_min, float X_max, int Bits);
int16_t move_nearby_int16(uint16_t target_angle, uint16_t angle_now);
float move_nearby_float(float target_angle, float angle_now);
#endif // DEVICE_MOTOR_H


