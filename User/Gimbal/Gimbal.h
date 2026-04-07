#ifndef __GIMBAL_H
#define __GIMBAL_H
/*INCLUDE*/
#include "stdbool.h"
#include "RMLibHead.h"
#include "DJIMOTOR.h"
#include "robot.h"
#include "DMmotor.h"
#include "pid.h"
#include "MiniPC.h"
#include "BMI088driver.h"
#include "remote.h"

// 云台参数
// 控制周期 1ms
#define GIMBAL_CONTROL_PERIOD 1

// Pitch轴限位（机械角度）
#define PITCH_ANGLE_MAX 30.0f   // Pitch轴最大角度（向上）
#define PITCH_ANGLE_MIN -25.0f  // Pitch轴最小角度（向下）

// 云台遥控器灵敏度
#define GIMBAL_RC_YAW_SENSITIVITY   0.005f    // 遥控器Yaw灵敏度
#define GIMBAL_RC_PITCH_SENSITIVITY 0.003f    // 遥控器Pitch灵敏度

// 鼠标灵敏度
#define GIMBAL_MOUSE_YAW_SENSITIVITY   0.0015f   // 鼠标Yaw灵敏度
#define GIMBAL_MOUSE_PITCH_SENSITIVITY 0.001f    // 鼠标Pitch灵敏度

// 云台状态结构体
typedef struct
{
    DM_Motor_Instance *pitch_motor;   // Pitch轴电机实例
    DM_Motor_Instance *yaw_motor;     // Yaw轴电机实例
 
} Gimbal_t;

// 函数声明
void Gimbal_Init(void);
void Gimbal_Control(void);

extern Gimbal_t gimbal; // 云台状态实例

#endif /* __GIMBAL_H */