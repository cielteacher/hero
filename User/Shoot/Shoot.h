#ifndef __SHOOT_H
#define __SHOOT_H
/*INCLUDE*/
#include "stdbool.h"
#include "RMLibHead.h"
#include "DJIMOTOR.h"
#include "DMmotor.h"
#include "pid.h"
#include "robot.h"
#include "bsp_dwt.h"
#include "fdcan.h"

// 发射控制周期 1ms
#define SHOOT_CONTROL_PERIOD 1

// 发射机构结构体 - 仅包含电机数据
typedef struct
{
    DJI_Motor_Instance *Pluck_motor;   // 拨弹盘电机
    DJI_Motor_Instance *fric_motor_LF;  // 摩擦轮1（左前）
    DJI_Motor_Instance *fric_motor_RF;  // 摩擦轮2（右前）
    DJI_Motor_Instance *fric_motor_LB;  // 摩擦轮3（左后）
    DJI_Motor_Instance *fric_motor_RB;  // 摩擦轮4（右后）
    uint8_t pluck_lock;
    int32_t pluck_target_angle;        // 拨弹盘目标角度（连续角度）
} Shoot_t;

// 函数声明
void Shoot_Init(void);
void Shoot_Control(void);
uint8_t CheckFricSpeedReady(void);
extern Shoot_t shoot; // 发射机构状态实例

#endif /* __SHOOT_H */
