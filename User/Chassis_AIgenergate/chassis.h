#ifndef CHASSIS_H
#define CHASSIS_H

/*include*/
#include "DJIMOTOR.h"
#include "pid.h"

/* Defines */
#define CHASSIS_MOTOR_NUM 4
#define CHASSIS_POWER_LIMIT 200.0f  // 底盘功率限制 200W
#define CHASSIS_CURRENT_LIMIT 5000  // 底盘电流限制

// 底盘运动模式
typedef enum
{
    CHASSIS_RELAX = 0,        // 放松模式
    CHASSIS_FOLLOW_GIMBAL = 1, // 跟随云台
    CHASSIS_NORMAL = 2,        // 普通模式
    CHASSIS_SPIN = 3           // 旋转模式
} Chassis_Mode_Typedef;

// 底盘运动参数
typedef struct
{
    float vx;    // 前后速度
    float vy;    // 左右速度
    float wz;    // 旋转速度
    float power; // 实时功率
} Chassis_Motion_Typedef;

// 底盘力反馈数据
typedef struct
{
    float force[CHASSIS_MOTOR_NUM]; // 每个电机的力反馈
    float total_current;            // 总电流
} Chassis_Force_Feedback_Typedef;

// 底盘速度解算结果
typedef struct
{
    int16_t motor_speed[CHASSIS_MOTOR_NUM]; // 4个电机的速度目标值
} Chassis_Speed_Resolve_Typedef;

// 底盘结构体
typedef struct
{
    DJI_Motor_Instance *motor[CHASSIS_MOTOR_NUM];        // 4个电机实例
    PID speed_pid[CHASSIS_MOTOR_NUM];                    // 4个电机的速度PID
    Chassis_Mode_Typedef chassis_mode;                   // 底盘工作模式
    Chassis_Motion_Typedef motion;                       // 运动参数
    Chassis_Force_Feedback_Typedef feedback;             // 力反馈
    Chassis_Speed_Resolve_Typedef speed_resolve;         // 速度解算结果
    float power_buffer;                                  // 功率缓冲
    uint8_t chassis_online_flag;                         // 底盘在线标志
    uint32_t chassis_last_update_time;                   // 上次更新时间
} Chassis_Instance_Typedef;

/* Function prototypes */
/**
 * @brief  初始化底盘
 * @param  None
 * @retval 底盘实例指针
 */
Chassis_Instance_Typedef *Chassis_Init(void);

/**
 * @brief  底盘控制更新
 * @param  chassis: 底盘实例指针
 * @param  vx: 前后速度
 * @param  vy: 左右速度
 * @param  wz: 旋转速度
 * @retval None
 */
void Chassis_Control_Update(Chassis_Instance_Typedef *chassis, float vx, float vy, float wz);

/**
 * @brief  底盘速度解算（根据运动参数计算各电机目标速度）
 * @param  chassis: 底盘实例指针
 * @retval None
 */
void Chassis_Speed_Resolve(Chassis_Instance_Typedef *chassis);

/**
 * @brief  底盘功率控制
 * @param  chassis: 底盘实例指针
 * @retval None
 */
void Chassis_Power_Control(Chassis_Instance_Typedef *chassis);

/**
 * @brief  底盘力控制（电机电流环控制）
 * @param  chassis: 底盘实例指针
 * @retval None
 */
void Chassis_Force_Control(Chassis_Instance_Typedef *chassis);

/**
 * @brief  底盘在线检测
 * @param  chassis: 底盘实例指针
 * @retval None
 */
void Chassis_AliveCheck(Chassis_Instance_Typedef *chassis);

/**
 * @brief  底盘模式设置
 * @param  chassis: 底盘实例指针
 * @param  mode: 底盘模式
 * @retval None
 */
void Chassis_Set_Mode(Chassis_Instance_Typedef *chassis, Chassis_Mode_Typedef mode);

/**
 * @brief  底盘停止
 * @param  chassis: 底盘实例指针
 * @retval None
 */
void Chassis_Stop(Chassis_Instance_Typedef *chassis);

/**
 * @brief 绑定电机实例到底盘
 * @param chassis 底盘实例
 * @param motor_idx 电机索引 (0-3)
 * @param motor 电机实例指针
 * @retval None
 */
void Chassis_Bind_Motor(Chassis_Instance_Typedef *chassis, uint8_t motor_idx, DJI_Motor_Instance *motor);

#endif // CHASSIS_H
