#ifndef TYPE_MOON_DJIMOTOR_H
#define TYPE_MOON_DJIMOTOR_H

#include "bsp_fdcan.h"
#include "motor.h"

// 大疆电机数量定义
#define DJI_MOTOR_CNT 5 /* 使能大疆电机总数 */

// 电机类型枚举
typedef enum
{
    DJI6020,
    DJI3508,
    DJI2060,
    NO_TYPE = 0,
} DJIMOTOR_TYPE;

// 电机数据结构体
// 不同电机/电调的反馈数据单位不同，注意甄别
// 6020：-16384~0~16384, 对应最大转矩电流范围 -3A~0~3A
typedef struct
{
    int16_t Current;                     /*!< 扭矩电流-16384~0~16384   -20~0~20A */
    int16_t Velocity;                    /*!< Motor rotate velocity (RPM) */
    int16_t round;                       /*!< 圈数 */
    uint16_t MechanicalAngle;             /*!< Motor encoder angle */
    uint16_t Last_MechanicalAngle;        /*!< previous Motor encoder angle */
    int32_t Continuous_Mechanical_angle; /*!< 连续机械角度 */
    float Angle;                         
    float Angle_DEG;
    float SpeedFilter;   /*!< 转速 /rpm */
    float CurrentFilter; /*!< 转矩电流 */
    uint8_t Temperature; /*!< Motor Temperature */

} DJI_Motor_Data_Typedef;

// 电机设置
typedef struct
{
    DJIMOTOR_TYPE Type;                 // 电机形式
    DJIMOTOR_CONTROL_TYPE Control_Type; // 控制方式
    CAN_Init_Config_s Can_Config;       // Can设置
} DJI_Motor_Config;

// 大疆结构体实例
typedef struct
{
    CANInstance *CANFrame;                  /*!< CAN设置 */
    DJI_Motor_Data_Typedef Data;            /*!< 数据 */
    DJIMOTOR_CONTROL_TYPE motor_controller; /*!< 控制方式 */
    DJIMOTOR_TYPE Type;                     /*!< 电机种类 */
    volatile uint8_t dji_motor_online_flag; /*!< 电机是否在线 */
    volatile uint32_t djimotor_last_rx_ms;
} DJI_Motor_Instance;

// 外部全局变量声明
extern DJI_Motor_Instance dji_motor_instance[DJI_MOTOR_CNT];

// 函数声明
DJI_Motor_Instance *DJI_Motor_Init(DJI_Motor_Config *_config);
void DJIMotordisable(DJI_Motor_Instance *motor);
void DJI_Motor_CAN_TxMessage(DJI_Motor_Instance *DJI_Motor, int16_t *txbuffer);
void DJI_Motor_AliveCheck(void);
#endif
