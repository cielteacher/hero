/**
 * @file    robot.h
 * @brief   机器人参数与状态定义
 * @note    定义了机器人各模块的工作模式、参数配置及控制命令结构体
 *          
 *          模块分工:
 *          - Gimbal_Cmd.c (决策层): 输入解析、模式决策、目标角度更新、板间通信
 *          - Gimbal.c (执行层): 云台PID控制、电机驱动
 *          - Shoot.c (执行层): 发射PID控制、电机驱动
 */
#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>
#include "stdbool.h"
#include "RMLibHead.h"

/* ============ 开发板类型定义 ============ */
//#define ONE_BOARD       // 单片机控整车
//#define CHASSIS_BOARD   // 底盘板
#define GIMBAL_BOARD      // 云台板

/* ============ 视觉数据传输类型定义 ============ */
#define VISION_USE_VCP    // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* ============ 陀螺仪选择 ============ */
#define BMI088_INS
//#define WT931_IMU

/* ============ 云台参数 ============ */
#define Yaw_Mid_Front 3374  //!< Yaw轴电机中值(前方) [0~8191]
#if (Yaw_Mid_Front + 2048) > 8191
#define Yaw_Mid_Left (Yaw_Mid_Front - 6143)
#else
#define Yaw_Mid_Left (Yaw_Mid_Front + 2048)
#endif

#if (Yaw_Mid_Left + 2048) > 8191
#define Yaw_Mid_Back (Yaw_Mid_Left - 6143)
#else
#define Yaw_Mid_Back (Yaw_Mid_Left + 2048)
#endif

#if (Yaw_Mid_Back + 2048) > 8191
#define Yaw_Mid_Right (Yaw_Mid_Back - 6143)
#else
#define Yaw_Mid_Right (Yaw_Mid_Back + 2048)
#endif

#define Pitch_Mid 234  //!< Pitch轴电机中值 [0~8191]

/* ============ Pitch轴限位 ============ */
#define IMU_UP_limit     42.0f   //!< IMU坐标系上限位 (度)
#define IMU_DOWN_limit   -6.0f   //!< IMU坐标系下限位 (度)
#define MCH_UP_limit     6000    //!< 机械角上限位 [0~8191]
#define MCH_DOWN_limit   4800    //!< 机械角下限位 [0~8191]

/* ============ 发射机构参数 ============ */
#define PLUCK_SINGLE_ANGLE 1382         //!< 拨弹盘单发角度 (编码器单位)
#define UNJAM_TIME 200                  //!< 卡弹判定时间 (ms)
#define FRIC_SPEED_UP_TIME 300          //!< 摩擦轮加速时间 (ms)
#define SHOOT_SPEED_Front 3700          //!< 前摩擦轮目标转速 (RPM)
#define SHOOT_SPEED_Behind 4000         //!< 后摩擦轮目标转速 (RPM)
#define SHOOT_UNIT_HEAT_17MM 5          //!< 17mm弹丸单发热量
#define SHOOT_UNIT_HEAT_42MM 100        //!< 42mm弹丸单发热量

/* ============ 控制灵敏度 ============ */
#define RC_YAW_SENSITIVITY   0.005f     //!< 遥控器Yaw灵敏度 (度/单位)
#define RC_PITCH_SENSITIVITY 0.003f     //!< 遥控器Pitch灵敏度 (度/单位)
#define MOUSE_YAW_SENSITIVITY   0.08f   //!< 鼠标Yaw灵敏度 (度/像素)
#define MOUSE_PITCH_SENSITIVITY 0.05f   //!< 鼠标Pitch灵敏度 (度/像素)
#define MECH_YAW_SENSITIVITY   8.0f     //!< 机械角Yaw灵敏度 (编码器/单位)
#define MECH_PITCH_SENSITIVITY 6.0f     //!< 机械角Pitch灵敏度 (编码器/单位)

/* ============ 底盘参数 ============ */
#define rotate_speed_MAX 4000           //!< 最大旋转速度
#define WHEEL_LF 0              //!< 左前轮索引
#define WHEEL_RF 1              //!< 右前轮索引
#define WHEEL_LB 2              //!< 左后轮索引
#define WHEEL_RB 3              //!< 右后轮索引
/*============  板间通信定义  ============*/
// 云台->底盘数据包
#pragma pack(1) // 压缩结构体,取消字节对齐
typedef union
{
  uint8_t raw;
  struct {
      uint8_t Close_flag        : 1;
      uint8_t Unlimit_flag      : 1;
      uint8_t Gimbal_Online     : 1;
      uint8_t Shoot_Online      : 1;
      uint8_t Vision_Online     : 1;
      uint8_t Reserve1          : 1;
      uint8_t Reserve2          : 1;
      uint8_t Reserve3          : 1;
    } bits;
}Gimbal_Flag;
typedef struct  {
    int16_t vx;            	// 单位 基准速度的倍率（基准速度由底盘模块根据功率自动计算）
    int16_t vy;            	// 单位 基准速度的倍率
    int16_t rotate;        	// 单位 旋转速度度每秒
	  Gimbal_Flag flag;	   	//!< @brief 底盘关闭标志位
    uint8_t Chassis_Mode;     //!< @brief 底盘模式
} Gimbal_board_send_t;
// 云台<-底盘数据包
typedef struct {
  int16_t chassis_gyro;  // 将底盘主控的imu数据发到云台
  int16_t chassis_slop;  // 底盘所在斜坡倾角
  uint8_t game_progress; // 比赛状态
uint16_t heat_limit_remain;   // 剩余热量
uint8_t shoot_barrel_cooling; // 枪口热量冷却值
} Chassis_board_send_t;
#pragma pack()
/* ============ 数据结构定义 ============ */

/**
 * @brief 底盘速度结构体
 */
typedef struct {
    float X;      //!< 前后速度
    float Y;      //!< 左右速度
    float Omega;  //!< 旋转速度
} Chassis_speed_s;

/**
 * @brief 卡弹状态枚举
 */
typedef enum {
    JAM_IDLE = 0,     //!< 空闲
    JAM_BACKWARD,     //!< 反转退弹
    JAM_WAIT,         //!< 等待
    JAM_FORWARD,      //!< 正转恢复
} Jam_State_e;

/**
 * @brief 机器人状态枚举
 */
typedef enum {
    ROBOT_STOP = 0,   //!< 停止
    ROBOT_RUNNING     //!< 运行
} Robot_State_e;

/**
 * @brief 发射状态枚举
 */
typedef enum {
    SHOOT_DISABLED = 0,     //!< 完全失能 (不执行任何控制)
    SHOOT_STOP ,            //!< 停止（摩擦轮和拨弹盘速度为0）
    SHOOT_READY_NOFRIC,    //!< 准备(摩擦轮不启动)
    SHOOT_READY,           //!< 准备(摩擦轮启动)
    SHOOT_AIM,             //!< 自瞄开火
    SHOOT_STUCKING,        //!< 卡弹处理中
    SHOOT_FIRE,            //!< 手动开火
    SHOOT_CHECKOUT,        //!< 检录模式
    SHOOT_COOLING,         //!< 热量冷却
} ShootState_e;

/**
 * @brief 云台状态枚举
 */
typedef enum {
    GIMBAL_DISABLED = 0,   //!< 完全失能 (不执行任何控制)
    GIMBAL_STOP,           //!< 停止 (保持当前角度，失能电机输出)
    GIMBAL_RUNNING,        //!< 运行
    GIMBAL_RUNNING_AIM,    //!< 自瞄模式
} GimbalState_e;

/**
 * @brief 底盘状态枚举
 */
typedef enum {
    CHASSIS_DISABLED = 0,     //!< 完全失能 (不执行任何控制)
    CHASSIS_STOP = 1,        //!< 停止
    CHASSIS_RUNNING_FOLLOW = 2,  //!< 跟随云台
    CHASSIS_RUNNING_SPIN = 4,    //!< 小陀螺
    CHASSIS_RUNNING_NORMAL = 8,  //!< 独立移动
} ChassisState_e;

/**
 * @brief 控制模式枚举
 */
typedef enum {
    CONTROL_REMOTE = 0,    //!< 遥控器
    CONTROL_KEY_MOUSE = 1  //!< 键鼠
} ControlMode_e;

/**
 * @brief 归中模式枚举
 */
typedef enum {
    MID_NONE = 0,   //!< 就近归中
    MID_FRONT = 1,  //!< 前方归中
    MID_BACK = 2,   //!< 后方归中
} eMidMode;

#ifdef GIMBAL_BOARD
/**
 * @brief 机器人控制命令结构体
 * @note  全局共享，由Gimbal_Cmd.c决策层更新，Gimbal.c/Shoot.c执行层读取
 */
typedef struct {
    /* ===== 状态模式 ===== */
    Robot_State_e robot_state;      //!< 整车状态 (停止/运行)
    ControlMode_e Control_mode;     //!< 控制方式 (遥控器/键鼠)
    GimbalState_e gimbal_mode;      //!< 云台模式
    ChassisState_e chassis_mode;    //!< 底盘模式
    ShootState_e shoot_mode;        //!< 发射模式
    eMidMode Mid_mode;              //!< 归中角度选择
    bool Speed_Up_flag;             //!< 加速标志
    bool Unlimit_flag;             //!< 解除功率限制标志 (底盘)
    /* ===== 云台陀螺仪参考 (IMU坐标系) ===== */
    float Gyro_Position_Yaw;                 //!< Yaw目标位置 (度, 连续累计)
    float Gyro_Position_Pitch;               //!< Pitch目标位置 (度, -180~180)

} Robot_ctrl_cmd_t;
#endif

#ifdef CHASSIS_BOARD
/**
 * @brief 底盘板控制命令结构体
 * @note  全局共享，由Chassis_Cmd.c决策层更新，Chassis.c执行层读取
 */
typedef struct {
    /* ===== 状态模式 ===== */
    ChassisState_e chassis_mode;    //!< 底盘模式


} Chassis_board_cmd_t;
#endif
#endif /* __ROBOT_H */