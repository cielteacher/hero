/**
 * @file    Gimbal.c
 * @brief   云台执行层，执行cmd文件的指令 - 电机PID控制
 */
#include "Gimbal.h"
#include "PID.h"
#include "arm_math.h"
#include "bsp_dwt.h"
#include "can_comm.h"
#include "fdcan.h"
#include "motor.h"
#include "remote.h"
#include "robot.h"
#include <math.h>
#include <stdint.h>

/* ==================== 全局实例 ==================== */
Gimbal_t gimbal = {0};
extern Robot_ctrl_cmd_t robot_cmd;

#define CENTER_STABLE_TICKS    200U
#define CENTER_YAW_DEADBAND  0.1f
#define CENTER_PITCH_DEADBAND 0.1f
#define GIMBAL_STOP_OUT_MAX               14000.0f

uint8_t power_on_center_flag = 0U;
static uint16_t gimbal_poweron_center_stable_cnt = 0U;

typedef enum
{
    PID_GYRO = 0,
    PID_MECH = 1, // 保留索引位，避免历史 PID 参数表下标变化
    PID_CENTER = 2,
    PID_AIM = 3,
} GimbalPidMode_e;

/* Yaw 前馈 */
static FeedForward_Typedef GimbalYaw_FF = {
    .K1 = 0.0f,
    .K2 = 20.0f,
    .K3 = 0.0f,
    .OutMax = 3000,
    .DWT_CNT = 0,
};

/* Yaw 位置环 */
static PID_Smis Yaw_Pos_PID[4] = {
    {.Kp = 9.0f, .Ki = 0.0f, .Kd = -35.0f, .interlimit = 3000, .outlimit = 20000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 22.0f, .Ki = 0.0f, .Kd = -20.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 10.0f, .Ki = 0.0f, .Kd = -20.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 14.0f, .Ki = 0.1f, .Kd = -37.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
};

/* Pitch 位置环 */
static PID_Smis Pitch_Pos_PID[4] = {
    {.Kp = -15.0f, .Ki = 0.0f, .Kd = 10.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 27.0f, .Ki = 0.0f, .Kd = -10.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.01f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 30.0f, .Ki = 0.0f, .Kd = -25.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = -240.0f, .Ki = 0.0f, .Kd = 7.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
};

/* Yaw 速度环 */
static PID Yaw_Speed_PID[4] = {
    {.Kp = 80.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 2.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 2.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 110.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
};

/* Pitch 速度环 */
static PID Pitch_Speed_PID[4] = {
    {.Kp = 80.0f, .Ki = 0.0f, .Kd = 10.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 90.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 1.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 2.5f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 12.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
};

/* ==================== 内部函数声明 ==================== */
static void Gimbal_Stop(void);
static void Gimbal_Disable(void);
static void ResetGimbalPidIout(void);
static void Pitch_Limit(void);
/* ==================== Pitch 限位 ==================== */
static void Pitch_Limit(void)
{
    robot_cmd.Gyro_Position_Pitch = limit( robot_cmd.Gyro_Position_Pitch, IMU_UP_limit, IMU_DOWN_limit);
}
static void ResetGimbalPidIout(void)
{
    for (uint8_t i = 0; i <= PID_AIM; i++)
    {
        PID_IoutReset(&Yaw_Speed_PID[i]);
        PID_IoutReset(&Pitch_Speed_PID[i]);
    }
}
/* ==================== 云台初始化 ==================== */
void Gimbal_Init(void)
{
    DM_Motor_Info_Typedef yaw_dm4310 = {
        .type = DM_4310,
        .Control_Mode = DJI_MODEL,
        .CANFrame = {
            .can_handle = &hfdcan3,
            .tx_id = 0x3FE,
            .rx_id = 0x302,
        },
    };

    DM_Motor_Info_Typedef pitch_dm3507 = {
        .type = DM_3507,
        .Control_Mode = DJI_MODEL,
        .CANFrame = {
            .can_handle = &hfdcan3,
            .tx_id = 0x3FE,
            .rx_id = 0x301,
        },
    };

    gimbal.pitch_motor = DM_Motor_Init(&pitch_dm3507);
    gimbal.yaw_motor = DM_Motor_Init(&yaw_dm4310);

    robot_cmd.gimbal_mode = GIMBAL_STOP;
    power_on_center_flag = 0U;// 上电归中标志位，置位后才允许正常控制逻辑
    gimbal_poweron_center_stable_cnt = 0U;// 上电归中稳定计数器，连续满足条件达到一定次数后认为归中完成
}

/* ==================== 云台主控制函数 ==================== */
void Gimbal_Control(void)
{
    int16_t can_send[4] = {0};
    if (gimbal.yaw_motor == NULL || gimbal.pitch_motor == NULL)
    {
        return;
    }
    /* 停止/失能状态提前处理 */
    if(robot_cmd.gimbal_mode == GIMBAL_DISABLED)
    {
        Gimbal_Disable();
        return;
    }
    else if(robot_cmd.gimbal_mode == GIMBAL_STOP)
    {
        Gimbal_Stop();
        return;
    }
     /* PITCH目标位置限位 */
        Pitch_Limit();
    /* 上电归中（强制机械反馈 + CENTER 参数） */
    if (power_on_center_flag == 0U)
    {
        // 这里单独使用目标角度 不与robot_cmd.Gyro_Position混用，防止突变
        uint16_t target_yaw = QuickCentering(gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle,Yaw_Mid_Front );
        PID_Control_Smis(gimbal.yaw_motor->Data.DJI_data.MechanicalAngle, (float)target_yaw, &Yaw_Pos_PID[PID_CENTER], gimbal.yaw_motor->Data.DJI_data.SpeedFilter);
        PID_Control(gimbal.yaw_motor->Data.DJI_data.SpeedFilter, Yaw_Pos_PID[PID_CENTER].pid_out, &Yaw_Speed_PID[PID_CENTER]);

        PID_Control_Smis(gimbal.pitch_motor->Data.DJI_data.MechanicalAngle, (float)Pitch_Mid, &Pitch_Pos_PID[PID_CENTER], gimbal.pitch_motor->Data.DJI_data.SpeedFilter);
        PID_Control(gimbal.pitch_motor->Data.DJI_data.SpeedFilter, Pitch_Pos_PID[PID_CENTER].pid_out, &Pitch_Speed_PID[PID_CENTER]);

        can_send[0] = (int16_t)Pitch_Speed_PID[PID_CENTER].pid_out;
        can_send[1] = (int16_t)Yaw_Speed_PID[PID_CENTER].pid_out;
        DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);

        if ((fabsf(Yaw_Pos_PID[PID_CENTER].error_last) < CENTER_YAW_DEADBAND) &&
            (fabsf(Pitch_Pos_PID[PID_CENTER].error_last) < CENTER_PITCH_DEADBAND))
        {
            gimbal_poweron_center_stable_cnt++;
        }
        else
        {
            gimbal_poweron_center_stable_cnt = 0U;
        }

        if (gimbal_poweron_center_stable_cnt >= CENTER_STABLE_TICKS)
        {
            power_on_center_flag = 1U;
            gimbal_poweron_center_stable_cnt = 0U;
            // 上电归中完成，允许进入正常控制逻辑,更新位置数据，防止突变
            robot_cmd.Gyro_Position_Yaw = INS_Info.Yaw_TolAngle;
            robot_cmd.Gyro_Position_Pitch = INS_Info.Pitch_Angle;
        }
        return;
    }

    switch (robot_cmd.gimbal_mode)
    {
    case GIMBAL_RUNNING_AIM:
    {   // 计算前馈 防止云台跟随底盘转动
        FeedForward_Calc(&GimbalYaw_FF, can_comm_instance.can_comm_rx_data.chassis_gyro);

        PID_Control_Smis(INS_Info.Yaw_TolAngle, robot_cmd.Gyro_Position_Yaw, &Yaw_Pos_PID[PID_AIM], INS_Info.Yaw_Gyro);
        PID_Control(INS_Info.Yaw_Gyro, Yaw_Pos_PID[PID_AIM].pid_out, &Yaw_Speed_PID[PID_AIM]);

        PID_Control_Smis(INS_Info.Pitch_Angle,robot_cmd.Gyro_Position_Pitch,&Pitch_Pos_PID[PID_AIM], INS_Info.Pitch_Gyro);
        PID_Control(INS_Info.Pitch_Gyro, Pitch_Pos_PID[PID_AIM].pid_out, &Pitch_Speed_PID[PID_AIM]);

        can_send[0] = (int16_t)Pitch_Speed_PID[PID_AIM].pid_out;
        can_send[1] = (int16_t)Yaw_Speed_PID[PID_AIM].pid_out;
        DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);
        break;
    }
    case GIMBAL_RUNNING:
    {   // 计算前馈 防止云台跟随底盘转动
        FeedForward_Calc(&GimbalYaw_FF, can_comm_instance.can_comm_rx_data.chassis_gyro);

        PID_Control_Smis(INS_Info.Yaw_TolAngle, robot_cmd.Gyro_Position_Yaw, &Yaw_Pos_PID[PID_AIM], INS_Info.Yaw_Gyro);
        PID_Control(INS_Info.Yaw_Gyro, Yaw_Pos_PID[PID_AIM].pid_out, &Yaw_Speed_PID[PID_AIM]);

        PID_Control_Smis(INS_Info.Pitch_Angle,robot_cmd.Gyro_Position_Pitch,&Pitch_Pos_PID[PID_AIM], INS_Info.Pitch_Gyro);
        PID_Control(INS_Info.Pitch_Gyro, Pitch_Pos_PID[PID_AIM].pid_out, &Pitch_Speed_PID[PID_AIM]);
        can_send[0] = (int16_t)Pitch_Speed_PID[PID_AIM].pid_out;
        can_send[1] = (int16_t)Yaw_Speed_PID[PID_AIM].pid_out;
        DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);
        break;
    }
    default:
        Gimbal_Stop();
        return;
    }
}

/* ==================== 云台失能与急停 ==================== */
static void Gimbal_Disable(void)
{
    DMMotordisable(gimbal.yaw_motor);
    DMMotordisable(gimbal.pitch_motor);
    ResetGimbalPidIout();
}

static void Gimbal_Stop(void)
{
    int16_t can_send[4] = {0};
    // 停止态语义为急停：速度环目标置 0，由 PID 闭环主动制动。
    PID_Control( gimbal.yaw_motor->Data.DJI_data.SpeedFilter, 0.0f, &Yaw_Speed_PID[PID_MECH]);
    PID_Control(gimbal.pitch_motor->Data.DJI_data.SpeedFilter, 0.0f, &Pitch_Speed_PID[PID_MECH]);

    can_send[0] = (int16_t)limit(Pitch_Speed_PID[PID_MECH].pid_out,
                                 GIMBAL_STOP_OUT_MAX,
                                 -GIMBAL_STOP_OUT_MAX);
    can_send[1] = (int16_t)limit(Yaw_Speed_PID[PID_MECH].pid_out,
                                 GIMBAL_STOP_OUT_MAX,
                                 -GIMBAL_STOP_OUT_MAX);

    DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);
    ResetGimbalPidIout();
}







