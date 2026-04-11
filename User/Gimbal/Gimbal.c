/**
 * @file    Gimbal.c
 * @brief   云台执行层 - 纯电机PID控制
 * 
 * @note    【模块职责】
 *          - 电机初始化与硬件配置
 *          - 双环PID闭环控制 (位置环+速度环)
 *          - 电机指令输出
 *          - 参考角度限位
 */
#include "Gimbal.h"
#include "PID.h"
#include "fdcan.h"
#include "robot.h"
#include "can_comm.h"
#include "arm_math.h"
#include "motor.h"
#include "remote.h"
#include "bsp_dwt.h"
#include <math.h>
#include <stdint.h>

/* ==================== 全局实例 ==================== */
Gimbal_t gimbal = {0};
extern Robot_ctrl_cmd_t robot_cmd;
extern uint8_t imu_online;

#define POWERON_CENTER_STABLE_TICKS      200U
#define POWERON_CENTER_YAW_ERR_THRESHOLD 120.0f
#define POWERON_CENTER_PITCH_ERR_THRESHOLD 80.0f

uint8_t power_on_center_flag = 0U;
static uint16_t gimbal_poweron_center_stable_cnt = 0U;

/* ==================== PID模式定义 ==================== */
/**
 * @brief PID参数组索引
 * @note  不同模式使用不同PID参数，适应各场景需求
 */
typedef enum {
    PID_GYRO   = 0,  // 陀螺仪闭环 (FOLLOW/SPIN模式)
    PID_MECH   = 1,  // 机械角闭环 (NORMAL模式)
    PID_CENTER = 2,  // 归中控制 (快速响应)
    PID_AIM    = 3,  // 自瞄模式 (高精度跟踪)
} GimbalPidMode_e;
/* ==================== 底盘跟随PID ==================== */
// 位置环: 计算底盘应旋转的速度目标
static PID_Smis Chassis_Rotate_PIDS = {
    .Kp = 1.0f, .Ki = 0, .Kd = -8.0f,
    .interlimit = 1500, .outlimit = 2000 * PI,
    .DeadBand = 0, .inter_threLow = 500, .inter_threUp = 2000
};

// 速度环: 平滑底盘旋转速度
static PID Chassis_Rotate_PID = {
    .Kp = 1.0f, .Ki = 0.0f, .Kd = 0,
    .interlimit = 4500 * PI, .outlimit = 4000 * PI,
    .DeadBand = 0, .inter_threLow = 10 * PI, .inter_threUp = 4000 * PI
};

// 小陀螺Yaw前馈: 补偿底盘旋转造成的角度偏差
static FeedForward_Typedef GimbalYaw_FF = {
    .K1 = 0.0f, .K2 = 20.0f, .K3 = 0.0f, .OutMax = 3000, .DWT_CNT = 0
};

/* ==================== 云台电机PID ==================== */
/**
 * Yaw轴位置环 (PID_Smis: Smith预估补偿)
 * [0]=GYRO: 陀螺仪模式, 响应快
 * [1]=MECH: 机械角模式, 稳定
 * [2]=CENTER: 归中模式, 快速收敛
 * [3]=AIM: 自瞄模式, 高精度
 */
static PID_Smis Yaw_Pos_PID[4] = {
    {.Kp = 9.0f,  .Ki = 0.0f, .Kd = -35.0f, .interlimit = 3000, .outlimit = 20000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 22.0f, .Ki = 0.0f, .Kd = -20.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 10.0f, .Ki = 0.0f, .Kd = -20.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 14.0f, .Ki = 0.1f, .Kd = -37.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
};

// Pitch轴位置环
static PID_Smis Pitch_Pos_PID[4] = {
    {.Kp = -15.0f, .Ki = 0.0f, .Kd = 10.0f,  .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f,  .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 27.0f,  .Ki = 0.0f, .Kd = -10.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.01f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 30.0f,  .Ki = 0.0f, .Kd = -25.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f,  .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = -240.0f,.Ki = 0.0f, .Kd = 7.0f,   .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f,  .inter_threLow = 50, .inter_threUp = 500},
};

// Yaw轴速度环
static PID Yaw_Speed_PID[4] = {
    {.Kp = 80.0f,  .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 2.0f,   .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 2.0f,   .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 110.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
};

// Pitch轴速度环
static PID Pitch_Speed_PID[4] = {
    {.Kp = 80.0f, .Ki = 0.0f, .Kd = 10.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 90.0f, .Ki = 0.5f, .Kd = 0.0f,  .interlimit = 3000, .outlimit = 24000, .DeadBand = 1.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 2.5f,  .Ki = 0.5f, .Kd = 0.0f,  .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 12.0f, .Ki = 0.5f, .Kd = 0.0f,  .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
};

/* ==================== 内部函数声明 ==================== */
static void Gimbal_Stop(void);
static void Pitch_Limit(uint8_t imu_online);
static void Chassis_Follow_Calc(void);
static int32_t gimbal_center(int32_t yaw_now);
static int32_t front_center_target(int32_t yaw_now);
static void GetFeedbackData(uint8_t imu_online, float *yaw_fdb, float *pitch_fdb,
                            float *yaw_rate, float *pitch_rate, float *yaw_ref, float *pitch_ref);
/* VisionCanAutoAim() 由 Gimbal_Cmd.c 定义 */
extern uint8_t VisionCanAutoAim(void);
/* ==================== 云台初始化 ==================== */
void Gimbal_Init(void)
{
    // Yaw轴DM4310配置 (使用DJI协议模式)
    DM_Motor_Info_Typedef yaw_dm4310 = {
        .type = DM_4310,
        .Control_Mode = DJI_MODEL,
        .CANFrame = {
            .can_handle = &hfdcan3,
            .tx_id = 0x3FE,
            .rx_id = 0x302,
        },
    };

    // Pitch轴DM3507配置 (使用DJI协议模式)
    DM_Motor_Info_Typedef pitch_dm3507 = {
        .type = DM_3507,
        .Control_Mode = DJI_MODEL,
        .CANFrame = {
            .can_handle = &hfdcan3,
            .tx_id = 0x3FE,
            .rx_id = 0x301,
        },
    };

    // 注册电机
    gimbal.pitch_motor = DM_Motor_Init(&pitch_dm3507);
    gimbal.yaw_motor = DM_Motor_Init(&yaw_dm4310);

    // 初始化参考角度为当前位置 (避免上电跳变)
    robot_cmd.gimbal_mode = GIMBAL_STOP;
    robot_cmd.Mech_Yaw = gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
    robot_cmd.Mech_Pitch = gimbal.pitch_motor->Data.DJI_data.MechanicalAngle;
    robot_cmd.Gyro_Yaw = INS_Info.Yaw_TolAngle;
    robot_cmd.Gyro_Pitch = INS_Info.Pitch_Angle;

    power_on_center_flag = 0U;
    gimbal_poweron_center_stable_cnt = 0U;
}
/* ==================== 云台主控制函数 ==================== */
/**
 * @brief 云台控制主函数 - 1ms周期调用
 */
void Gimbal_Control(void)
{
    int16_t can_send[4] = {0};
    float yaw_fdb = 0.0f;
    float pitch_fdb = 0.0f;
    float yaw_rate = 0.0f;
    float pitch_rate = 0.0f;
    float yaw_ref = 0.0f;
    float pitch_ref = 0.0f;
    uint8_t use_gyro_feedback = imu_online ? 1U : 0U;
    GimbalPidMode_e pid_mode = use_gyro_feedback ? PID_GYRO : PID_MECH;

    switch (robot_cmd.gimbal_mode)
    {
    case GIMBAL_STOP:
        Gimbal_Stop();
        return;
    default:
        break;
    }

    if (power_on_center_flag == 0U)
    {
        // 上电归中强制走机械角反馈和机械PID参数
        robot_cmd.Mech_Yaw = front_center_target(gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle);
        robot_cmd.Mech_Pitch = (uint16_t)limit((float)gimbal.pitch_motor->Data.DJI_data.MechanicalAngle,
                                               (float)MCH_UP_limit, (float)MCH_DOWN_limit);

        Pitch_Limit(0U);
        GetFeedbackData(0U, &yaw_fdb, &pitch_fdb, &yaw_rate, &pitch_rate, &yaw_ref, &pitch_ref);

        PID_Control_Smis(yaw_fdb, yaw_ref, &Yaw_Pos_PID[PID_CENTER], yaw_rate);
        PID_Control(yaw_rate, Yaw_Pos_PID[PID_CENTER].pid_out, &Yaw_Speed_PID[PID_CENTER]);

        PID_Control_Smis(pitch_fdb, pitch_ref, &Pitch_Pos_PID[PID_CENTER], pitch_rate);
        PID_Control(pitch_rate, Pitch_Pos_PID[PID_CENTER].pid_out, &Pitch_Speed_PID[PID_CENTER]);

        can_send[0] = (int16_t)Pitch_Speed_PID[PID_CENTER].pid_out;
        can_send[1] = (int16_t)Yaw_Speed_PID[PID_CENTER].pid_out;
        DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);

        if ((fabsf(yaw_ref - yaw_fdb) < POWERON_CENTER_YAW_ERR_THRESHOLD) &&
            (fabsf(pitch_ref - pitch_fdb) < POWERON_CENTER_PITCH_ERR_THRESHOLD))
        {
            gimbal_poweron_center_stable_cnt++;
        }
        else
        {
            gimbal_poweron_center_stable_cnt = 0U;
        }

        if (gimbal_poweron_center_stable_cnt >= POWERON_CENTER_STABLE_TICKS)
        {
            power_on_center_flag = 1U;
            gimbal_poweron_center_stable_cnt = 0U;

            // 归中完成后同步参考，避免切回陀螺仪闭环时跳变
            robot_cmd.Mech_Yaw = gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
            robot_cmd.Mech_Pitch = gimbal.pitch_motor->Data.DJI_data.MechanicalAngle;
            robot_cmd.Gyro_Yaw = INS_Info.Yaw_TolAngle;
            robot_cmd.Gyro_Pitch = INS_Info.Pitch_Angle;
        }
        return;
    }

    switch (robot_cmd.gimbal_mode)
    {
    case GIMBAL_RUNNING_FOLLOW:
    {
        float yaw_speed_ref = 0.0f;

        Chassis_Follow_Calc();
        Pitch_Limit(use_gyro_feedback);
        GetFeedbackData(use_gyro_feedback, &yaw_fdb, &pitch_fdb, &yaw_rate, &pitch_rate, &yaw_ref, &pitch_ref);

        if (use_gyro_feedback)
        {
            FeedForward_Calc(&GimbalYaw_FF, robot_cmd.rotate_feedforward);
        }

        PID_Control_Smis(yaw_fdb, yaw_ref, &Yaw_Pos_PID[pid_mode], yaw_rate);
        yaw_speed_ref = Yaw_Pos_PID[pid_mode].pid_out;
        if (use_gyro_feedback)
        {
            yaw_speed_ref += GimbalYaw_FF.Out;
        }
        PID_Control(yaw_rate, yaw_speed_ref, &Yaw_Speed_PID[pid_mode]);

        PID_Control_Smis(pitch_fdb, pitch_ref, &Pitch_Pos_PID[pid_mode], pitch_rate);
        PID_Control(pitch_rate, Pitch_Pos_PID[pid_mode].pid_out, &Pitch_Speed_PID[pid_mode]);

        can_send[0] = (int16_t)Pitch_Speed_PID[pid_mode].pid_out;
        can_send[1] = (int16_t)Yaw_Speed_PID[pid_mode].pid_out;
        DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);
        return;
    }

    case GIMBAL_RUNNING_NORMAL:
        Pitch_Limit(use_gyro_feedback);
        GetFeedbackData(use_gyro_feedback, &yaw_fdb, &pitch_fdb, &yaw_rate, &pitch_rate, &yaw_ref, &pitch_ref);

        PID_Control_Smis(yaw_fdb, yaw_ref, &Yaw_Pos_PID[pid_mode], yaw_rate);
        PID_Control(yaw_rate, Yaw_Pos_PID[pid_mode].pid_out, &Yaw_Speed_PID[pid_mode]);

        PID_Control_Smis(pitch_fdb, pitch_ref, &Pitch_Pos_PID[pid_mode], pitch_rate);
        PID_Control(pitch_rate, Pitch_Pos_PID[pid_mode].pid_out, &Pitch_Speed_PID[pid_mode]);

        can_send[0] = (int16_t)Pitch_Speed_PID[pid_mode].pid_out;
        can_send[1] = (int16_t)Yaw_Speed_PID[pid_mode].pid_out;
        DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);
        return;

    case GIMBAL_RUNNING_AIM:
    {
        float yaw_speed_ref = 0.0f;

        Pitch_Limit(use_gyro_feedback);
        GetFeedbackData(use_gyro_feedback, &yaw_fdb, &pitch_fdb, &yaw_rate, &pitch_rate, &yaw_ref, &pitch_ref);

        if (use_gyro_feedback)
        {
            robot_cmd.rotate_feedforward = can_comm_instance.can_comm_rx_data.chassis_gyro / 200.0f;
            FeedForward_Calc(&GimbalYaw_FF, robot_cmd.rotate_feedforward);
        }

        PID_Control_Smis(yaw_fdb, yaw_ref, &Yaw_Pos_PID[pid_mode], yaw_rate);
        yaw_speed_ref = Yaw_Pos_PID[pid_mode].pid_out + (use_gyro_feedback ? GimbalYaw_FF.Out : 0.0f);
        PID_Control(yaw_rate, yaw_speed_ref, &Yaw_Speed_PID[pid_mode]);

        PID_Control_Smis(pitch_fdb, pitch_ref, &Pitch_Pos_PID[pid_mode], pitch_rate);
        PID_Control(pitch_rate, Pitch_Pos_PID[pid_mode].pid_out, &Pitch_Speed_PID[pid_mode]);

        can_send[0] = (int16_t)Pitch_Speed_PID[pid_mode].pid_out;
        can_send[1] = (int16_t)Yaw_Speed_PID[pid_mode].pid_out;
        DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);
        return;
    }

    default:
        // 未知模式: 停止
        Gimbal_Stop();
        return;
    }
}

/* ==================== 云台停止 ==================== */
/**
 * @brief 云台停止 - 电机禁用并同步参考角度
 */
static void Gimbal_Stop(void)
{
    // 禁用电机 (安全最优先)
    DMMotorStop(gimbal.yaw_motor);
    DMMotorStop(gimbal.pitch_motor);

    // 清除所有PID参数集的积分项
    // 防止停止期间的积分饱和 (常见于紧急场景)
    for (uint8_t i = 0; i <= PID_AIM; i++)
    {
        PID_IoutReset(&Yaw_Speed_PID[i]);
        PID_IoutReset(&Pitch_Speed_PID[i]);
    }

    // === 同步机械角参考值 ===
    robot_cmd.Mech_Yaw = gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
    robot_cmd.Mech_Pitch = gimbal.pitch_motor->Data.DJI_data.MechanicalAngle;

    // === 同步陀螺仪角参考值 ===
    robot_cmd.Gyro_Yaw = INS_Info.Yaw_TolAngle;
    robot_cmd.Gyro_Pitch = INS_Info.Pitch_Angle;
}

/* ==================== 云台归中计算 ==================== */
/**
 * @brief 计算归中目标位置 (就近选择4个方位之一)
 *
 * 概念: "归中"表示将云台对齐到相对底盘的4个基本方位:
 *      - FRONT: 云台平行于底盘前方
 *      - LEFT:  90度左转
 *      - BACK:  180度后转
 *      - RIGHT: 90度右转
 *
 * 两种归中模式:
 * 1. 固定方位 (MID_FRONT, MID_BACK等)
 *    - 用户指定确切方位
 *    - 示例: 键鼠模式F键 → 归中到前方
 *
 * 2. 最近方位 (MID_NONE)
 *    - 算法自动选择最近的基本方位
 *    - 最小化旋转角度
 *    - 示例: 按下归中按钮 → 自动对齐最近方位
 *
 * 最近方位算法:
 * ┌───────────────────────────────────┐
 * │ 1. 连续角度转换为单圈 [0, 8191]    │
 * │ 2. 计算到4个点的距离              │
 * │ 3. 处理跨越: 若距离>4096→减8192   │
 * │ 4. 选择最小距离点                 │
 * │ 5. 返回: 当前位置 + 最小距离      │
 * └───────────────────────────────────┘
 */
static int32_t gimbal_center(int32_t yaw_now)
{
    // 情景1: 固定归中方位 - 前方
    if (robot_cmd.Mid_mode == MID_FRONT)
    {
        // 计算当前圈数索引
        int32_t current_turn_base = (yaw_now / 8192) * 8192;
        // 返回同圈内的目标
        return current_turn_base + Yaw_Mid_Front;
    }

    // 情景2: 固定归中方位 - 后方
    if (robot_cmd.Mid_mode == MID_BACK)
    {
        int32_t current_turn_base = (yaw_now / 8192) * 8192;
        return current_turn_base + Yaw_Mid_Back;
    }

    // 情景3: 最近方位归中算法
    // 连续角度转换为单圈表示 [0, 8191]
    const uint16_t alignment_targets[4] = {Yaw_Mid_Front, Yaw_Mid_Left, Yaw_Mid_Back, Yaw_Mid_Right};
    uint16_t yaw_in_turn = (uint16_t)((yaw_now % 8192 + 8192) % 8192);

    // 寻找最近的基本方位
    int16_t best_offset = 8192;

    for (uint8_t i = 0; i < 4; i++)
    {
        // 计算原始距离
        int16_t candidate_offset = (int16_t)alignment_targets[i] - (int16_t)yaw_in_turn;

        // 处理跨越: 若距离>180°, 走反向
        if (candidate_offset > 4096)
            candidate_offset -= 8192;  // 太远往前, 改走回头
        if (candidate_offset < -4096)
            candidate_offset += 8192;  // 太远往后, 改走向前

        // 记录最近距离
        if (abs(candidate_offset) < abs(best_offset))
        {
            best_offset = candidate_offset;
        }
    }

    // 返回目标连续角度 (当前位置+最短距离)
    return yaw_now + best_offset;
}

static int32_t front_center_target(int32_t yaw_now)
{
    int32_t yaw_in_turn = (yaw_now % 8192 + 8192) % 8192;
    int32_t target = yaw_now - yaw_in_turn + Yaw_Mid_Front;
    int32_t offset = target - yaw_now;

    if (offset > 4096)
    {
        target -= 8192;
    }
    else if (offset < -4096)
    {
        target += 8192;
    }

    return target;
}

/* ==================== Pitch轴限位 ==================== */
/**
 * @brief 限制Pitch轴运动范围
 * Pitch限位范围:
 * ┌─────────────────────────────────────────────────┐
 * │ 反馈源    │ 最小/最大      │ 单位   │ 典型值   │
 * ├─────────────────────────────────────────────────┤
 * │ FB_GYRO   │ -6° ~ 42°      │ 度     │ IMU角   │
 * │ FB_MECH   │ 4800~6000编码器│ 值     │ 编码器  │
 * └─────────────────────────────────────────────────┘
 */
static void Pitch_Limit(uint8_t imu_online)
{
    if (imu_online)
    {
        // IMU坐标系限位 (单位: 度)
        // 范围: [-6°, 42°] - RoboMaster云台典型值
        // -6° = 地平面下6°, 42° = 地平面上42°
        robot_cmd.Gyro_Pitch = limit(robot_cmd.Gyro_Pitch, IMU_UP_limit, IMU_DOWN_limit);
    }
    else  
    {
        // 机械编码器限位 (单位: 编码器计数)
        // 防止电机打到机械停止块
        // 典型范围: [4800, 6000] / 8192圈
        robot_cmd.Mech_Pitch = (uint16_t)limit((float)robot_cmd.Mech_Pitch,
                                                (float)MCH_UP_limit, (float)MCH_DOWN_limit);
    }
}
/* ==================== 底盘跟随计算 ==================== */
/**
 * @brief 底盘跟随控制 - 计算底盘应旋转的速度
 * @note  底盘跟随云台时, 云台不需要主动控制Yaw
 *        而是计算偏差让底盘旋转来跟随云台
 */
static void Chassis_Follow_Calc(void)
{
    int32_t yaw_now = gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
    int32_t target = gimbal_center(yaw_now);
    
    // 获取底盘陀螺仪反馈 (用于前馈补偿)
    robot_cmd.rotate_feedforward = can_comm_instance.can_comm_rx_data.chassis_gyro / 200.0f;
    
    // 位置环: 计算偏差
    PID_Control_Smis((float)yaw_now, (float)target, &Chassis_Rotate_PIDS, robot_cmd.rotate_feedforward);
    
    // 速度环: 平滑输出
    PID_Control(robot_cmd.rotate_feedforward, Chassis_Rotate_PIDS.pid_out, &Chassis_Rotate_PID);
    
    // 限幅并输出底盘旋转速度
    robot_cmd.rotate_speed = limit(Chassis_Rotate_PID.pid_out, (float)rotate_speed_MAX, -(float)rotate_speed_MAX);
}

/* ==================== 反馈数据获取 ==================== */
/**
 * @brief 统一获取反馈源数据 (消除代码重复)
 *
 * 目的: 集中管理反馈源数据初始化，消除代码重复，提高可维护性
 * 所有数据源都在一处处理
 *
 * 代码精简: 消除Motor_Control()内~20行重复代码
 */
static void GetFeedbackData(uint8_t imu_online, float *yaw_fdb, float *pitch_fdb,
                            float *yaw_rate, float *pitch_rate, float *yaw_ref, float *pitch_ref)
{
    if (imu_online == 0)
    {
        // 机械角反馈 (编码器反馈，原始电机信号)
        *yaw_fdb = (float)gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
        *pitch_fdb = (float)gimbal.pitch_motor->Data.DJI_data.MechanicalAngle;
        *yaw_rate = gimbal.yaw_motor->Data.DJI_data.SpeedFilter;
        *pitch_rate = gimbal.pitch_motor->Data.DJI_data.SpeedFilter;
        *yaw_ref = (float)robot_cmd.Mech_Yaw;
        *pitch_ref = (float)robot_cmd.Mech_Pitch;
    }
    else  // imu_online == 1
    {
        // 陀螺仪反馈 (IMU反馈，稳定的角度信号)
        *yaw_fdb = INS_Info.Yaw_TolAngle;
        *pitch_fdb = INS_Info.Pitch_Angle;
        *yaw_rate = INS_Info.Yaw_Gyro;
        *pitch_rate = INS_Info.Pitch_Gyro;
        *yaw_ref = robot_cmd.Gyro_Yaw;
        *pitch_ref = robot_cmd.Gyro_Pitch;
    }
}
