/**
 * @file    Gimbal.c
 * @brief   云台执行层 - 纯电机PID控制
 * 
 * @note    【模块职责】
 *          - 电机初始化与硬件配置
 *          - 双环PID闭环控制 (位置环+速度环)
 *          - 电机指令输出
 *          - 参考角度限位
 *          
 *          【数据流向】
 *          robot_cmd (由Gimbal_Cmd.c更新) --> Motor_Control() --> CAN发送
 *          
 *          【反馈源选择】
 *          - IMU在线时: 陀螺仪反馈 (INS_Info) + 陀螺仪参考 (Gyro_*)
 *          - IMU离线时: 机械角反馈 (编码器) + 机械角参考 (Mech_*)
 *          - NORMAL模式: 始终使用机械角
 *          
 *          【控制周期】 1ms (由Task_Robot_Control调用)
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

/**
 * @brief 反馈源类型
 * @note  用于Motor_Control()选择反馈来源
 */
typedef enum {
    FB_GYRO = 0,  // 使用陀螺仪反馈
    FB_MECH = 1,  // 使用机械角反馈
} FeedbackSource_e;

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
static void Motor_Control(GimbalPidMode_e pid_mode, FeedbackSource_e fb_src);
static void Pitch_Limit(FeedbackSource_e fb_src);
static void Chassis_Follow_Calc(void);
static void Rotate_Speed_Set(void);
static int32_t Gimbal_CenterAlign(int32_t yaw_now);
static void GetFeedbackData(FeedbackSource_e fb_src, float *yaw_fdb, float *pitch_fdb,
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
}
/* ==================== 云台主控制函数 ==================== */
/**
 * @brief 云台控制主函数 - 1ms周期调用
 * @note  控制流程:
 *        1. 停止模式 → 电机停转，同步参考角度
 *        2. 获取已决策的反馈源 (由Cmd层根据IMU状态决定)
 *        3. 应用限位 (根据反馈源统一处理)
 *        4. IMU离线降级 (所有模式 → 机械角)
 *        5. 选择PID参数 (根据模式)
 *        6. 执行统一的电机控制
 *
 * 关键改进: 单一IMU决策点
 * - 旧: Gimbal_Control()重复判断IMU (冗余)
 * - 新: 使用Cmd层已决策的gimbal_fb
 * - 结果: 代码清晰，易于维护
 *
 * 反馈源策略:
 * - FB_GYRO: 用于跟随/自瞄/小陀螺 (需要IMU)
 * - FB_MECH: 用于NORMAL模式或IMU离线
 */
void Gimbal_Control(void)
{
    // 状态: 停止模式 → 电机停转并同步参考角度
    if (robot_cmd.gimbal_mode == GIMBAL_STOP)
    {
        Gimbal_Stop();
        return;
    }

    // 获取已决策的反馈源 (由Cmd层根据IMU状态决定)
    // 不需要在此再检查IMU状态 - Cmd层已处理
    FeedbackSource_e fb_src = (FeedbackSource_e)robot_cmd.gimbal_fb;

    // 根据反馈源统一处理限位
    // 防止机械越界或IMU角度越限
    Pitch_Limit(fb_src);

    /**
     * @brief IMU离线降级逻辑
     *
     * 条件: FB_MECH (由Cmd层在IMU离线时设置) && !NORMAL模式
     * 原因: IMU失信号时，除NORMAL外的所有模式应该
     *       降级到机械角反馈 + 机械PID参数
     * 效果: 优雅降级 - 机器人继续工作，但失去稳定性
     *
     * 示例场景:
     * - FOLLOW模式+IMU离线 → 降级到机械角+PID_MECH
     * - SPIN模式+IMU离线   → 降级到机械角+PID_MECH
     * - NORMAL模式         → 始终使用机械角(无需改变)
     */
    if (fb_src == FB_MECH && robot_cmd.gimbal_mode != GIMBAL_RUNNING_NORMAL)
    {
        // IMU失效时的直接机械控制
        Motor_Control(PID_MECH, FB_MECH);
        return;
    }

    /**
     * @brief 模式→PID参数映射
     *
     * 这个映射根据云台模式选择合适的PID参数集。
     * 每个模式都有独特的控制特性:
     * - PID_GYRO: 快速响应，用于跟随/小陀螺 (陀螺仪反馈)
     * - PID_MECH: 稳定控制，用于NORMAL模式 (机械反馈)
     * - PID_AIM:  精准跟踪，用于自瞄 (高增益)
     * - PID_CENTER: 归中动作 (当前未使用)
     */
    GimbalPidMode_e pid_mode = PID_GYRO;  // 默认参数集

    switch (robot_cmd.gimbal_mode)
    {
    case GIMBAL_RUNNING_FOLLOW:
        // 底盘跟随模式: 计算底盘旋转以保持云台居中
        Chassis_Follow_Calc();
        pid_mode = PID_GYRO;  // 使用陀螺仪PID参数
        break;

    case GIMBAL_RUNNING_AIM:
        // 自瞄模式: 有目标用AIM参数, 无目标降级用GYRO
        // 这避免了目标丢失时的抖动现象
        pid_mode = VisionCanAutoAim() ? PID_AIM : PID_GYRO;
        break;

    case GIMBAL_RUNNING_SPIN:
        // 小陀螺模式: 根据装甲板朝向动态调整旋转速度
        Rotate_Speed_Set();
        pid_mode = PID_GYRO;  // 使用陀螺仪PID参数
        break;

    case GIMBAL_RUNNING_NORMAL:
        // 普通模式: 直接编码器控制
        pid_mode = PID_MECH;  // 使用机械角PID参数
        break;

    default:
        // 未知模式: 停止
        Gimbal_Stop();
        return;
    }

    // 统一的PID控制执行 (使用选定的参数和反馈源)
    Motor_Control(pid_mode, fb_src);
}

/* ==================== 云台停止 ==================== */
/**
 * @brief 云台停止 - 电机禁用并同步参考角度
 *
 * 调用时机:
 * - 机器人进入STOP状态 (紧急关闭)
 * - 云台收到GIMBAL_STOP指令
 * - 系统故障检测到
 *
 * 关键动作:
 * 1. 禁用电机 (防止失控)
 * 2. 清除所有PID积分项 (防止积分饱和)
 * 3. 同步参考角度到当前反馈
 *    目的: 云台恢复时不会有突跳
 *    原理: (参考值 = 反馈值 => 误差 = 0, 平滑启动)
 *
 * 参考值同步策略:
 * - Mech_Yaw/Pitch: 同步到编码器反馈
 *   何时: 恢复使用机械反馈 (NORMAL模式或IMU离线)
 * - Gyro_Yaw/Pitch: 同步到IMU反馈
 *   何时: 恢复使用陀螺仪反馈 (跟随/自瞄/小陀螺模式+IMU在线)
 *
 * 好处: 停止后平滑过渡, 无突跳运动
 *
 * @note 这个函数必须在停止时调用以保证系统完整性
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
    // 使用场景: NORMAL模式或IMU离线 (机械反馈)
    robot_cmd.Mech_Yaw = gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
    robot_cmd.Mech_Pitch = gimbal.pitch_motor->Data.DJI_data.MechanicalAngle;

    // === 同步陀螺仪角参考值 ===
    // 使用场景: 跟随/自瞄/小陀螺模式+IMU在线 (陀螺仪反馈)
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
 *
 * 示例:
 * - 当前角度: 7000 (接近0°)
 * - 前方点: 1000
 * - 后方点: 5096 (180°)
 * - 最近: 前方 (距离 = 1000-7000+8192 = 2192)
 *
 * @param yaw_now 当前连续机械角
 * @return 目标连续机械角 (可跨越多圈)
 *
 * @note "连续"表示角度可超出[0,8191]用于多圈跟踪
 */
static int32_t Gimbal_CenterAlign(int32_t yaw_now)
{
    // 情景1: 固定归中方位 - 前方
    if (robot_cmd.Mid_mode == MID_FRONT)
    {
        // 计算当前圈数索引
        int32_t round_base = (yaw_now / 8192) * 8192;
        // 返回同圈内的目标
        return round_base + Yaw_Mid_Front;
    }

    // 情景2: 固定归中方位 - 后方
    if (robot_cmd.Mid_mode == MID_BACK)
    {
        int32_t round_base = (yaw_now / 8192) * 8192;
        return round_base + Yaw_Mid_Back;
    }

    // 情景3: 最近方位归中算法
    // 连续角度转换为单圈表示 [0, 8191]
    const uint16_t mids[4] = {Yaw_Mid_Front, Yaw_Mid_Left, Yaw_Mid_Back, Yaw_Mid_Right};
    uint16_t yaw_single = (uint16_t)((yaw_now % 8192 + 8192) % 8192);

    // 寻找最近的基本方位
    int16_t best_dist = 8192;

    for (uint8_t i = 0; i < 4; i++)
    {
        // 计算原始距离
        int16_t dist = (int16_t)mids[i] - (int16_t)yaw_single;

        // 处理跨越: 若距离>180°, 走反向
        if (dist > 4096)
            dist -= 8192;  // 太远往前, 改走回头
        if (dist < -4096)
            dist += 8192;  // 太远往后, 改走向前

        // 记录最近距离
        if (abs(dist) < abs(best_dist))
        {
            best_dist = dist;
        }
    }

    // 返回目标连续角度 (当前位置+最短距离)
    return yaw_now + best_dist;
}

/* ==================== Pitch轴限位 ==================== */
/**
 * @brief 限制Pitch轴运动范围
 *
 * Pitch限位范围:
 * ┌─────────────────────────────────────────────────┐
 * │ 反馈源    │ 最小/最大      │ 单位   │ 典型值   │
 * ├─────────────────────────────────────────────────┤
 * │ FB_GYRO   │ -6° ~ 42°      │ 度     │ IMU角   │
 * │ FB_MECH   │ 4800~6000编码器│ 值     │ 编码器  │
 * └─────────────────────────────────────────────────┘
 *
 * 为什么需要两套限位?
 * - 不同反馈源有不同特性:
 *   - 陀螺仪: 相对机体的角度 (平滑, 范围有限)
 *   - 机械: 绝对编码器位置 (全范围, 物理限制)
 * - 限位必须匹配反馈源类型以防越界
 *
 * 调用时机:
 * - Gimbal_Control()开始时总是调用
 * - 在模式切换前处理
 * - 保护云台机械结构不要超界
 *
 * @param fb_src 反馈源类型 (决定使用哪套限位)
 *
 * 实现原理:
 * limit()函数将值限制在[min, max]范围内:
 *   if (value > max) value = max;
 *   if (value < min) value = min;
 */
static void Pitch_Limit(FeedbackSource_e fb_src)
{
    if (fb_src == FB_GYRO)
    {
        // IMU坐标系限位 (单位: 度)
        // 范围: [-6°, 42°] - RoboMaster云台典型值
        // -6° = 地平面下6°, 42° = 地平面上42°
        robot_cmd.Gyro_Pitch = limit(robot_cmd.Gyro_Pitch, IMU_UP_limit, IMU_DOWN_limit);
    }
    else  // FB_MECH
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
    int32_t target = Gimbal_CenterAlign(yaw_now);
    
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
 * @param fb_src 输入 - 反馈源 (FB_GYRO 或 FB_MECH)
 * @param yaw_fdb, pitch_fdb       输出 - 位置反馈指针
 * @param yaw_rate, pitch_rate     输出 - 速度反馈指针
 * @param yaw_ref, pitch_ref       输出 - 参考角度指针
 *
 * 反馈源详解:
 * - FB_MECH: 编码器反馈 (Continuous_Mechanical_angle, MechanicalAngle)
 *            用于直接机械控制 (NORMAL模式或IMU离线)
 *            单位: 编码器值 (每圈8192)
 *
 * - FB_GYRO: IMU反馈 (Yaw_TolAngle, Pitch_Angle from INS_Info)
 *            用于稳定控制 (跟随/自瞄/小陀螺模式)
 *            单位: 度
 *
 * 代码精简: 消除Motor_Control()内~20行重复代码
 */
static void GetFeedbackData(FeedbackSource_e fb_src, float *yaw_fdb, float *pitch_fdb,
                            float *yaw_rate, float *pitch_rate, float *yaw_ref, float *pitch_ref)
{
    if (fb_src == FB_MECH)
    {
        // 机械角反馈 (编码器反馈，原始电机信号)
        *yaw_fdb = (float)gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
        *pitch_fdb = (float)gimbal.pitch_motor->Data.DJI_data.MechanicalAngle;
        *yaw_rate = gimbal.yaw_motor->Data.DJI_data.SpeedFilter;
        *pitch_rate = gimbal.pitch_motor->Data.DJI_data.SpeedFilter;
        *yaw_ref = (float)robot_cmd.Mech_Yaw;
        *pitch_ref = (float)robot_cmd.Mech_Pitch;
    }
    else  // FB_GYRO
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

/* ==================== 电机统一控制 ==================== */
/**
 * @brief 云台电机双环PID控制 (统一实现)
 *
 * 控制架构:
 * ┌────────────────────────────────────────┐
 * │ 位置环错误 → 位置环PID                  │
 * │ (输出速度指令)                         │
 * ├────────────────────────────────────────┤
 * │ 速度环错误 → 速度环PID                  │
 * │ (输出电机电流)                         │
 * ├────────────────────────────────────────┤
 * │ 跟随模式: 添加底盘旋转前馈补偿         │
 * └────────────────────────────────────────┘
 *
 * 统一接口优势:
 * - 所有反馈源类型使用同一入口 (FB_GYRO/FB_MECH)
 * - 无论反馈类型，双环结构一致
 * - 前馈补偿统一应用
 *
 * @param pid_mode PID参数集索引 (PID_GYRO, PID_MECH, PID_AIM, PID_CENTER)
 * @param fb_src   反馈源类型 (FB_GYRO: IMU反馈, FB_MECH: 编码器反馈)
 *
 * 执行流程:
 * 1. 调用GetFeedbackData()获取反馈数据
 * 2. 计算小陀螺前馈 (若使用陀螺仪反馈)
 * 3. Yaw轴双环:
 *    - 位置环: 从角度误差计算所需速度
 *    - 速度环: 从速度误差转换电机电流
 *    - 跟随模式添加前馈
 * 4. Pitch轴双环 (结构相同)
 * 5. 通过CAN发送电机指令
 */
static void Motor_Control(GimbalPidMode_e pid_mode, FeedbackSource_e fb_src)
{
    int16_t can_send[4] = {0};
    float yaw_fdb, pitch_fdb, yaw_rate, pitch_rate, yaw_ref, pitch_ref;

    // 统一获取反馈数据 (消除分支)
    GetFeedbackData(fb_src, &yaw_fdb, &pitch_fdb, &yaw_rate, &pitch_rate, &yaw_ref, &pitch_ref);

    // 计算小陀螺前馈补偿 (仅陀螺仪反馈时)
    // 补偿底盘旋转对云台的影响
    if (fb_src == FB_GYRO)
    {
        FeedForward_Calc(&GimbalYaw_FF, robot_cmd.rotate_feedforward);
    }

    // ===== Yaw轴双环 =====
    // 位置环: 将角度误差转换为速度指令
    PID_Control_Smis(yaw_fdb, yaw_ref, &Yaw_Pos_PID[pid_mode], yaw_rate);

    // 速度环: 跟随模式时叠加前馈补偿
    float yaw_speed_ref = Yaw_Pos_PID[pid_mode].pid_out;
    if (fb_src == FB_GYRO && robot_cmd.gimbal_mode == GIMBAL_RUNNING_FOLLOW)
    {
        // 前馈: 直接使用底盘旋转指令+云台补偿
        yaw_speed_ref = Chassis_Rotate_PIDS.pid_out + GimbalYaw_FF.Out;
    }
    PID_Control(yaw_rate, yaw_speed_ref, &Yaw_Speed_PID[pid_mode]);

    // ===== Pitch轴双环 =====
    // 位置环: 将角度误差转换为速度指令
    PID_Control_Smis(pitch_fdb, pitch_ref, &Pitch_Pos_PID[pid_mode], pitch_rate);
    // 速度环: 从速度误差转换电机电流
    PID_Control(pitch_rate, Pitch_Pos_PID[pid_mode].pid_out, &Pitch_Speed_PID[pid_mode]);

    // ===== 发送CAN电机指令 =====
    can_send[0] = (int16_t)Pitch_Speed_PID[pid_mode].pid_out;
    can_send[1] = (int16_t)Yaw_Speed_PID[pid_mode].pid_out;
    DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);
}
/* ==================== 小陀螺速度设置 ==================== */
/**
 * @brief 小陀螺旋转速度设置 (按Yaw角度变速)
 * @note  根据装甲板朝向动态调整转速
 *        装甲板正对敌方时加速，对角时减速
 *        目的是减少被击中概率
 */
static void Rotate_Speed_Set(void)
{
    // 获取Yaw单圈角度 [0, 8191]
    float yaw_single = fmodf((float)gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle, 8192.0f);
    if (yaw_single < 0.0f) yaw_single += 8192.0f;

    // 四个装甲板扇区中心 (每扇区约2048编码器值)
    const float sector_centers[] = {865.0f, 2755.0f, 4805.0f, 6855.0f, 8036.0f};
    const float sector_bounds[][2] = {
        {0.0f, 1730.0f}, {1730.0f, 3780.0f}, {3780.0f, 5830.0f}, 
        {5830.0f, 7880.0f}, {7880.0f, 8191.0f}
    };

    // 计算角度比例因子 (距扇区中心的偏移)
    float angle_ratio = 0.0f;
    for (uint8_t i = 0; i < 5; i++)
    {
        if (yaw_single >= sector_bounds[i][0] && yaw_single <= sector_bounds[i][1])
        {
            angle_ratio = fabsf(sector_centers[i] - yaw_single) / 2050.57f;
            break;
        }
    }

    // 变速公式: 基础转速 + 余弦调制
    // 扇区中心(角度比0) -> cos(0)=1 -> 最大附加速度
    // 扇区边缘(角度比1) -> cos(π)=-1 -> 最小附加速度
    robot_cmd.rotate_speed = 700.0f * PI + 800.0f * PI * arm_cos_f32(angle_ratio * PI);
}
