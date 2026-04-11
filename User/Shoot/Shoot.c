/**
 * @file    Shoot.c
 * @brief   英雄机器人发射机构控制
 * @note    1ms控制周期，只支持单发模式
 *          所有控制状态由robot_cmd.shoot_mode决定
 */
#include "Shoot.h"
#include "MiniPC.h"
#include "robot.h"
#include "can_comm.h"

/* ============ 发射机构实例 ============ */
Shoot_t shoot = {0};
extern Robot_ctrl_cmd_t robot_cmd;

/* ============ PID参数配置 ============ */

/**
 * @brief 摩擦轮PID通用配置
 * @note  所有4个摩擦轮使用相同参数
 */
#define FRIC_PID_INIT { \
    .Kp = 7.0f, .Ki = 0.0f, .Kd = 0.0f, \
    .interlimit = 3000, .outlimit = 16000, \
    .DeadBand = 0.50f, .inter_threLow = 500, .inter_threUp = 1000 \
}

/* 摩擦轮速度环PID数组 */
static PID Fric_Speed_PID[4] = {
    FRIC_PID_INIT, FRIC_PID_INIT, FRIC_PID_INIT, FRIC_PID_INIT
};

/* 拨弹盘位置环PID */
static PID_Smis Pluck_Place_PID = {
    .Kp = 6.0f, .Ki = 0, .Kd = 0.0f,
    .interlimit = 3000, .outlimit = 16000,
    .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000
};

/* 拨弹盘速度环PID */
static PID Pluck_Speed_PID = {
    .Kp = 4.0f, .Ki = 0.0f, .Kd = 50.0f,
    .interlimit = 5000, .outlimit = 15000,
    .DeadBand = 0.0f, .inter_threLow = 20, .inter_threUp = 5000
};

static uint8_t single_fire_state = 0;

/* ============ 函数声明 ============ */
static void Shoot_Stop(void);
static void Fric_Wheel_Control(void);
static void Single_Fire(void);
static void Single_Fire_Reset(void);
static void Jam_Handle(void);
static void Pluck_Hold(void);
static uint8_t VisionFireAllowed(void);

/* ============ 发射机构初始化 ============ */
void Shoot_Init(void)
{
    // 摩擦轮通用配置模板
    DJI_Motor_Config fric_config = {
        .Type = DJI3508,
        .Control_Type = NONE_LOOP,
        .Can_Config = {
            .can_handle = &hfdcan1,
            .tx_id = 0x200,
        }
    };

    // 注册4个摩擦轮电机 (rx_id: 0x201~0x204)
    fric_config.Can_Config.rx_id = 0x201;
    shoot.fric_motor_1 = DJI_Motor_Init(&fric_config);
    fric_config.Can_Config.rx_id = 0x202;
    shoot.fric_motor_2 = DJI_Motor_Init(&fric_config);
    fric_config.Can_Config.rx_id = 0x203;
    shoot.fric_motor_3 = DJI_Motor_Init(&fric_config);
    fric_config.Can_Config.rx_id = 0x204;
    shoot.fric_motor_4 = DJI_Motor_Init(&fric_config);

    // 拨弹盘电机配置
    DJI_Motor_Config pluck_config = {
        .Type = DJI3508,
        .Control_Type = NONE_LOOP,
        .Can_Config = {
            .can_handle = &hfdcan2,
            .tx_id = 0x200,
            .rx_id = 0x201,
        }
    };
    shoot.Pluck_motor = DJI_Motor_Init(&pluck_config);

    // 初始化状态
    robot_cmd.shoot_mode = SHOOT_STOP;
    shoot.pluck_target_angle = 0;
}
/**
 * @brief 卡弹检测 - 通过速度和电流异常判断
 */
static void Jam_Check(void)
{
    static uint16_t stuck_time = 0;

    if (robot_cmd.shoot_mode == SHOOT_FIRE || robot_cmd.shoot_mode == SHOOT_AIM)
    {
        // 速度过低或电流过高可能表示卡弹
        if (fabsf(shoot.Pluck_motor->Data.SpeedFilter) <= 50.0f ||
            fabsf(shoot.Pluck_motor->Data.CurrentFilter) >= 5000.0f)
        {
            stuck_time++;
            if (stuck_time >= UNJAM_TIME)
            {
                robot_cmd.shoot_mode = SHOOT_STUCKING;
            }
        }
        else
        {
            stuck_time = 0;
        }
    }
    else
    {
        stuck_time = 0;
    }
}

/* ==================== 热量检测 ==================== */
static void Heat_Check(void)
{
    if (can_comm_instance.can_comm_online_flag == 0)
    {
        robot_cmd.shoot_mode = SHOOT_STOP;
        return;
    }

    uint16_t remain = can_comm_instance.can_comm_rx_data.heat_limit_remain;
    uint8_t cooling_rate = can_comm_instance.can_comm_rx_data.shoot_barrel_cooling;
    float heat_budget = (float)remain + cooling_rate * 0.001f;
    const float cool_enter_threshold = SHOOT_UNIT_HEAT_42MM * 1.2f;
    const float cool_exit_threshold = SHOOT_UNIT_HEAT_42MM * 1.8f;

    if (robot_cmd.shoot_mode == SHOOT_COOLING)
    {
        // 使用滞回阈值退出冷却，避免模式在边界点频繁抖动
        if (heat_budget >= cool_exit_threshold)
        {
            robot_cmd.shoot_mode = SHOOT_READY;
        }
        return;
    }

    if (robot_cmd.shoot_mode == SHOOT_FIRE || robot_cmd.shoot_mode == SHOOT_AIM)
    {
        // 剩余热量不足时进入冷却
        if (heat_budget < cool_enter_threshold)
        {
            robot_cmd.shoot_mode = SHOOT_COOLING;
        }
    }
}
/* ============ 发射机构主控制函数 - 1ms周期 ============ */
void Shoot_Control(void)
{
    // 每个控制周期先做安全检查，再进入状态机
    Heat_Check();
    Jam_Check();

    if (robot_cmd.shoot_mode != SHOOT_FIRE && robot_cmd.shoot_mode != SHOOT_AIM)
    {
        Single_Fire_Reset();
    }

    switch (robot_cmd.shoot_mode)
    {
        case SHOOT_STOP:
            Shoot_Stop();
            break;

        case SHOOT_AIM:
        {
            static uint16_t auto_fire_div = 0;
            Fric_Wheel_Control();
            if (VisionFireAllowed())
            {
                auto_fire_div++;
                if (auto_fire_div >= 80)
                {
                    auto_fire_div = 0;
                    Single_Fire();
                }
                else
                {
                    Pluck_Hold();
                }
            }
            else
            {
                auto_fire_div = 0;
                Pluck_Hold();
            }
            break;
        }
            
        case SHOOT_READY:
            Fric_Wheel_Control();
            Pluck_Hold();
            break;
            
        case SHOOT_FIRE:
            shoot.pluck_lock = 0; // 解锁拨弹盘位置
            Fric_Wheel_Control();
            Single_Fire();
            break;
            
        case SHOOT_STUCKING:
            Fric_Wheel_Control();
            Jam_Handle();
            break;

        case SHOOT_CHECKOUT:
            Fric_Wheel_Control();
            Pluck_Hold();
            break;
            
        case SHOOT_COOLING:
            Fric_Wheel_Control();
            Pluck_Hold();
            break;
            
        default:
            Shoot_Stop();
            break;
    }
}

/* ============ 发射机构停止 ============ */
static void Shoot_Stop(void)
{
    DJIMotorStop(shoot.fric_motor_1);
    DJIMotorStop(shoot.fric_motor_2);
    DJIMotorStop(shoot.fric_motor_3);
    DJIMotorStop(shoot.fric_motor_4);
    DJIMotorStop(shoot.Pluck_motor);
    Single_Fire_Reset();
    
    // 清除PID积分项
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_IoutReset(&Fric_Speed_PID[i]);
    }
    PID_IoutReset(&Pluck_Speed_PID);
}

/* ============ 拨弹盘保持位置 ============ */
static void Pluck_Hold(void)
{
	if(shoot.pluck_lock == 0)
	{
		shoot.pluck_target_angle = shoot.Pluck_motor->Data.Continuous_Mechanical_angle;
		shoot.pluck_lock = 1;
	}
    PID_Control_Smis(shoot.Pluck_motor->Data.Continuous_Mechanical_angle,
                     shoot.pluck_target_angle, &Pluck_Place_PID,
                     shoot.Pluck_motor->Data.SpeedFilter);
    PID_Control(shoot.Pluck_motor->Data.SpeedFilter, Pluck_Place_PID.pid_out,
                &Pluck_Speed_PID);
    
    int16_t pluck_output[4] = {0};
    pluck_output[0] = (int16_t)Pluck_Speed_PID.pid_out;
    DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, pluck_output);
}

/* ============ 单发控制 ============ */
static void Single_Fire(void)
{
    if (single_fire_state == 0)
    {
        shoot.pluck_target_angle =shoot.Pluck_motor->Data.Continuous_Mechanical_angle  + PLUCK_SINGLE_ANGLE;
        single_fire_state = 1;
    }
    
    // 双环PID控制
    PID_Control_Smis(shoot.Pluck_motor->Data.Continuous_Mechanical_angle,
                     shoot.pluck_target_angle, &Pluck_Place_PID,
                     shoot.Pluck_motor->Data.SpeedFilter);
    PID_Control(shoot.Pluck_motor->Data.SpeedFilter, Pluck_Place_PID.pid_out,
                &Pluck_Speed_PID);
    
    int16_t pluck_output[4] = {0};
    pluck_output[0] = (int16_t)Pluck_Speed_PID.pid_out;
    DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, pluck_output);

    // 判断到位
    if (fabsf(shoot.Pluck_motor->Data.Continuous_Mechanical_angle - shoot.pluck_target_angle) <= 5.0f)
    {
        robot_cmd.shoot_mode = SHOOT_READY;
        shoot.pluck_lock = 0;
        single_fire_state = 0;
    }
}

static void Single_Fire_Reset(void)
{
    single_fire_state = 0;
}

/**
 * @brief 检查视觉是否允许开火
 * @return 1=允许, 0=不允许
 */
static uint8_t VisionFireAllowed(void)
{
    return (MiniPC_instance.MiniPC_Online_Flag &&
            MiniPC_instance.receive_data.data.dis <= 0.2f) ? 1U : 0U;
}

/**
 * @brief 摩擦轮控制 (4轮速度环)
 * @note  前后摩擦轮方向相反以产生旋转
 */
static void Fric_Wheel_Control(void)
{
    // 摩擦轮速度目标 (正负表示方向)
    static const float fric_targets[4] = {
        SHOOT_SPEED_Front,   // 电机1 (左前正转)
        -SHOOT_SPEED_Front,  // 电机2 (右前反转)
        SHOOT_SPEED_Behind,  // 电机3 (左后正转)
        -SHOOT_SPEED_Behind  // 电机4 (右后反转)
    };
    
    // 获取各电机实例指针数组便于遍历
    DJI_Motor_Instance *fric_motors[4] = {
        shoot.fric_motor_1, shoot.fric_motor_2, 
        shoot.fric_motor_3, shoot.fric_motor_4
    };
    
    int16_t txbuffer[4] = {0};
    
    // PID计算
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Control(fric_motors[i]->Data.SpeedFilter, fric_targets[i], &Fric_Speed_PID[i]);
        txbuffer[i] = (int16_t)Fric_Speed_PID[i].pid_out;
    }
    
    DJI_Motor_CAN_TxMessage(shoot.fric_motor_1, txbuffer);
}

/**
 * @brief 卡弹处理状态机
 * @note  流程: 反转退弹 -> 等待 -> 正转恢复
 */
static void Jam_Handle(void)
{
    static Jam_State_e jam_state = JAM_IDLE;
    static float target_angle = 0;
    static uint16_t timer = 0;
    int16_t can_send[4] = {0};

    float current_angle = (float)shoot.Pluck_motor->Data.Continuous_Mechanical_angle;

    switch (jam_state)
    {
    case JAM_IDLE: // 初始化：设置反转目标
        target_angle = current_angle - PLUCK_SINGLE_ANGLE * 0.7f;
        timer = 0;
        jam_state = JAM_BACKWARD;
        break;

    case JAM_BACKWARD: // 反转退弹
        PID_Control_Smis(current_angle, target_angle, &Pluck_Place_PID,
                         shoot.Pluck_motor->Data.SpeedFilter);
        PID_Control(shoot.Pluck_motor->Data.SpeedFilter, Pluck_Place_PID.pid_out, &Pluck_Speed_PID);
        
        can_send[0] = (int16_t)Pluck_Speed_PID.pid_out;
        DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, can_send);

        if (fabsf(current_angle - target_angle) < 2.0f)
        {
            timer = 0;
            jam_state = JAM_WAIT;
        }
        break;

    case JAM_WAIT: // 等待弹丸释放
        DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, can_send); // 发送零输出

        timer++;
        if (timer > 100) // 100ms后准备正转
        {
            target_angle = current_angle + PLUCK_SINGLE_ANGLE;
            jam_state = JAM_FORWARD;
        }
        break;

    case JAM_FORWARD: // 正转恢复
        PID_Control_Smis(current_angle, target_angle, &Pluck_Place_PID,
                         shoot.Pluck_motor->Data.SpeedFilter);
        PID_Control(shoot.Pluck_motor->Data.SpeedFilter, Pluck_Place_PID.pid_out, &Pluck_Speed_PID);

        can_send[0] = (int16_t)Pluck_Speed_PID.pid_out;
        DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, can_send);

        if (fabsf(current_angle - target_angle) < 2.0f)
        {
            jam_state = JAM_IDLE;
            robot_cmd.shoot_mode = SHOOT_READY; // 退出卡弹处理
        }
        break;
    }
}
