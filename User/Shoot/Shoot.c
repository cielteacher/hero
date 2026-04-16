/**
 * @file    Shoot.c
 * @brief   发射控制层 - 摩擦轮、拨弹盘、卡弹恢复
 */
#include "Shoot.h"
#include "MiniPC.h"
#include "can_comm.h"
#include "robot.h"
#include <math.h>
#include <stdint.h>

Shoot_t shoot = {0};
extern Robot_ctrl_cmd_t robot_cmd;

/* ==================== 发射参数 ==================== */
#define FRIC_PID_INIT                                                                                 \
    {                                                                                                 \
        .Kp = 7.0f, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.50f, \
        .inter_threLow = 500, .inter_threUp = 1000                                                   \
    }

#define SINGLE_FIRE_POS_ERR_TH 5.0f
#define SINGLE_FIRE_BRAKE_MIN_CYCLES 2U
#define SINGLE_FIRE_BRAKE_MAX_CYCLES 5U
#define SINGLE_FIRE_BRAKE_SPEED_TH 20.0f

/* ==================== 全局/静态变量 ==================== */
static PID Fric_Speed_PID[4] = {
    FRIC_PID_INIT,
    FRIC_PID_INIT,
    FRIC_PID_INIT,
    FRIC_PID_INIT,
};

static PID_Smis Pluck_Place_PID = {
    .Kp = 6.0f,
    .Ki = 0,
    .Kd = 0.0f,
    .interlimit = 3000,
    .outlimit = 16000,
    .DeadBand = 0.0f,
    .inter_threLow = 500,
    .inter_threUp = 1000,
};

static PID Pluck_Speed_PID = {
    .Kp = 4.0f,
    .Ki = 0.0f,
    .Kd = 50.0f,
    .interlimit = 5000,
    .outlimit = 15000,
    .DeadBand = 0.0f,
    .inter_threLow = 20,
    .inter_threUp = 5000,
};

static uint8_t fire_step = 0U;            // 单发流程状态：0待机 1拨弹中 2刹车停盘
static uint8_t fire_brake_ticks = 0U;     // 刹车阶段持续周期计数
static uint8_t keep_aim_after_fire = 0U;  // 单发结束后是否保持自瞄模式
static uint16_t auto_fire_tick = 0U;      // 自瞄触发计时，持续锁定后自动单发
static uint16_t jam_ticks = 0U;           // 卡弹检测累计时间
static Jam_State_e jam_state = JAM_IDLE;  // 卡弹处理状态机
static float jam_target_angle = 0.0f;     // 解卡过程目标角度
static uint16_t jam_timer = 0U;           // 解卡等待计时器
/* ==================== 内部函数声明 ==================== */
static void Shoot_Disable(void);
static void Shoot_Stop(void);
static void FricWheelControl(void);
static void SingleFire(uint8_t keep_aim_mode);
static void JamCheck(void);
static void JamHandle(void);
static void PluckHold(void);
static void HeatCheck(void);
static void ResetShootState(void);

/* ==================== 机器人初始化 ==================== */
void Shoot_Init(void)
{
    DJI_Motor_Config fric_config = {
        .Type = DJI3508,
        .Control_Type = SPEED_LOOP,
        .Can_Config = {
            .can_handle = &hfdcan1,
            .tx_id = 0x200,
        },
    };

    fric_config.Can_Config.rx_id = 0x201;
    shoot.fric_motor_LF = DJI_Motor_Init(&fric_config);
    fric_config.Can_Config.rx_id = 0x202;
    shoot.fric_motor_RF = DJI_Motor_Init(&fric_config);
    fric_config.Can_Config.rx_id = 0x203;
    shoot.fric_motor_LB = DJI_Motor_Init(&fric_config);
    fric_config.Can_Config.rx_id = 0x204;
    shoot.fric_motor_RB = DJI_Motor_Init(&fric_config);

    DJI_Motor_Config pluck_config = {
        .Type = DJI3508,
        .Control_Type = NONE_LOOP,
        .Can_Config = {
            .can_handle = &hfdcan2,
            .tx_id = 0x200,
            .rx_id = 0x201,
        },
    };
    shoot.Pluck_motor = DJI_Motor_Init(&pluck_config);
    robot_cmd.shoot_mode = SHOOT_STOP;
    ResetShootState();
}

/* ==================== 发射主控制 ==================== */
void Shoot_Control(void)
{
    if (shoot.Pluck_motor == NULL || shoot.fric_motor_LF == NULL || shoot.fric_motor_RF == NULL ||
        shoot.fric_motor_LB == NULL || shoot.fric_motor_RB == NULL)
    {
        return;
    }

    if (robot_cmd.shoot_mode == SHOOT_DISABLED)
    {
        Shoot_Disable();
        return;
    }

    if (robot_cmd.shoot_mode != SHOOT_AIM)
    {
        auto_fire_tick = 0U;
    }

    HeatCheck();
    JamCheck();

    if (robot_cmd.shoot_mode != SHOOT_FIRE && robot_cmd.shoot_mode != SHOOT_AIM)
    {
        fire_step = 0U;
        fire_brake_ticks = 0U;
        keep_aim_after_fire = 0U;
    }

    switch (robot_cmd.shoot_mode)
    {
    case SHOOT_STOP:
        Shoot_Stop();
        break;

    case SHOOT_READY_NOFRIC:
    {
        DJI_Motor_Instance *fric_motors[4] = {
            shoot.fric_motor_LF,
            shoot.fric_motor_RF,
            shoot.fric_motor_LB,
            shoot.fric_motor_RB,
        };
        int16_t txbuffer[4] = {0};

        for (uint8_t i = 0; i < 4; i++)
        {
            PID_Control(fric_motors[i]->Data.SpeedFilter, 0.0f, &Fric_Speed_PID[i]);
            txbuffer[i] = (int16_t)Fric_Speed_PID[i].pid_out;
        }
        DJI_Motor_CAN_TxMessage(shoot.fric_motor_LF, txbuffer);
        PluckHold();
        break;
    }

    case SHOOT_READY:
    case SHOOT_CHECKOUT:
    case SHOOT_COOLING:
        FricWheelControl();
        PluckHold();
        break;

    case SHOOT_FIRE:
        shoot.pluck_lock = 0U;
        FricWheelControl();
        SingleFire(0U);
        break;

    case SHOOT_AIM:
        FricWheelControl();
        if (MiniPC_instance.MiniPC_Online_Flag && MiniPC_instance.receive_data.data.dis <= 0.2f)
        {
            auto_fire_tick++;
            if (auto_fire_tick >= 80U)
            {
                auto_fire_tick = 0U;
                SingleFire(1U);
            }
            else
            {
                PluckHold();
            }
        }
        else
        {
            auto_fire_tick = 0U;
            PluckHold();
        }
        break;

    case SHOOT_STUCKING:
        FricWheelControl();
        JamHandle();
        break;

    default:
        Shoot_Disable();
        break;
    }
}

/* ==================== 停止/失能 ==================== */
static void Shoot_Disable(void)
{
    DJIMotordisable(shoot.fric_motor_LF);
    DJIMotordisable(shoot.fric_motor_RF);
    DJIMotordisable(shoot.fric_motor_LB);
    DJIMotordisable(shoot.fric_motor_RB);
    DJIMotordisable(shoot.Pluck_motor);

    ResetShootState();
}

static void Shoot_Stop(void)
{
    DJI_Motor_Instance *fric_motors[4] = {
        shoot.fric_motor_LF,
        shoot.fric_motor_RF,
        shoot.fric_motor_LB,
        shoot.fric_motor_RB,
    };
    int16_t txbuffer[4] = {0};

    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Control(fric_motors[i]->Data.SpeedFilter, 0.0f, &Fric_Speed_PID[i]);
        txbuffer[i] = (int16_t)Fric_Speed_PID[i].pid_out;
    }
    DJI_Motor_CAN_TxMessage(shoot.fric_motor_LF, txbuffer);

    DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, (int16_t[4]){0});

    ResetShootState();
}

static void ResetShootState(void)
{
    fire_step = 0U;
    fire_brake_ticks = 0U;
    keep_aim_after_fire = 0U;
    auto_fire_tick = 0U;
    jam_ticks = 0U;
    jam_state = JAM_IDLE;
    jam_target_angle = 0.0f;
    jam_timer = 0U;
    shoot.pluck_lock = 0U;
    shoot.pluck_target_angle = 0;

    for (uint8_t i = 0; i < 4; i++)
    {
        PID_IoutReset(&Fric_Speed_PID[i]);
    }
    PID_IoutReset(&Pluck_Speed_PID);
}

/* ==================== 堵塞检查 ==================== */
static void JamCheck(void)
{
    if (robot_cmd.shoot_mode == SHOOT_FIRE ||
        (robot_cmd.shoot_mode == SHOOT_AIM && fire_step != 0U))
    {
        if (fabsf(shoot.Pluck_motor->Data.SpeedFilter) <= 50.0f ||
            fabsf(shoot.Pluck_motor->Data.CurrentFilter) >= 5000.0f)
        {
            jam_ticks++;
            if (jam_ticks >= UNJAM_TIME)
            {
                jam_ticks = 0U;
                robot_cmd.shoot_mode = SHOOT_STUCKING;
            }
        }
        else
        {
            jam_ticks = 0U;
        }
    }
    else
    {
        jam_ticks = 0U;
    }
}

/* ==================== 热量检查 ==================== */
static void HeatCheck(void)
{
    uint16_t remain;    // 当前热量剩余值
    uint8_t cooling_rate;// 当前散热速率
    float heat_margin;
    const float cool_enter_threshold = SHOOT_UNIT_HEAT_42MM * 1.2f;
    const float cool_exit_threshold = SHOOT_UNIT_HEAT_42MM * 1.8f;

    if (can_comm_instance.can_comm_online_flag == 0U)
    {
        robot_cmd.shoot_mode = SHOOT_STOP;
        return;
    }

    remain = can_comm_instance.can_comm_rx_data.heat_limit_remain;
    cooling_rate = can_comm_instance.can_comm_rx_data.shoot_barrel_cooling;
    heat_margin = (float)remain + cooling_rate * 0.001f;

    if (robot_cmd.shoot_mode == SHOOT_COOLING)
    {
        if (heat_margin >= cool_exit_threshold)
        {
            robot_cmd.shoot_mode = SHOOT_READY;
        }
        return;
    }

    if (robot_cmd.shoot_mode == SHOOT_FIRE || robot_cmd.shoot_mode == SHOOT_AIM)
    {
        if (heat_margin < cool_enter_threshold)
        {
            robot_cmd.shoot_mode = SHOOT_COOLING;
        }
    }
}

/* ==================== 拨弹盘控制 ==================== */
static void PluckHold(void)
{
    int16_t tx[4] = {0};

    if (shoot.pluck_lock == 0U)
    {
        shoot.pluck_target_angle = shoot.Pluck_motor->Data.Continuous_Mechanical_angle;
        shoot.pluck_lock = 1U;
    }

    PID_Control_Smis(shoot.Pluck_motor->Data.Continuous_Mechanical_angle,
                     shoot.pluck_target_angle,
                     &Pluck_Place_PID,
                     shoot.Pluck_motor->Data.SpeedFilter);
    PID_Control(shoot.Pluck_motor->Data.SpeedFilter,
                Pluck_Place_PID.pid_out,
                &Pluck_Speed_PID);

    tx[0] = (int16_t)limit(Pluck_Speed_PID.pid_out, 15000.0f, -15000.0f);
    DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, tx);
}
/* ==================== 单发控制 ==================== */
/**
 * @brief 单发状态机
 *
 * fire_step:
 * 0 -> 设置目标角度（拨一发）
 * 1 -> PID控制拨盘转到目标位置
 * 2 -> 反向刹车，防止惯性过冲导致连发
 */
static void SingleFire(uint8_t keep_aim_mode)
{
    float angle_err;
    int16_t tx[4] = {0};

    if (fire_step == 0U)
    {
        shoot.pluck_target_angle = shoot.Pluck_motor->Data.Continuous_Mechanical_angle + PLUCK_SINGLE_ANGLE;
        fire_step = 1U;
        fire_brake_ticks = 0U;
        keep_aim_after_fire = keep_aim_mode;
        return;
    }

    if (fire_step == 1U)
    {
        PID_Control_Smis(shoot.Pluck_motor->Data.Continuous_Mechanical_angle,
                         shoot.pluck_target_angle,
                         &Pluck_Place_PID,
                         shoot.Pluck_motor->Data.SpeedFilter);
        PID_Control(shoot.Pluck_motor->Data.SpeedFilter,
                    Pluck_Place_PID.pid_out,
                    &Pluck_Speed_PID);

        tx[0] = (int16_t)limit(Pluck_Speed_PID.pid_out, 15000.0f, -15000.0f);
        DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, tx);

        angle_err = fabsf(shoot.Pluck_motor->Data.Continuous_Mechanical_angle - shoot.pluck_target_angle);
        if (angle_err <= SINGLE_FIRE_POS_ERR_TH)
        {
            fire_step = 2U;
            fire_brake_ticks = 0U;
        }
        return;
    }

    if (fire_step == 2U)
    {
        float brake_output = -shoot.Pluck_motor->Data.SpeedFilter * 65.0f;

        tx[0] = (int16_t)limit(brake_output, 15000.0f, -15000.0f);
        DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, tx);

        fire_brake_ticks++;
        if ((fire_brake_ticks >= SINGLE_FIRE_BRAKE_MIN_CYCLES &&
             fabsf(shoot.Pluck_motor->Data.SpeedFilter) <= SINGLE_FIRE_BRAKE_SPEED_TH) ||
            (fire_brake_ticks >= SINGLE_FIRE_BRAKE_MAX_CYCLES))
        {
            shoot.pluck_lock = 0U;
            fire_step = 0U;
            fire_brake_ticks = 0U;
            robot_cmd.shoot_mode = keep_aim_after_fire ? SHOOT_AIM : SHOOT_READY;
        }
    }
}

/* ==================== 摩擦轮控制 ==================== */
static void FricWheelControl(void)
{
    static const float fric_targets[4] = {
        SHOOT_SPEED_Front,
        -SHOOT_SPEED_Front,
        SHOOT_SPEED_Behind,
        -SHOOT_SPEED_Behind,
    };

    DJI_Motor_Instance *fric_motors[4] = {
        shoot.fric_motor_LF,
        shoot.fric_motor_RF,
        shoot.fric_motor_LB,
        shoot.fric_motor_RB,
    };

    int16_t txbuffer[4] = {0};

    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Control(fric_motors[i]->Data.SpeedFilter, fric_targets[i], &Fric_Speed_PID[i]);
        txbuffer[i] = (int16_t)Fric_Speed_PID[i].pid_out;
    }

    DJI_Motor_CAN_TxMessage(shoot.fric_motor_LF, txbuffer);
}

/* ==================== 卡弹处理 ==================== */
/**
 * @brief 卡弹恢复状态机
 *
 * JAM_IDLE     : 初始化解卡流程
 * JAM_BACKWARD : 拨盘反转退弹
 * JAM_WAIT     : 等待机构稳定
 * JAM_FORWARD  : 正转复位
 */
static void JamHandle(void)
{
    float current_angle = (float)shoot.Pluck_motor->Data.Continuous_Mechanical_angle;
    int16_t tx[4] = {0};

    switch (jam_state)
    {
    case JAM_IDLE:
        jam_target_angle = current_angle - PLUCK_SINGLE_ANGLE * 0.7f;
        jam_timer = 0U;
        jam_state = JAM_BACKWARD;
        break;

    case JAM_BACKWARD:
        PID_Control_Smis(current_angle,
                         jam_target_angle,
                         &Pluck_Place_PID,
                         shoot.Pluck_motor->Data.SpeedFilter);
        PID_Control(shoot.Pluck_motor->Data.SpeedFilter,
                    Pluck_Place_PID.pid_out,
                    &Pluck_Speed_PID);
        tx[0] = (int16_t)limit(Pluck_Speed_PID.pid_out, 15000.0f, -15000.0f);
        DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, tx);

        if (fabsf(current_angle - jam_target_angle) < 2.0f)
        {
            jam_timer = 0U;
            jam_state = JAM_WAIT;
        }
        break;

    case JAM_WAIT:
        DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, (int16_t[4]){0});
        jam_timer++;
        if (jam_timer > 100U)
        {
            jam_target_angle = current_angle + PLUCK_SINGLE_ANGLE;
            jam_state = JAM_FORWARD;
        }
        break;

    case JAM_FORWARD:
        PID_Control_Smis(current_angle,
                         jam_target_angle,
                         &Pluck_Place_PID,
                         shoot.Pluck_motor->Data.SpeedFilter);
        PID_Control(shoot.Pluck_motor->Data.SpeedFilter,
                    Pluck_Place_PID.pid_out,
                    &Pluck_Speed_PID);
        tx[0] = (int16_t)limit(Pluck_Speed_PID.pid_out, 15000.0f, -15000.0f);
        DJI_Motor_CAN_TxMessage(shoot.Pluck_motor, tx);

        if (fabsf(current_angle - jam_target_angle) < 2.0f)
        {
            jam_state = JAM_IDLE;
            jam_ticks = 0U;
            robot_cmd.shoot_mode = SHOOT_READY;
        }
        break;
    }
}
uint8_t CheckFricSpeedReady(void)
{
    if (shoot.fric_motor_LF == NULL || shoot.fric_motor_RF == NULL ||
        shoot.fric_motor_LB == NULL || shoot.fric_motor_RB == NULL)
    {
        return 0U;
    }
    if (fabsf(shoot.fric_motor_LF->Data.SpeedFilter) < SHOOT_SPEED_Front ||
        fabsf(shoot.fric_motor_RF->Data.SpeedFilter) < SHOOT_SPEED_Front ||
        fabsf(shoot.fric_motor_LB->Data.SpeedFilter) < SHOOT_SPEED_Behind ||
        fabsf(shoot.fric_motor_RB->Data.SpeedFilter) < SHOOT_SPEED_Behind)
    {
        return 0U;
    }
    return 1U;
}