/**
 * @file    Gimbal.c
 * @brief   云台执行层 - 电机PID控制
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

#define POWERON_CENTER_STABLE_TICKS       200U
#define POWERON_CENTER_YAW_ERR_THRESHOLD  120.0f
#define POWERON_CENTER_PITCH_ERR_THRESHOLD 80.0f
#define IMU_OFFLINE_DEBOUNCE_TICKS        100U
#define GIMBAL_STOP_PID_MODE              PID_GYRO
#define GIMBAL_STOP_SPEED_DEADBAND        2.0f
#define GIMBAL_STOP_OUT_MAX               14000.0f

uint8_t power_on_center_flag = 0U;
static uint16_t gimbal_poweron_center_stable_cnt = 0U;
static uint16_t imu_offline_debounce = 0U;
static uint8_t last_used_feedback = 1U; /* 1=Gyro, 0=Mech */

typedef enum
{
    PID_GYRO = 0,
    PID_MECH = 1,
    PID_CENTER = 2,
    PID_AIM = 3,
} GimbalPidMode_e;

/* 位置环：计算底盘旋转速度参考 */
static PID_Smis Chassis_Rotate_PIDS = {
    .Kp = 1.0f,
    .Ki = 0,
    .Kd = -8.0f,
    .interlimit = 1500,
    .outlimit = 2000 * PI,
    .DeadBand = 0,
    .inter_threLow = 500,
    .inter_threUp = 2000,
};

/* 速度环：平滑底盘旋转输出 */
static PID Chassis_Rotate_PID = {
    .Kp = 1.0f,
    .Ki = 0.0f,
    .Kd = 0,
    .interlimit = 4500 * PI,
    .outlimit = 4000 * PI,
    .DeadBand = 0,
    .inter_threLow = 10 * PI,
    .inter_threUp = 4000 * PI,
};

/* 小陀螺Yaw前馈 */
static FeedForward_Typedef GimbalYaw_FF = {
    .K1 = 0.0f,
    .K2 = 20.0f,
    .K3 = 0.0f,
    .OutMax = 3000,
    .DWT_CNT = 0,
};

/* Yaw轴位置环 */
static PID_Smis Yaw_Pos_PID[4] = {
    {.Kp = 9.0f, .Ki = 0.0f, .Kd = -35.0f, .interlimit = 3000, .outlimit = 20000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 22.0f, .Ki = 0.0f, .Kd = -20.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 10.0f, .Ki = 0.0f, .Kd = -20.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 14.0f, .Ki = 0.1f, .Kd = -37.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
};

/* Pitch轴位置环 */
static PID_Smis Pitch_Pos_PID[4] = {
    {.Kp = -15.0f, .Ki = 0.0f, .Kd = 10.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 27.0f, .Ki = 0.0f, .Kd = -10.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.01f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = 30.0f, .Ki = 0.0f, .Kd = -25.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
    {.Kp = -240.0f, .Ki = 0.0f, .Kd = 7.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 0.1f, .inter_threLow = 50, .inter_threUp = 500},
};

/* Yaw轴速度环 */
static PID Yaw_Speed_PID[4] = {
    {.Kp = 80.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 2.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 2.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 110.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
};

/* Pitch轴速度环 */
static PID Pitch_Speed_PID[4] = {
    {.Kp = 80.0f, .Ki = 0.0f, .Kd = 10.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 90.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 1.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 2.5f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
    {.Kp = 12.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 24000, .DeadBand = 2.0f, .inter_threLow = 500, .inter_threUp = 1000},
};

/* ==================== 内部函数声明 ==================== */
static void Gimbal_Disable(void);
static void Gimbal_Stop(void);
static void ResetAllGimbalPidIout(void);
static void SyncGimbalReference(void);
static uint8_t DebounceFeedbackSelection(uint8_t desired_feedback);
static void Pitch_Limit(uint8_t use_gyro_feedback);
static void Chassis_Follow_Calc(void);
static int32_t gimbal_center(int32_t yaw_now);
static int32_t front_center_target(int32_t yaw_now);
static void GetFeedbackData(uint8_t use_gyro_feedback,
                            float *yaw_fdb,
                            float *pitch_fdb,
                            float *yaw_rate,
                            float *pitch_rate,
                            float *yaw_ref,
                            float *pitch_ref);

static void SyncGimbalReference(void)
{
    if (gimbal.yaw_motor != NULL && gimbal.pitch_motor != NULL)
    {
        robot_cmd.Mech_Yaw = gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
        robot_cmd.Mech_Pitch = gimbal.pitch_motor->Data.DJI_data.MechanicalAngle;
    }

    robot_cmd.Gyro_Yaw = INS_Info.Yaw_TolAngle;
    robot_cmd.Gyro_Pitch = INS_Info.Pitch_Angle;
}

static void ResetAllGimbalPidIout(void)
{
    for (uint8_t i = 0; i <= PID_AIM; i++)
    {
        PID_IoutReset(&Yaw_Speed_PID[i]);
        PID_IoutReset(&Pitch_Speed_PID[i]);
    }
}

static uint8_t DebounceFeedbackSelection(uint8_t desired_feedback)
{
    if (desired_feedback == last_used_feedback)
    {
        imu_offline_debounce = 0U;
        return desired_feedback;
    }

    if (desired_feedback)
    {
        last_used_feedback = 1U;
        imu_offline_debounce = 0U;
        SyncGimbalReference();
        return 1U;
    }

    imu_offline_debounce++;
    if (imu_offline_debounce >= IMU_OFFLINE_DEBOUNCE_TICKS)
    {
        last_used_feedback = 0U;
        imu_offline_debounce = 0U;
        SyncGimbalReference();
        return 0U;
    }

    return 1U;
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
    SyncGimbalReference();

    power_on_center_flag = 0U;
    gimbal_poweron_center_stable_cnt = 0U;
    imu_offline_debounce = 0U;
    last_used_feedback = 1U;
}

/* ==================== 云台主控制函数==================== */
void Gimbal_Control(void)
{
    int16_t can_send[4] = {0};
    float yaw_fdb = 0.0f;
    float pitch_fdb = 0.0f;
    float yaw_rate = 0.0f;
    float pitch_rate = 0.0f;
    float yaw_ref = 0.0f;
    float pitch_ref = 0.0f;
    float yaw_speed_ref = 0.0f;
    uint8_t imu_online_now = INS_Info.INS_online_flag;
    uint8_t use_gyro_feedback = 0U;
    GimbalPidMode_e pid_mode = PID_MECH;

    if (gimbal.yaw_motor == NULL || gimbal.pitch_motor == NULL)
    {
        return;
    }

    /* 阶段1: 停止/失能状态提前处理 */
    switch (robot_cmd.gimbal_mode)
    {
    case GIMBAL_DISABLED:
        Gimbal_Disable();
        return;
    case GIMBAL_STOP:
        Gimbal_Stop();
        return;
    default:
        break;
    }

    /* 阶段2: 上电归中（强制机械反馈 + CENTER参数） */
    if (power_on_center_flag == 0U)
    {
        robot_cmd.Mech_Yaw = front_center_target(gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle);
        robot_cmd.Mech_Pitch = (uint16_t)limit((float)gimbal.pitch_motor->Data.DJI_data.MechanicalAngle,
                                               (float)MCH_UP_limit,
                                               (float)MCH_DOWN_limit);

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
            SyncGimbalReference();
        }

        return;
    }

    /* 阶段3: 反馈源与PID组选择 */
    if (robot_cmd.gimbal_mode == GIMBAL_RUNNING_AIM)
    {
        if (imu_online_now)
        {
            use_gyro_feedback = 1U;
            pid_mode = PID_AIM;
            last_used_feedback = 1U;
            imu_offline_debounce = 0U;
        }
        else
        {
            robot_cmd.gimbal_mode = (robot_cmd.Control_mode == CONTROL_KEY_MOUSE)
                                        ? GIMBAL_RUNNING_NORMAL
                                        : GIMBAL_RUNNING_FOLLOW;
            SyncGimbalReference();
            use_gyro_feedback = 0U;
            pid_mode = PID_MECH;
            last_used_feedback = 0U;
            imu_offline_debounce = 0U;
        }
    }
    else
    {
        use_gyro_feedback = DebounceFeedbackSelection(imu_online_now ? 1U : 0U);
        pid_mode = use_gyro_feedback ? PID_GYRO : PID_MECH;
    }

    /* 阶段4: 速度控制通道 */
    if (robot_cmd.use_velocity_control)
    {
        GetFeedbackData(use_gyro_feedback, &yaw_fdb, &pitch_fdb, &yaw_rate, &pitch_rate, &yaw_ref, &pitch_ref);

        PID_Control(yaw_rate, robot_cmd.gimbal_yaw_velocity_target, &Yaw_Speed_PID[pid_mode]);
        PID_Control(pitch_rate, robot_cmd.gimbal_pitch_velocity_target, &Pitch_Speed_PID[pid_mode]);

        can_send[0] = (int16_t)Pitch_Speed_PID[pid_mode].pid_out;
        can_send[1] = (int16_t)Yaw_Speed_PID[pid_mode].pid_out;
        DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);
        return;
    }

    /* 阶段5: 位置控制通道 */
    switch (robot_cmd.gimbal_mode)
    {
    case GIMBAL_RUNNING_FOLLOW:
        Chassis_Follow_Calc();
        Pitch_Limit(use_gyro_feedback);
        GetFeedbackData(use_gyro_feedback, &yaw_fdb, &pitch_fdb, &yaw_rate, &pitch_rate, &yaw_ref, &pitch_ref);

        if (use_gyro_feedback)
        {
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

    case GIMBAL_RUNNING_NORMAL:
    case GIMBAL_RUNNING_AIM:
        Pitch_Limit(use_gyro_feedback);
        GetFeedbackData(use_gyro_feedback, &yaw_fdb, &pitch_fdb, &yaw_rate, &pitch_rate, &yaw_ref, &pitch_ref);

        if (robot_cmd.gimbal_mode == GIMBAL_RUNNING_AIM && use_gyro_feedback)
        {
            robot_cmd.rotate_feedforward = can_comm_instance.can_comm_rx_data.chassis_gyro / 200.0f;
            FeedForward_Calc(&GimbalYaw_FF, robot_cmd.rotate_feedforward);
        }

        PID_Control_Smis(yaw_fdb, yaw_ref, &Yaw_Pos_PID[pid_mode], yaw_rate);
        yaw_speed_ref = Yaw_Pos_PID[pid_mode].pid_out +
                        ((robot_cmd.gimbal_mode == GIMBAL_RUNNING_AIM && use_gyro_feedback) ? GimbalYaw_FF.Out : 0.0f);
        PID_Control(yaw_rate, yaw_speed_ref, &Yaw_Speed_PID[pid_mode]);

        PID_Control_Smis(pitch_fdb, pitch_ref, &Pitch_Pos_PID[pid_mode], pitch_rate);
        PID_Control(pitch_rate, Pitch_Pos_PID[pid_mode].pid_out, &Pitch_Speed_PID[pid_mode]);

        can_send[0] = (int16_t)Pitch_Speed_PID[pid_mode].pid_out;
        can_send[1] = (int16_t)Yaw_Speed_PID[pid_mode].pid_out;
        DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);
        return;

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
    ResetAllGimbalPidIout();
    SyncGimbalReference();
}

static void Gimbal_Stop(void)
{
    int16_t can_send[4] = {0};
    float yaw_speed = gimbal.yaw_motor->Data.DJI_data.SpeedFilter;
    float pitch_speed = gimbal.pitch_motor->Data.DJI_data.SpeedFilter;

    // 停止态语义为急停：速度环目标直接置0，由PID闭环主动制动。
    PID_Control(yaw_speed, 0.0f, &Yaw_Speed_PID[GIMBAL_STOP_PID_MODE]);
    PID_Control(pitch_speed, 0.0f, &Pitch_Speed_PID[GIMBAL_STOP_PID_MODE]);

    if (fabsf(yaw_speed) <= GIMBAL_STOP_SPEED_DEADBAND)
    {
        Yaw_Speed_PID[GIMBAL_STOP_PID_MODE].pid_out = 0.0f;
    }
    if (fabsf(pitch_speed) <= GIMBAL_STOP_SPEED_DEADBAND)
    {
        Pitch_Speed_PID[GIMBAL_STOP_PID_MODE].pid_out = 0.0f;
    }

    can_send[0] = (int16_t)limit(Pitch_Speed_PID[GIMBAL_STOP_PID_MODE].pid_out,
                                 GIMBAL_STOP_OUT_MAX,
                                 -GIMBAL_STOP_OUT_MAX);
    can_send[1] = (int16_t)limit(Yaw_Speed_PID[GIMBAL_STOP_PID_MODE].pid_out,
                                 GIMBAL_STOP_OUT_MAX,
                                 -GIMBAL_STOP_OUT_MAX);

    DM_Motor_DJI_CAN_TxMessage(gimbal.yaw_motor, can_send);
    ResetAllGimbalPidIout();
    SyncGimbalReference();
}

/* ==================== 云台归中计算 ==================== */
static int32_t gimbal_center(int32_t yaw_now)
{
    if (robot_cmd.Mid_mode == MID_FRONT)
    {
        int32_t current_turn_base = (yaw_now / 8192) * 8192;
        return current_turn_base + Yaw_Mid_Front;
    }

    if (robot_cmd.Mid_mode == MID_BACK)
    {
        int32_t current_turn_base = (yaw_now / 8192) * 8192;
        return current_turn_base + Yaw_Mid_Back;
    }

    {
        const uint16_t alignment_targets[4] = {Yaw_Mid_Front, Yaw_Mid_Left, Yaw_Mid_Back, Yaw_Mid_Right};
        uint16_t yaw_in_turn = (uint16_t)((yaw_now % 8192 + 8192) % 8192);
        int16_t best_offset = 8192;

        for (uint8_t i = 0; i < 4; i++)
        {
            int16_t candidate_offset = (int16_t)alignment_targets[i] - (int16_t)yaw_in_turn;

            if (candidate_offset > 4096)
            {
                candidate_offset -= 8192;
            }
            if (candidate_offset < -4096)
            {
                candidate_offset += 8192;
            }

            if (abs(candidate_offset) < abs(best_offset))
            {
                best_offset = candidate_offset;
            }
        }

        return yaw_now + best_offset;
    }
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

/* ==================== Pitch轴限位==================== */
static void Pitch_Limit(uint8_t use_gyro_feedback)
{
    if (use_gyro_feedback)
    {
        robot_cmd.Gyro_Pitch = limit(robot_cmd.Gyro_Pitch, IMU_UP_limit, IMU_DOWN_limit);
    }
    else
    {
        robot_cmd.Mech_Pitch = (uint16_t)limit((float)robot_cmd.Mech_Pitch,
                                               (float)MCH_UP_limit,
                                               (float)MCH_DOWN_limit);
    }
}

/* ==================== 底盘跟随计算 ==================== */
static void Chassis_Follow_Calc(void)
{
    int32_t yaw_now = gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
    int32_t target = gimbal_center(yaw_now);

    robot_cmd.rotate_feedforward = can_comm_instance.can_comm_rx_data.chassis_gyro / 200.0f;

    PID_Control_Smis((float)yaw_now, (float)target, &Chassis_Rotate_PIDS, robot_cmd.rotate_feedforward);
    PID_Control(robot_cmd.rotate_feedforward, Chassis_Rotate_PIDS.pid_out, &Chassis_Rotate_PID);

    robot_cmd.rotate_speed = limit(Chassis_Rotate_PID.pid_out,
                                   (float)rotate_speed_MAX,
                                   -(float)rotate_speed_MAX);
}

/* ==================== 反馈数据获取 ==================== */
static void GetFeedbackData(uint8_t use_gyro_feedback,
                            float *yaw_fdb,
                            float *pitch_fdb,
                            float *yaw_rate,
                            float *pitch_rate,
                            float *yaw_ref,
                            float *pitch_ref)
{
    if (use_gyro_feedback == 0U)
    {
        *yaw_fdb = (float)gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
        *pitch_fdb = (float)gimbal.pitch_motor->Data.DJI_data.MechanicalAngle;
        *yaw_rate = gimbal.yaw_motor->Data.DJI_data.SpeedFilter;
        *pitch_rate = gimbal.pitch_motor->Data.DJI_data.SpeedFilter;
        *yaw_ref = (float)robot_cmd.Mech_Yaw;
        *pitch_ref = (float)robot_cmd.Mech_Pitch;
    }
    else
    {
        *yaw_fdb = INS_Info.Yaw_TolAngle;
        *pitch_fdb = INS_Info.Pitch_Angle;
        *yaw_rate = INS_Info.Yaw_Gyro;
        *pitch_rate = INS_Info.Pitch_Gyro;
        *yaw_ref = robot_cmd.Gyro_Yaw;
        *pitch_ref = robot_cmd.Gyro_Pitch;
    }
}
