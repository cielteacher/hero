/**
 * @file    Gimbal_Cmd.c
 * @brief   云台决策层 - 模式决策、目标更新、板间通信
 */
#include "Gimbal_Cmd.h"
#include "arm_math.h"
#include "bsp_fdcan.h"
#include "robot.h"
#include "slope.h"
#include <math.h>

/* ==================== 全局/静态变量 ==================== */
Robot_ctrl_cmd_t robot_cmd;
static Gimbal_board_send_t Gimbal_To_Chassis;
static Gimbal_action_t Gimabl_To_Chassis_Action;
static Slope_s slope_X, slope_Y, slope_Omega;
static CANInstance *can_comm2;
extern uint8_t power_on_center_flag;

/* ==================== 内部函数声明 ==================== */
static void RCUpdate(uint8_t imu_online_now);
static void KMUpdate(uint8_t imu_online_now);
static void Gimbal_Target_Update(uint8_t imu_online_now);
static void Chassis_Comm_Update(uint8_t gimbal_online_now, uint8_t shoot_online_now);
static void Rotate_Speed_Set(void);
static void ApplySafetyInterlocks(uint8_t gimbal_online_now,
                                  uint8_t shoot_online_now,
                                  uint8_t chassis_online_now,
                                  uint8_t imu_online_now);
static void SyncCmdReferenceToFeedback(void);
static void ClearGimbalVelocityControlState(void);
static void ForceRobotDisabledState(void);
static void DowngradeAimModeWhenImuOffline(void);
static float SlewLimit(float target, float current, float step);

/**
 * @brief 检查云台模块在线状态
 */
static uint8_t IsGimbalModuleOnline(void)
{
    if (gimbal.pitch_motor == NULL || gimbal.yaw_motor == NULL)
    {
        return 0U;
    }

    return (gimbal.pitch_motor->dm_motor_online_flag && gimbal.yaw_motor->dm_motor_online_flag) ? 1U : 0U;
}

/**
 * @brief 检查发射模块在线状态
 */
static uint8_t IsShootModuleOnline(void)
{
    if (shoot.Pluck_motor == NULL || shoot.fric_motor_1 == NULL || shoot.fric_motor_2 == NULL ||
        shoot.fric_motor_3 == NULL || shoot.fric_motor_4 == NULL)
    {
        return 0U;
    }

    return (shoot.Pluck_motor->dji_motor_online_flag &&
            shoot.fric_motor_1->dji_motor_online_flag &&
            shoot.fric_motor_2->dji_motor_online_flag &&
            shoot.fric_motor_3->dji_motor_online_flag &&
            shoot.fric_motor_4->dji_motor_online_flag)
               ? 1U
               : 0U;
}

static uint8_t VisionCanAutoAim(void)
{
    return (MiniPC_instance.MiniPC_Online_Flag &&
        MiniPC_instance.receive_data.data.dis <= 0.2f)
           ? 1U
           : 0U;
}

static void ClearGimbalVelocityControlState(void)
{
    robot_cmd.use_velocity_control = 0U;
    robot_cmd.gimbal_yaw_velocity_target = 0.0f;
    robot_cmd.gimbal_pitch_velocity_target = 0.0f;
    robot_cmd.last_velocity_mode = 0U;
}

static void ForceRobotDisabledState(void)
{
    robot_cmd.robot_state = ROBOT_STOP;
    robot_cmd.gimbal_mode = GIMBAL_DISABLED;
    robot_cmd.chassis_mode = CHASSIS_DISABLED;
    robot_cmd.shoot_mode = SHOOT_DISABLED;
    robot_cmd.Speed_Up_flag = false;
    ClearGimbalVelocityControlState();
}

static void DowngradeAimModeWhenImuOffline(void)
{
    robot_cmd.gimbal_mode = (robot_cmd.Control_mode == CONTROL_KEY_MOUSE)
                                ? GIMBAL_RUNNING_NORMAL
                                : GIMBAL_RUNNING_FOLLOW;

    if (robot_cmd.shoot_mode == SHOOT_AIM)
    {
        robot_cmd.shoot_mode = SHOOT_READY;
    }
}

static float SlewLimit(float target, float current, float step)
{
    float delta = target - current;

    if (delta > step)
    {
        delta = step;
    }
    else if (delta < -step)
    {
        delta = -step;
    }

    return current + delta;
}

static void SyncCmdReferenceToFeedback(void)
{
    if (gimbal.yaw_motor == NULL || gimbal.pitch_motor == NULL)
    {
        return;
    }

    robot_cmd.Mech_Yaw = gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
    robot_cmd.Mech_Pitch = gimbal.pitch_motor->Data.DJI_data.MechanicalAngle;
    robot_cmd.Gyro_Yaw = INS_Info.Yaw_TolAngle;
    robot_cmd.Gyro_Pitch = INS_Info.Pitch_Angle;
}

static void ApplySafetyInterlocks(uint8_t gimbal_online_now,
                                  uint8_t shoot_online_now,
                                  uint8_t chassis_online_now,
                                  uint8_t imu_online_now)
{
    if (!gimbal_online_now)
    {
        robot_cmd.gimbal_mode = GIMBAL_DISABLED;
        robot_cmd.shoot_mode = SHOOT_DISABLED;
    }

    if (!shoot_online_now)
    {
        robot_cmd.shoot_mode = SHOOT_DISABLED;
    }

    if (!chassis_online_now)
    {
        robot_cmd.chassis_mode = CHASSIS_DISABLED;
    }

    if ((robot_cmd.gimbal_mode == GIMBAL_RUNNING_AIM) && !imu_online_now)
    {
        DowngradeAimModeWhenImuOffline();
    }
}

/* ==================== 机器人初始化 ==================== */
void Robot_Init(void)
{
    CAN_Init_Config_s can_comm2_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x120,
        .rx_id = 0,
        .rx_len = 8,
    };

    Slope_Init(&slope_X, 0.006f, 0.08f, SLOPE_FIRST_REAL);
    Slope_Init(&slope_Y, 0.006f, 0.08f, SLOPE_FIRST_REAL);
    Slope_Init(&slope_Omega, 3.0f * PI / 1000.0f, 30.0f * PI / 1000.0f, SLOPE_FIRST_REAL);

    can_comm2 = CANRegister(&can_comm2_config);

    robot_cmd.robot_state = ROBOT_STOP;
    robot_cmd.gimbal_mode = GIMBAL_STOP;
    robot_cmd.shoot_mode = SHOOT_STOP;
    robot_cmd.chassis_mode = CHASSIS_STOP;
    robot_cmd.Control_mode = CONTROL_REMOTE;
    robot_cmd.Mid_mode = MID_FRONT;
    robot_cmd.Speed_Up_flag = false;
    robot_cmd.Unlimit_flag = false;
    robot_cmd.rotate_feedforward = 0.0f;
    robot_cmd.rotate_speed = 0.0f;
    ClearGimbalVelocityControlState();
    robot_cmd.last_control_mode = robot_cmd.Control_mode;
    robot_cmd.last_gimbal_mode = robot_cmd.gimbal_mode;

    Gimbal_Init();
    Shoot_Init();
    Can_Comm_Init();
}

/* ==================== 机器人状态更新 ==================== */
void Robot_Update(void)
{
    uint8_t remote_online = DR16_instance.dr16_online_flag;
    uint8_t gimbal_online_now = IsGimbalModuleOnline();
    uint8_t shoot_online_now = IsShootModuleOnline();
    uint8_t chassis_online_now = can_comm_instance.can_comm_online_flag;
    uint8_t imu_online_now = INS_Info.INS_online_flag;

    if (!remote_online)
    {
        ForceRobotDisabledState();
        Chassis_Comm_Update(gimbal_online_now, shoot_online_now);
        return;
    }

    robot_cmd.robot_state = ROBOT_RUNNING;

    switch (DR16_instance.control_data.input_mode)
    {
    case REMOTE_INPUT:
        robot_cmd.Control_mode = CONTROL_REMOTE;
        RCUpdate(imu_online_now);
        break;

    case KEY_MOUSE_INPUT:
        robot_cmd.Control_mode = CONTROL_KEY_MOUSE;
        KMUpdate(imu_online_now);
        break;

    default:
        ForceRobotDisabledState();
        Chassis_Comm_Update(gimbal_online_now, shoot_online_now);
        return;
    }

    ApplySafetyInterlocks(gimbal_online_now, shoot_online_now, chassis_online_now, imu_online_now);

    if (robot_cmd.gimbal_mode != robot_cmd.last_gimbal_mode ||
        robot_cmd.Control_mode != robot_cmd.last_control_mode)
    {
        SyncCmdReferenceToFeedback();
    }

    Gimbal_Target_Update(imu_online_now);

    robot_cmd.last_gimbal_mode = robot_cmd.gimbal_mode;
    robot_cmd.last_control_mode = robot_cmd.Control_mode;

    Chassis_Comm_Update(gimbal_online_now, shoot_online_now);
}

/* ==================== 遥控器模式更新 ==================== */
static void RCUpdate(uint8_t imu_online_now)
{
    robot_cmd.gimbal_mode = GIMBAL_RUNNING_FOLLOW;
    robot_cmd.chassis_mode = CHASSIS_RUNNING_FOLLOW;
    robot_cmd.Mid_mode = MID_FRONT;
    robot_cmd.Speed_Up_flag = false;
    robot_cmd.shoot_mode = SHOOT_READY_NOFRIC;

    if (DR16_instance.control_data.fn_1)
    {
        robot_cmd.shoot_mode = SHOOT_READY;

        if (DR16_instance.control_data.fn_2)
        {
            robot_cmd.gimbal_mode = GIMBAL_RUNNING_AIM;
            robot_cmd.chassis_mode = CHASSIS_RUNNING_NORMAL;
            robot_cmd.shoot_mode = (imu_online_now && VisionCanAutoAim()) ? SHOOT_AIM : SHOOT_READY;
        }

        if (DR16_instance.control_data.trigger)
        {
            robot_cmd.shoot_mode = SHOOT_FIRE;
        }
    }
}

/* ==================== 键鼠模式更新 ==================== */
static void KMUpdate(uint8_t imu_online_now)
{
    static uint8_t fric_toggle = 0U;

    robot_cmd.gimbal_mode = GIMBAL_RUNNING_FOLLOW;
    robot_cmd.chassis_mode = CHASSIS_RUNNING_FOLLOW;
    robot_cmd.Mid_mode = MID_FRONT;
    robot_cmd.Speed_Up_flag = false;
    robot_cmd.rotate_speed = 0.0f;
    robot_cmd.shoot_mode = fric_toggle ? SHOOT_READY : SHOOT_READY_NOFRIC;

    if (DR16_instance.control_data.keys.bits.Ctrl &&
        DR16_instance.control_data.keys.bits.R &&
        !DR16_instance.control_data.last_keys.bits.R)
    {
        NVIC_SystemReset();
    }

    if (DR16_instance.control_data.keys.bits.Ctrl &&
        DR16_instance.control_data.keys.bits.G &&
        !DR16_instance.control_data.last_keys.bits.G)
    {
        robot_cmd.Unlimit_flag ^= 1U;
    }

    if (DR16_instance.control_data.keys.bits.B &&
        !DR16_instance.control_data.last_keys.bits.B)
    {
        fric_toggle ^= 1U;
    }

    robot_cmd.Speed_Up_flag = DR16_instance.control_data.keys.bits.Shift ? true : false;

    if (DR16_instance.control_data.keys.bits.V &&
        !DR16_instance.control_data.last_keys.bits.V)
    {
        power_on_center_flag = 0U;
    }

    if (DR16_instance.control_data.keys.bits.F &&
        !DR16_instance.control_data.last_keys.bits.F)
    {
        robot_cmd.gimbal_mode = GIMBAL_RUNNING_FOLLOW;
        robot_cmd.chassis_mode = CHASSIS_RUNNING_FOLLOW;
        robot_cmd.Mid_mode = MID_NONE;
        if (imu_online_now)
        {
            robot_cmd.Gyro_Yaw += 180.0f;
        }
        else
        {
            robot_cmd.Mech_Yaw += 4096;
        }
    }

    if (DR16_instance.control_data.keys.bits.E)
    {
        robot_cmd.gimbal_mode = GIMBAL_RUNNING_NORMAL;
        robot_cmd.chassis_mode = CHASSIS_RUNNING_NORMAL;
    }

    if (DR16_instance.control_data.keys.bits.Q)
    {
        robot_cmd.gimbal_mode = GIMBAL_RUNNING_NORMAL;
        robot_cmd.chassis_mode = CHASSIS_RUNNING_SPIN;
    }

    if (DR16_instance.control_data.long_press_r)
    {
        if (imu_online_now && VisionCanAutoAim())
        {
            robot_cmd.gimbal_mode = GIMBAL_RUNNING_AIM;
            robot_cmd.shoot_mode = SHOOT_AIM;
        }
        else
        {
            robot_cmd.gimbal_mode = GIMBAL_RUNNING_FOLLOW;
            robot_cmd.shoot_mode = fric_toggle ? SHOOT_READY : SHOOT_READY_NOFRIC;
        }
    }

    if (DR16_instance.control_data.press_l &&
        !DR16_instance.control_data.last_mouse_press_l &&
        fric_toggle)
    {
        robot_cmd.shoot_mode = SHOOT_FIRE;
    }
}

/* ==================== 云台目标更新 ==================== */
static void Gimbal_Target_Update(uint8_t imu_online_now)
{
    static float yaw_velocity_cmd = 0.0f;
    static float pitch_velocity_cmd = 0.0f;
    float yaw_speed_target = 0.0f;
    float pitch_speed_target = 0.0f;

    if (robot_cmd.gimbal_mode == GIMBAL_DISABLED || robot_cmd.gimbal_mode == GIMBAL_STOP)
    {
        ClearGimbalVelocityControlState();
        yaw_velocity_cmd = 0.0f;
        pitch_velocity_cmd = 0.0f;
        return;
    }

    if (robot_cmd.gimbal_mode == GIMBAL_RUNNING_AIM)
    {
        robot_cmd.use_velocity_control = 0U;

        if (imu_online_now && VisionCanAutoAim())
        {
            float vision_yaw = MiniPC_instance.receive_data.data.Ref_yaw;
            float yaw_diff = vision_yaw - INS_Info.Yaw_Angle;
            int32_t round_offset = 0;

            if (yaw_diff < -240.0f)
            {
                round_offset = 1;
            }
            else if (yaw_diff > 240.0f)
            {
                round_offset = -1;
            }

            robot_cmd.Gyro_Yaw = (INS_Info.YawRoundCount + round_offset) * 360.0f + vision_yaw;
            robot_cmd.Gyro_Pitch = INS_Info.Pitch_Angle + MiniPC_instance.receive_data.data.Ref_pitch;
            yaw_velocity_cmd = 0.0f;
            pitch_velocity_cmd = 0.0f;
            robot_cmd.last_velocity_mode = 0U;
            return;
        }

        DowngradeAimModeWhenImuOffline();
    }

    robot_cmd.use_velocity_control = 1U;

    if (!robot_cmd.last_velocity_mode)
    {
        if (imu_online_now)
        {
            yaw_velocity_cmd = INS_Info.Yaw_Gyro;
            pitch_velocity_cmd = INS_Info.Pitch_Gyro;
        }
        else if (gimbal.yaw_motor != NULL && gimbal.pitch_motor != NULL)
        {
            yaw_velocity_cmd = gimbal.yaw_motor->Data.DJI_data.SpeedFilter;
            pitch_velocity_cmd = gimbal.pitch_motor->Data.DJI_data.SpeedFilter;
        }
        else
        {
            yaw_velocity_cmd = 0.0f;
            pitch_velocity_cmd = 0.0f;
        }
    }

    if (robot_cmd.Control_mode == CONTROL_KEY_MOUSE)
    {
        const float mouse_speed_yaw_gain = 320.0f;
        const float mouse_speed_pitch_gain = 230.0f;
        const float mouse_delta_yaw_gain = 140.0f;
        const float mouse_delta_pitch_gain = 95.0f;
        float yaw_speed_from_mouse = ((float)DR16_instance.control_data.x / (float)MOUSE_MAX) * mouse_speed_yaw_gain;
        float pitch_speed_from_mouse = -((float)DR16_instance.control_data.y / (float)MOUSE_MAX) * mouse_speed_pitch_gain;
        float yaw_speed_from_delta = DR16_instance.control_data.Postion_x * mouse_delta_yaw_gain;
        float pitch_speed_from_delta = -DR16_instance.control_data.Postion_y * mouse_delta_pitch_gain;

        yaw_speed_target = 0.6f * yaw_speed_from_mouse + 0.4f * yaw_speed_from_delta;
        pitch_speed_target = 0.6f * pitch_speed_from_mouse + 0.4f * pitch_speed_from_delta;
    }
    else
    {
        yaw_speed_target = DR16_instance.control_data.Normalize_ch0 * 240.0f;
        pitch_speed_target = DR16_instance.control_data.Normalize_ch1 * 180.0f;
    }

    deadline_limit(yaw_speed_target, 1.0f);
    deadline_limit(pitch_speed_target, 1.0f);

    yaw_speed_target = limit(yaw_speed_target, 300.0f, -300.0f);
    pitch_speed_target = limit(pitch_speed_target, 220.0f, -220.0f);

    yaw_velocity_cmd = SlewLimit(yaw_speed_target, yaw_velocity_cmd, 12.0f);
    pitch_velocity_cmd = SlewLimit(pitch_speed_target, pitch_velocity_cmd, 10.0f);

    robot_cmd.gimbal_yaw_velocity_target = yaw_velocity_cmd;
    robot_cmd.gimbal_pitch_velocity_target = pitch_velocity_cmd;

    if (imu_online_now)
    {
        robot_cmd.Gyro_Yaw = INS_Info.Yaw_TolAngle;
        robot_cmd.Gyro_Pitch = INS_Info.Pitch_Angle;
    }
    else if (gimbal.yaw_motor != NULL && gimbal.pitch_motor != NULL)
    {
        robot_cmd.Mech_Yaw = gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;
        robot_cmd.Mech_Pitch = (uint16_t)limit(
            (float)gimbal.pitch_motor->Data.DJI_data.MechanicalAngle,
            (float)MCH_UP_limit,
            (float)MCH_DOWN_limit);
    }

    robot_cmd.last_velocity_mode = 1U;
}

/* ==================== 小陀螺速度设置 ==================== */
static void Rotate_Speed_Set(void)
{
    const float sector_centers[] = {865.0f, 2755.0f, 4805.0f, 6855.0f, 8036.0f};
    float yaw_single;
    float min_offset = 8192.0f;

    if (gimbal.yaw_motor == NULL)
    {
        robot_cmd.rotate_speed = 0.0f;
        return;
    }

    yaw_single = fmodf((float)gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle, 8192.0f);
    if (yaw_single < 0.0f)
    {
        yaw_single += 8192.0f;
    }

    for (uint8_t i = 0; i < 5U; i++)
    {
        float offset = fabsf(sector_centers[i] - yaw_single);
        if (offset > 4096.0f)
        {
            offset = 8192.0f - offset;
        }
        if (offset < min_offset)
        {
            min_offset = offset;
        }
    }

    {
        float angle_ratio = min_offset / 2050.57f;
        float target_speed = 700.0f * PI + 800.0f * PI * arm_cos_f32(angle_ratio * PI);
        robot_cmd.rotate_speed = limit(target_speed, (float)rotate_speed_MAX, -(float)rotate_speed_MAX);
    }
}

/* ==================== 底盘通信更新 ==================== */
static void Chassis_Comm_Update(uint8_t gimbal_online_now, uint8_t shoot_online_now)
{
    static const float LEVEL_GAIN = 2.0f;
    static Chassis_speed_s slopeningV = {0};
    Chassis_speed_s planningV = {0};

    if (can_comm_instance.can_comm_online_flag == 0U || can_comm_instance.can_comm == NULL || can_comm2 == NULL)
    {
        return;
    }

    Gimabl_To_Chassis_Action.Gimbal_Online = gimbal_online_now;
    Gimabl_To_Chassis_Action.Shoot_Online = shoot_online_now;
    Gimabl_To_Chassis_Action.Vision_Online = MiniPC_instance.MiniPC_Online_Flag;
    Gimabl_To_Chassis_Action.Move_Status = (uint8_t)robot_cmd.chassis_mode;
    Gimabl_To_Chassis_Action.Shoot_Mode = (uint8_t)robot_cmd.shoot_mode;

    Gimbal_To_Chassis.Unlimit_flag = robot_cmd.Unlimit_flag ? 1U : 0U;

    if (robot_cmd.robot_state == ROBOT_STOP || robot_cmd.chassis_mode == CHASSIS_DISABLED)
    {
        Gimbal_To_Chassis.vx = 0;
        Gimbal_To_Chassis.vy = 0;
        Gimbal_To_Chassis.rotate = 0;
        Gimbal_To_Chassis.Unlimit_flag = 0U;
        Gimbal_To_Chassis.Close_flag = 1U;
    }
    else
    {
        Gimbal_To_Chassis.Close_flag = 0U;

        if (robot_cmd.Control_mode == CONTROL_REMOTE)
        {
            planningV.Y = DR16_instance.control_data.Normalize_ch2 * LEVEL_GAIN * 2.0f;
            planningV.X = DR16_instance.control_data.Normalize_ch3 * LEVEL_GAIN;
            planningV.Omega = DR16_instance.control_data.Normalize_ch1 * LEVEL_GAIN;

            if (fabsf(DR16_instance.control_data.Normalize_wheel) > 0.08f)
            {
                robot_cmd.chassis_mode = CHASSIS_RUNNING_SPIN;
                robot_cmd.rotate_speed = DR16_instance.control_data.Normalize_wheel * (float)rotate_speed_MAX;
            }
        }
        else
        {
            int32_t key_x = (DR16_instance.control_data.keys.bits.W ? 1 : 0) -
                            (DR16_instance.control_data.keys.bits.S ? 1 : 0);
            int32_t key_y = (DR16_instance.control_data.keys.bits.A ? 1 : 0) -
                            (DR16_instance.control_data.keys.bits.D ? 1 : 0);
            float speed_scale = LEVEL_GAIN + (robot_cmd.Speed_Up_flag ? 1.0f : 0.0f);

            planningV.X = (float)key_x * speed_scale;
            planningV.Y = (float)key_y * speed_scale;
            planningV.Omega = 0.0f;
        }

        if (robot_cmd.chassis_mode == CHASSIS_RUNNING_SPIN)
        {
            if (robot_cmd.Control_mode == CONTROL_KEY_MOUSE)
            {
                Rotate_Speed_Set();
            }

            planningV.Omega = limit(robot_cmd.rotate_speed / (float)rotate_speed_MAX, 1.0f, -1.0f);
            planningV.X *= 1.4f;
            planningV.Y *= 1.4f;
        }

        slopeningV.X = Slope_Calc(&slope_X, planningV.X, slopeningV.X);
        slopeningV.Y = Slope_Calc(&slope_Y, planningV.Y, slopeningV.Y);
        slopeningV.Omega = Slope_Calc(&slope_Omega, planningV.Omega, slopeningV.Omega);

        if (gimbal.yaw_motor != NULL)
        {
            float chassis_offset =
                (float)(gimbal.yaw_motor->Data.DJI_data.MechanicalAngle - Yaw_Mid_Front) / 1303.80f;
            float sin_theta = arm_sin_f32(chassis_offset);
            float cos_theta = arm_cos_f32(chassis_offset);

            Gimbal_To_Chassis.vx = (int16_t)((slopeningV.Y * cos_theta - slopeningV.X * sin_theta) * 750.0f);
            Gimbal_To_Chassis.vy = (int16_t)((slopeningV.Y * sin_theta + slopeningV.X * cos_theta) * 750.0f);
        }
        else
        {
            Gimbal_To_Chassis.vx = (int16_t)(slopeningV.Y * 750.0f);
            Gimbal_To_Chassis.vy = (int16_t)(slopeningV.X * 750.0f);
        }

        Gimbal_To_Chassis.rotate = (int16_t)(slopeningV.Omega * 2000.0f);
    }

    Gimabl_To_Chassis_Action.Break_Limitation = Gimbal_To_Chassis.Unlimit_flag;

    CAN_Send(can_comm_instance.can_comm, (uint8_t *)&Gimbal_To_Chassis, 10.0f);
    CAN_Send(can_comm2, (uint8_t *)&Gimabl_To_Chassis_Action, 10.0f);
}
