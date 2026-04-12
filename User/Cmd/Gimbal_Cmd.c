/**
 * @file    Gimbal_Cmd.c
 * @brief   云台决策层 - 模式决策、目标更新、板间通信
 */
#include "Gimbal_Cmd.h"
#include "PID.h"
#include "arm_math.h"
#include "bsp_fdcan.h"
#include "robot.h"
#include "slope.h"
#include <math.h>

/* ==================== 全局/静态变量 ==================== */
Robot_ctrl_cmd_t robot_cmd = {0};
static Gimbal_board_send_t Gimbal_To_Chassis;
static Gimbal_action_t Gimabl_To_Chassis_Action;
static Slope_s slope_X, slope_Y, slope_Omega;
static CANInstance *can_comm2;
extern uint8_t power_on_center_flag;

/* 跟随模式底盘旋转PID参数（位置环+速度环） */
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

/* ==================== 内部函数声明 ==================== */
static void RCUpdate(void);
static void KMUpdate(void);
static void Gimbal_Target_Update(void);
static void Chassis_Comm_Update(uint8_t gimbal_online_now, uint8_t shoot_online_now);
static void Rotate_Speed_Set(void);
static void Follow_Rotate_Speed_Update(uint8_t gimbal_online_now);
static void ApplySafetyInterlocks(uint8_t gimbal_online_now,
                                  uint8_t shoot_online_now,
                                  uint8_t chassis_online_now);
static void SyncCmdReferenceToFeedback(void);
static void ClearGimbalVelocityControlState(void);
static void ForceRobotDisabledState(void);
static float SlewLimit(float target, float current, float step);

/**
 * @brief 检查云台模块在线状态
 */
static uint8_t CheckGimbalOnline(void)
{
    if(gimbal.yaw_motor == NULL || gimbal.pitch_motor == NULL)
    {
        robot_cmd.gimbal_mode = GIMBAL_DISABLED;
        return 0U;
    }
    if(!gimbal.yaw_motor->dji_motor_online_flag || !gimbal.pitch_motor->dji_motor_online_flag)
    {
        robot_cmd.gimbal_mode = GIMBAL_DISABLED;
        return 0U;
    }
}

/**
 * @brief 检查发射模块在线状态
 */
static uint8_t CheckShootOnline(void)
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
        MiniPC_instance.receive_data.data.dis <= 0.2f)? 1U: 0U;
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


/* ==================== 机器人初始化 ==================== */
void Robot_Init(void)
{
    CAN_Init_Config_s can_comm2_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x120,
        .rx_id = 0,
        .rx_len = 8,
    };
    // 初始化斜坡限幅器
    Slope_Init(&slope_X, 0.006f, 0.08f, SLOPE_FIRST_REAL);
    Slope_Init(&slope_Y, 0.006f, 0.08f, SLOPE_FIRST_REAL);
    Slope_Init(&slope_Omega, 3.0f * PI / 1000.0f, 30.0f * PI / 1000.0f, SLOPE_FIRST_REAL);

    can_comm2 = CANRegister(&can_comm2_config);
    // 初始化状态为停止
    robot_cmd.robot_state = ROBOT_STOP;
    robot_cmd.chassis_mode = CHASSIS_STOP;
    robot_cmd.Mid_mode = MID_FRONT;
    ClearGimbalVelocityControlState();
    robot_cmd.last_gimbal_mode = robot_cmd.gimbal_mode;

    Gimbal_Init();
    Shoot_Init();
    Can_Comm_Init();
}

/* ==================== 机器人状态更新 ==================== */
void Robot_Update(void)
{
    uint8_t remote_online = DR16_instance.dr16_online_flag;
    uint8_t chassis_online_now = can_comm_instance.can_comm_online_flag;
    // 遥控器掉线优先级最高，立即进入全失能状态；底盘/云台/发射模块掉线则进入对应的失能状态，但不影响其他模块继续工作。
    if (!remote_online)
    {
        robot_cmd.robot_state = ROBOT_STOP;
    }
    CheckGimbalOnline();//检查云台在线状态，并在离线时进入云台失能模式
    CheckShootOnline();//检查发射在线状态，并在离线时进入发射失能模式
    if(DR16_instance.control_data.input_mode == Control_Stop)
        robot_cmd.robot_state = ROBOT_STOP;
    if(robot_cmd.robot_state == ROBOT_STOP)
    {
        robot_cmd.gimbal_mode = GIMBAL_DISABLED;
        robot_cmd.shoot_mode = SHOOT_DISABLED;
        if(chassis_online_now)
        {
            Chassis_Comm_Update( 0U, 0U);
        }
        return;
    }
    // 根据遥控器输入更新控制方式（最左边停止，中遥控器，右键鼠标键盘）
    switch (DR16_instance.control_data.input_mode)
    {
    case REMOTE_INPUT:
        robot_cmd.Control_mode = CONTROL_REMOTE;
        RCUpdate();
        break;
    case KEY_MOUSE_INPUT:
        robot_cmd.Control_mode = CONTROL_KEY_MOUSE;
        KMUpdate();
        break;
    case Control_Stop:
    default:
        break;
    }

    Gimbal_Target_Update();
    Chassis_Comm_Update(robot_cmd.gimbal_mode?1u:0u , robot_cmd.shoot_mode?1u:0u);
}
// ==================== 模式更新 键鼠+遥控器 具体云台转动再gimbal函数处理 ====================
/*
 *@brief 处理遥控器输入，更新机器人控制命令
 * fn1=1时进入发射准备状态，
 * fn1+fn2=1时进入自瞄状态（如果视觉可用
 * fn1+trigger=1时进入开火状态
 * 滚轮控制旋转
 */
static void RCUpdate(void)
{
    robot_cmd.gimbal_mode = GIMBAL_RUNNING;
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
            robot_cmd.shoot_mode = VisionCanAutoAim() ? SHOOT_AIM : SHOOT_READY;
        }

        if (DR16_instance.control_data.trigger)
        {
            robot_cmd.shoot_mode = SHOOT_FIRE;
        }
    }
    if(DR16_instance.control_data.wheel)
    {
        robot_cmd.rotate_speed = DR16_instance.control_data.wheel * rotate_speed_MAX;
    }
}

/**
 * @brief 键鼠模式更新
 * 默认状态为跟随+关闭摩擦轮 
 * Ctrl+R组合键触发系统复位
 * Ctrl+G组合键触发底盘解限
 * B键切换摩擦轮状态（开启or关闭）
 * Shift键按时加速
 * F键按 下 时就近归中（和上电归中逻辑一致）(优先等级很高)
 * E键按 住 时取消跟随，进入底盘分离模式
 * Q键按 住 时进入小陀螺模式
 * 长按鼠标右键开启自瞄（自瞄可以识别则自瞄算法接管云台+发射机构反之，操作手位控云台）
                 （此时底盘默认为分离模式提高精度，但如果开启小陀螺还是小陀螺优先）
 * 鼠标左键开火（如果摩擦轮已开）如果未开，将开启并在延迟一段时间（等待摩擦轮上速）后发射
 * 鼠标左键长按（500ms）也只会开火一次
*/
static void KMUpdate(void)
{
    static uint8_t fric_toggle = 0U;// 键鼠模式下摩擦轮开关切换变量

    robot_cmd.gimbal_mode = GIMBAL_RUNNING;
    robot_cmd.chassis_mode = CHASSIS_RUNNING_FOLLOW;
    robot_cmd.Mid_mode = MID_FRONT;
    robot_cmd.shoot_mode = fric_toggle ? SHOOT_READY : SHOOT_READY_NOFRIC;
    // Ctrl+R组合键触发系统复位（
    if (DR16_instance.control_data.keys.bits.Ctrl && DR16_instance.control_data.keys.bits.R &&
        !DR16_instance.control_data.last_keys.bits.R)
    {
        NVIC_SystemReset();
    }
    // Ctrl+G组合键触发底盘解限（自爆？）
    if (DR16_instance.control_data.keys.bits.Ctrl && DR16_instance.control_data.keys.bits.G &&
        !DR16_instance.control_data.last_keys.bits.G)
    {
        robot_cmd.Unlimit_flag ^= 1U;
    }
    // B键切换摩擦轮状态
    if (DR16_instance.control_data.keys.bits.B && !DR16_instance.control_data.last_keys.bits.B)
    {
        fric_toggle ^= 1U;
    }
    // Shift键按下时加速
    robot_cmd.Speed_Up_flag = DR16_instance.control_data.keys.bits.Shift ? true : false;
    // F键按下时就近归中状态(优先等级很高)
    if (DR16_instance.control_data.keys.bits.F &&
        !DR16_instance.control_data.last_keys.bits.F)
    {
        power_on_center_flag = 0U;
    }
    // E键长按时取消跟随
    if (DR16_instance.control_data.keys.bits.E)
    {
        robot_cmd.chassis_mode = CHASSIS_RUNNING_NORMAL;
    }
    if (DR16_instance.control_data.long_press_r)
    {
        if (VisionCanAutoAim())
        {
            robot_cmd.gimbal_mode = GIMBAL_RUNNING_AIM;
            robot_cmd.chassis_mode = CHASSIS_RUNNING_NORMAL;
            robot_cmd.shoot_mode = SHOOT_AIM;
        }
        else
        {
            robot_cmd.gimbal_mode = GIMBAL_RUNNING;
            robot_cmd.shoot_mode = fric_toggle ? SHOOT_READY : SHOOT_READY_NOFRIC;
        }
    }
    // Q键长按时进入小陀螺模式
    if (DR16_instance.control_data.keys.bits.Q)
    {
        robot_cmd.chassis_mode = CHASSIS_RUNNING_SPIN;
    }
    static uint16_t speed_up_time = 0U;
    if (DR16_instance.control_data.press_l)
    {
        if(DR16_instance.control_data.long_press_l)
        {
            fric_toggle = 1U;
            speed_up_time = 0U;
            robot_cmd.shoot_mode = SHOOT_READY;
            return;
        // 长按鼠标左键时仅开启摩擦轮，不触发连续射击（因为长按阈值时500ms）因此只射击一次
        }
        if(fric_toggle)
        {
             robot_cmd.shoot_mode = SHOOT_FIRE;
        }
        else
        {
            fric_toggle = 1U;
            speed_up_time++;
            if(speed_up_time >= FRIC_SPEED_UP_TIME)
            {
                robot_cmd.shoot_mode = SHOOT_FIRE;
                speed_up_time = 0U;
            }
        }
    }
    else{speed_up_time = 0U;}
}

/* ==================== 云台目标更新 ==================== */
static void Gimbal_Target_Update(void)
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

        if (VisionCanAutoAim())
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

        robot_cmd.gimbal_mode = (robot_cmd.Control_mode == CONTROL_KEY_MOUSE)
                                    ? GIMBAL_RUNNING_NORMAL
                                    : GIMBAL_RUNNING_FOLLOW;
        if (robot_cmd.shoot_mode == SHOOT_AIM)
        {
            robot_cmd.shoot_mode = SHOOT_READY;
        }
    }

    robot_cmd.use_velocity_control = 1U;

    if (!robot_cmd.last_velocity_mode)
    {
        yaw_velocity_cmd = INS_Info.Yaw_Gyro;
        pitch_velocity_cmd = INS_Info.Pitch_Gyro;
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

    robot_cmd.Gyro_Yaw = INS_Info.Yaw_TolAngle;
    robot_cmd.Gyro_Pitch = INS_Info.Pitch_Angle;

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

/* ==================== 跟随模式旋转速度设置 ==================== */
static void Follow_Rotate_Speed_Update(uint8_t gimbal_online_now)
{
    int32_t yaw_now;
    int16_t yaw_offset;
    float yaw_fdb;
    float yaw_ref;
    float yaw_rate;

    if (!gimbal_online_now || gimbal.yaw_motor == NULL)
    {
        robot_cmd.rotate_speed = 0.0f;
        return;
    }

    yaw_now = gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle;

    // 跟随模式默认对正前方，使用现有就近转位函数获取最短角度偏差。
    yaw_offset = move_nearby_int16(Yaw_Mid_Front, gimbal.yaw_motor->Data.DJI_data.MechanicalAngle);

    yaw_fdb = (float)yaw_now;
    yaw_ref = (float)(yaw_now + yaw_offset);
    yaw_rate = gimbal.yaw_motor->Data.DJI_data.SpeedFilter;

    PID_Control_Smis(yaw_fdb, yaw_ref, &Chassis_Rotate_PIDS, yaw_rate);
    PID_Control(yaw_rate, Chassis_Rotate_PIDS.pid_out, &Chassis_Rotate_PID);

    robot_cmd.rotate_speed = limit(Chassis_Rotate_PID.pid_out,
                                   (float)rotate_speed_MAX,
                                   -(float)rotate_speed_MAX);
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

        if (robot_cmd.chassis_mode == CHASSIS_RUNNING_FOLLOW)
        {
            Follow_Rotate_Speed_Update(gimbal_online_now);
            planningV.Omega = limit(robot_cmd.rotate_speed / (float)rotate_speed_MAX, 1.0f, -1.0f);
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
