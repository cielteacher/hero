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
#include <stdbool.h>

/* ==================== 全局/静态变量 ==================== */
Robot_ctrl_cmd_t robot_cmd = {0};
static Slope_s slope_X, slope_Y, slope_Omega; // 底盘速度规划
Chassis_speed_s planningV = {0.0f, 0.0f, 0.0f};// 目标速度输出
Chassis_speed_s slopeningV = {0.0f, 0.0f, 0.0f};// 斜坡限幅输出
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
static void Chassis_Comm_Update(void);
static void Rotate_Speed_Set(void);
static void Follow_Rotate_Speed_Update(void);
// 检查模块在线（奇妙的检测逻辑，等有时间了在改改）
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
    if(!gimbal.yaw_motor->dm_motor_online_flag || !gimbal.pitch_motor->dm_motor_online_flag)
    {
        robot_cmd.gimbal_mode = GIMBAL_DISABLED;
        return 0U;
    }
    return 1U;
}

/**
 * @brief 检查发射模块在线状态
 */
static uint8_t CheckShootOnline(void)
{
    if (shoot.Pluck_motor == NULL || shoot.fric_motor_LF == NULL || shoot.fric_motor_RF == NULL ||
        shoot.fric_motor_LB == NULL || shoot.fric_motor_RB == NULL)
    {
        robot_cmd.shoot_mode = SHOOT_DISABLED;
        return 0U;
    }
    if(!shoot.Pluck_motor->dji_motor_online_flag || !shoot.fric_motor_LF->dji_motor_online_flag ||
       !shoot.fric_motor_RF->dji_motor_online_flag || !shoot.fric_motor_LB->dji_motor_online_flag ||
       !shoot.fric_motor_RB->dji_motor_online_flag)
    {
        robot_cmd.shoot_mode = SHOOT_DISABLED;
        return 0U;
    }
    return 1U;
}
/**
 * @brief 视觉是否可以接管云台
 */
static uint8_t VisionCanAutoAim(void)
{
    if(MiniPC_instance.MiniPC_Online_Flag &&MiniPC_instance.receive_data.data.dis <= 0.2f)
    {
        return 1U;
    }
    else 
    {
        return 0U;
    }
}
/* ==================== 机器人初始化 ==================== */
void Robot_Init(void)
{
    // 初始化斜坡限幅器
    Slope_Init(&slope_X, 0.006f, 0.08f, SLOPE_FIRST_REAL);
    Slope_Init(&slope_Y, 0.006f, 0.08f, SLOPE_FIRST_REAL);
    Slope_Init(&slope_Omega, 3.0f * PI / 1000.0f, 30.0f * PI / 1000.0f, SLOPE_FIRST_REAL);
    // 初始化状态为停止
    robot_cmd.robot_state = ROBOT_STOP;
    robot_cmd.chassis_mode = CHASSIS_STOP;
    robot_cmd.Mid_mode = MID_FRONT;// 默认前方归中
    robot_cmd.Gyro_Position_Pitch = INS_Info.Pitch_Angle;
    robot_cmd.Gyro_Position_Yaw = INS_Info.Yaw_TolAngle;
    Gimbal_Init();
    Shoot_Init();

}

/* ==================== 机器人状态更新 ==================== */
void Robot_Update(void)
{
         /* 先赋予默认值*/
    robot_cmd.robot_state = ROBOT_RUNNING;
    robot_cmd.gimbal_mode = GIMBAL_RUNNING;
    robot_cmd.chassis_mode = CHASSIS_RUNNING_FOLLOW;
    robot_cmd.shoot_mode = SHOOT_READY_NOFRIC;
    // 遥控器掉线优先级最高，立即进入全失能状态；底盘/云台/发射模块掉线则进入对应的失能状态，但不影响其他模块继续工作。
    if (!DR16_instance.dr16_online_flag)
    {
        robot_cmd.robot_state = ROBOT_STOP;
        robot_cmd.gimbal_mode = GIMBAL_DISABLED;
        robot_cmd.shoot_mode = SHOOT_DISABLED;
        robot_cmd.chassis_mode = CHASSIS_DISABLED;
        if(can_comm_instance.can_comm_online_flag)
        {
            Chassis_Comm_Update();
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
        robot_cmd.robot_state = ROBOT_STOP;
        robot_cmd.gimbal_mode = GIMBAL_DISABLED;
        robot_cmd.shoot_mode = SHOOT_DISABLED;
        robot_cmd.chassis_mode = CHASSIS_STOP;
        if(can_comm_instance.can_comm_online_flag)
        {
            Chassis_Comm_Update();
        }
        return;
        break;
    default:
        break;
    }
    CheckGimbalOnline();//检查云台在线状态，并在离线时进入云台失能模式
    CheckShootOnline();//检查发射在线状态，并在离线时进入发射失能模式
    if(robot_cmd.gimbal_mode == GIMBAL_DISABLED)
    {
        robot_cmd.shoot_mode = SHOOT_DISABLED;// 云台失能时发射也失能，防止云台失控时误伤
        robot_cmd.chassis_mode = CHASSIS_RUNNING_NORMAL;// 云台失能时底盘不跟随，进入普通遥控模式，增加操作灵活性
    }
    Gimbal_Target_Update();
    Chassis_Comm_Update();
}
// ==================== 模式更新 键鼠+遥控器 具体云台转动再gimbal函数处理 ====================
// 注意 这里只做状态更新 不做任何控制输出和目标设定
/*
 *@brief 处理遥控器输入，更新机器人控制命令
 * fn1=1时进入发射准备状态，
 * fn1+fn2=1时进入自瞄状态（如果视觉可用
 * fn1+trigger=1时进入开火状态
 * 滚轮控制旋转
 */
static void RCUpdate(void)
{
    robot_cmd.Speed_Up_flag = false;
    if (DR16_instance.control_data.fn_1)
    {
        robot_cmd.shoot_mode = SHOOT_READY;

        if (DR16_instance.control_data.fn_2)
        {
            robot_cmd.chassis_mode = CHASSIS_RUNNING_NORMAL;
            uint8_t aim_available = VisionCanAutoAim();//视觉自瞄是否可用存档
            robot_cmd.shoot_mode = aim_available ? SHOOT_AIM : SHOOT_READY;
            robot_cmd.gimbal_mode = aim_available ? GIMBAL_RUNNING_AIM : GIMBAL_RUNNING;
        }

        if (DR16_instance.control_data.trigger)
        {
            robot_cmd.shoot_mode = SHOOT_FIRE;
        }
    }
    else
    {
        robot_cmd.shoot_mode = SHOOT_READY_NOFRIC;
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
 * 鼠标左键开火（如果摩擦轮达到目标速度）如果未达到目标速度，将开启并在延迟一段时间（等待摩擦轮上速）后发射
*/
static void KMUpdate(void)
{
    static uint8_t fric_toggle = 0U;// 键鼠模式下摩擦轮开关切换变量
    //在开火时，做一定的保护
    if(robot_cmd.shoot_mode !=SHOOT_FIRE && robot_cmd.shoot_mode != SHOOT_DISABLED && robot_cmd.shoot_mode != SHOOT_AIM)
    {
        robot_cmd.shoot_mode = fric_toggle ? SHOOT_READY : SHOOT_READY_NOFRIC;
    }
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
        robot_cmd.Mid_mode = MID_NONE;// 就近归中
        robot_cmd.Gyro_Position_Yaw = INS_Info.Yaw_TolAngle + 180.0f;
    }
    // E键长按时取消跟随
    if (DR16_instance.control_data.keys.bits.E)
    {
        robot_cmd.chassis_mode = CHASSIS_RUNNING_NORMAL;
        robot_cmd.Mid_mode = MID_NONE;// 取消跟随后就近归中
    }
    // 长按鼠标右键开启自瞄
    if (DR16_instance.control_data.long_press_r)
    {
        if (VisionCanAutoAim())//检查自瞄是否可用
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
    // Q键松开时恢复跟随模式（如果之前不是就近归中状态的话）
    if (!DR16_instance.control_data.keys.bits.Q && DR16_instance.control_data.last_keys.bits.Q)
    {
        robot_cmd.Mid_mode = MID_FRONT;
    }
    // 左键控制 开火
    if (!DR16_instance.control_data.last_mouse_press_l&&DR16_instance.control_data.press_l)
    {
        if(fric_toggle && CheckFricSpeedReady())
        {
             robot_cmd.shoot_mode = SHOOT_FIRE;
        }
        else
        {
            fric_toggle = 1U;
            if(CheckFricSpeedReady())
            {
                robot_cmd.shoot_mode = SHOOT_FIRE;

            }
        }
    }

}

/* ==================== 云台目标更新 ==================== */
static void Gimbal_Target_Update(void)
{
    switch(robot_cmd.gimbal_mode)
    {    
        case GIMBAL_DISABLED:// 全失能状态，直接退出
            robot_cmd.Gyro_Position_Pitch = INS_Info.Pitch_Angle;
            robot_cmd.Gyro_Position_Yaw = INS_Info.Yaw_TolAngle;
            return;
        break;
        case GIMBAL_STOP:// 停止状态，保持当前角度(保持不住....)
            robot_cmd.Gyro_Position_Pitch = INS_Info.Pitch_Angle;
            robot_cmd.Gyro_Position_Yaw = INS_Info.Yaw_TolAngle;
        break;
        case GIMBAL_RUNNING://  遥控器or键鼠
            if (robot_cmd.Control_mode == CONTROL_KEY_MOUSE)
            {
                robot_cmd.Gyro_Position_Pitch += DR16_instance.control_data.y / (float)MOUSE_MAX * MOUSE_PITCH_SENSITIVITY;
                robot_cmd.Gyro_Position_Yaw +=   DR16_instance.control_data.x / (float)MOUSE_MAX * MOUSE_YAW_SENSITIVITY;
            }
            else if (robot_cmd.Control_mode == CONTROL_REMOTE)
            {
                robot_cmd.Gyro_Position_Pitch += DR16_instance.control_data.Normalize_ch2 * RC_YAW_SENSITIVITY;
                robot_cmd.Gyro_Position_Yaw   += DR16_instance.control_data.Normalize_ch3 * RC_PITCH_SENSITIVITY;
            }
        break;
        case GIMBAL_RUNNING_AIM:// 自瞄做一个就近转位
            robot_cmd.Gyro_Position_Pitch = MiniPC_instance.receive_data.data.Ref_pitch;
            float yaw_diff = MiniPC_instance.receive_data.data.Ref_yaw - INS_Info.Yaw_Angle;
            int8_t round_offset = 0;
            if (yaw_diff < -240.0f)
            {
                round_offset = 1;
            }
            else if (yaw_diff > 240.0f)
            {
                round_offset = -1;
            }
            robot_cmd.Gyro_Position_Yaw = MiniPC_instance.receive_data.data.Ref_yaw +(INS_Info.YawRoundCount + round_offset) * 360.0f;
        break;

    }
}

/* ==================== 小陀螺速度设置 ==================== */
// 根据云台离装甲板的位置变速，离得近快
static void Rotate_Speed_Set(void)
{
    const float sector_centers[] = {Yaw_Mid_Front, Yaw_Mid_Back, Yaw_Mid_Left, Yaw_Mid_Right};
    float yaw_single;
    float min_offset = 8192.0f;

    if (gimbal.yaw_motor == NULL)
    {
        planningV.Omega = 0.0f;
        return;
    }

    yaw_single = fmodf((float)gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle, 8192.0f);
    if (yaw_single < 0.0f)
    {
        yaw_single += 8192.0f;
    }

    for (uint8_t i = 0; i < 4; i++)
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
        planningV.Omega = limit(target_speed/ (float)rotate_speed_MAX,1.0f, -1.0f);
    }
}

/* ==================== 跟随模式旋转速度设置 ==================== */
static void Follow_Rotate_Speed_Update(void)
{
    if(gimbal.yaw_motor == NULL)
    {
        planningV.Omega = 0.0f;
        return;
    }
    const int32_t follow_centers[2] = {Yaw_Mid_Front, Yaw_Mid_Back };
    int32_t target_single = Yaw_Mid_Front;
    /* MID_NONE: 方向就近归中 */
    if (robot_cmd.Mid_mode == MID_NONE)
    {
        int32_t min_abs_err = MAX_EC_VALUE;
        for (uint8_t i = 0U; i < 2U; i++)
        {
            int32_t err = follow_centers[i] - gimbal.yaw_motor->Data.DJI_data.MechanicalAngle;
            if (err > 4096)
            {
                err -= MAX_EC_VALUE;
            }
            else if (err < -4096)
            {
                err += MAX_EC_VALUE;
            }
            int32_t abs_err = (err >= 0) ? err : -err;
            if (abs_err < min_abs_err)
            {
                min_abs_err = abs_err;
                target_single = follow_centers[i];
            }
        }
    }
    else if (robot_cmd.Mid_mode == MID_BACK)
    {
        target_single = Yaw_Mid_Back;
    }
    else
    {
        target_single = Yaw_Mid_Front;
    }
    // 进行一次快速归中，防止南辕北辙跳转
    target_single = QuickCentering(gimbal.yaw_motor->Data.DJI_data.MechanicalAngle, target_single);
    
    PID_Control_Smis((float)gimbal.yaw_motor->Data.DJI_data.MechanicalAngle, (float)target_single, &Chassis_Rotate_PIDS, gimbal.yaw_motor->Data.DJI_data.SpeedFilter);

    /* 跟随关系里机械角与底盘旋转通常为反号 */
    planningV.Omega = (-PID_Control(gimbal.yaw_motor->Data.DJI_data.SpeedFilter, Chassis_Rotate_PIDS.pid_out, &Chassis_Rotate_PID)/ (float)rotate_speed_MAX);
    planningV.Omega = limit(planningV.Omega, 1.0f, -1.0f);
}

/* ==================== 底盘通信更新 ==================== */
static void Chassis_Comm_Update()
{
    static const float LEVEL_GAIN = 2.0f;// 控制时的速度增益
    static Gimbal_board_send_t Gimbal_To_Chassis;// 每次更新前先清空发送结构体，防止未赋值项的垃圾数据干扰底盘控制
    if (can_comm_instance.can_comm_online_flag == 0U || can_comm_instance.can_comm == NULL)
    {
        return;
    }
    //每次更新清空
    planningV.X = 0.0f;
    planningV.Y = 0.0f;
    planningV.Omega = 0.0f;

    Gimbal_To_Chassis.flag.bits.Gimbal_Online = robot_cmd.gimbal_mode != GIMBAL_DISABLED ? 1U : 0U;
    Gimbal_To_Chassis.flag.bits.Shoot_Online = robot_cmd.shoot_mode != SHOOT_DISABLED ? 1U : 0U;
    Gimbal_To_Chassis.flag.bits.Vision_Online = MiniPC_instance.MiniPC_Online_Flag;
    Gimbal_To_Chassis.flag.bits.Unlimit_flag = robot_cmd.Unlimit_flag ? 1U : 0U;
    Gimbal_To_Chassis.flag.bits.Close_flag = 0U;

    switch (robot_cmd.chassis_mode) 
    {
        case CHASSIS_DISABLED:
        case CHASSIS_STOP:
            Gimbal_To_Chassis.vx = 0;
            Gimbal_To_Chassis.vy = 0;
            Gimbal_To_Chassis.rotate = 0;
            Gimbal_To_Chassis.flag.bits.Unlimit_flag = 0U;
            Gimbal_To_Chassis.flag.bits.Close_flag = 1U;
            // 进入停止状态时重置斜坡规划，防止再次启动时的速度突变
            Slope_Reset(&slope_X, 0.0f);
            Slope_Reset(&slope_Y, 0.0f);   
            Slope_Reset(&slope_Omega, 0.0f);
            goto chassis_send;
        break;
        case CHASSIS_RUNNING_FOLLOW:
        if(robot_cmd.gimbal_mode !=  GIMBAL_DISABLED)
        {
            Follow_Rotate_Speed_Update();
            if (robot_cmd.Control_mode == CONTROL_REMOTE)
            {
                planningV.Y = DR16_instance.control_data.Normalize_ch2 * LEVEL_GAIN * 2.0f;
                planningV.X = DR16_instance.control_data.Normalize_ch3 * LEVEL_GAIN;
            }
            else
            {
                int32_t key_x = (DR16_instance.control_data.keys.bits.W ? 1 : 0) - (DR16_instance.control_data.keys.bits.S ? 1 : 0);
                int32_t key_y = (DR16_instance.control_data.keys.bits.A ? 1 : 0) - (DR16_instance.control_data.keys.bits.D ? 1 : 0);
                float speed_scale = LEVEL_GAIN + (robot_cmd.Speed_Up_flag ? 1.0f : 0.0f);

                planningV.X = (float)key_x * speed_scale;
                planningV.Y = (float)key_y * speed_scale;

            }
        }
        break;
        case CHASSIS_RUNNING_SPIN:
            Rotate_Speed_Set();
            if (robot_cmd.Control_mode == CONTROL_REMOTE)
            {
                planningV.Y = DR16_instance.control_data.Normalize_ch2 * LEVEL_GAIN * 2.0f;
                planningV.X = DR16_instance.control_data.Normalize_ch3 * LEVEL_GAIN;
            }
            else
            {
                int32_t key_x = (DR16_instance.control_data.keys.bits.W ? 1 : 0) - (DR16_instance.control_data.keys.bits.S ? 1 : 0);
                int32_t key_y = (DR16_instance.control_data.keys.bits.A ? 1 : 0) - (DR16_instance.control_data.keys.bits.D ? 1 : 0);
                float speed_scale = LEVEL_GAIN + (robot_cmd.Speed_Up_flag ? 1.0f : 0.0f);

                planningV.X = (float)key_x * speed_scale;
                planningV.Y = (float)key_y * speed_scale;

            }
        break;
        case CHASSIS_RUNNING_NORMAL:
            if (robot_cmd.Control_mode == CONTROL_REMOTE)
            {
                planningV.Y = DR16_instance.control_data.Normalize_ch2 * LEVEL_GAIN * 2.0f;
                planningV.X = DR16_instance.control_data.Normalize_ch3 * LEVEL_GAIN;
                if(DR16_instance.control_data.Normalize_wheel)
                {
                    planningV.Omega = DR16_instance.control_data.Normalize_wheel;
                }
            }
            else
            {
                int32_t key_x = (DR16_instance.control_data.keys.bits.W ? 1 : 0) - (DR16_instance.control_data.keys.bits.S ? 1 : 0);
                int32_t key_y = (DR16_instance.control_data.keys.bits.A ? 1 : 0) - (DR16_instance.control_data.keys.bits.D ? 1 : 0);
                float speed_scale = LEVEL_GAIN + (robot_cmd.Speed_Up_flag ? 1.0f : 0.0f);

                planningV.X = (float)key_x * speed_scale;
                planningV.Y = (float)key_y * speed_scale;
                planningV.Omega = 0.0f;
            }
            // 其他模式在后续代码中更新速度
        break;
        default:
        break;
    }
        slopeningV.X = Slope_Calc(&slope_X, planningV.X, slopeningV.X);
        slopeningV.Y = Slope_Calc(&slope_Y, planningV.Y, slopeningV.Y);
        slopeningV.Omega = Slope_Calc(&slope_Omega, planningV.Omega, slopeningV.Omega);

        if (gimbal.yaw_motor != NULL && robot_cmd.gimbal_mode != GIMBAL_DISABLED)
        {
            float chassis_offset = (float)(gimbal.yaw_motor->Data.DJI_data.MechanicalAngle - Yaw_Mid_Front) / 1303.80f;
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
    
    chassis_send:
    CAN_Send(can_comm_instance.can_comm, (uint8_t *)&Gimbal_To_Chassis, 10.0f);

}
