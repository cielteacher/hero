/**
 * @file    Gimbal_Cmd.c
 * @brief   云台决策层 - 输入解析与模式管理
 */
#include "Gimbal_Cmd.h"
#include "bsp_fdcan.h"
#include "robot.h"
#include "arm_math.h"
#include "slope.h"
#include <math.h>

/* ==================== 全局/静态变量 ==================== */
Robot_ctrl_cmd_t robot_cmd;                        // 机器人控制指令
static Gimbal_board_send_t Gimbal_To_Chassis;      // 云台->底盘速度
static Gimbal_action_t Gimabl_To_Chassis_Action;   // 云台->底盘状态
static Slope_s slope_X, slope_Y, slope_Omega;      // 速度斜坡规划
static CANInstance *can_comm2;
// 在线状态标志
static uint8_t remote_online = 0;
static uint8_t chassis_online = 0;
static uint8_t gimbal_online = 0;
static uint8_t shoot_online = 0;
uint8_t imu_online = 0;
/* ==================== 内部函数声明 ==================== */
static void RCUpdate(void);
static void KMUpdate(void);
static void Gimbal_Target_Update(void);
static void Chassis_Comm_Update(void);
/**
 * @brief 检查云台模块在线状态
 */
static uint8_t IsGimbalModuleOnline(void)
{
    if (gimbal.pitch_motor == NULL || gimbal.yaw_motor == NULL)
        return 0;
    return (gimbal.pitch_motor->dm_motor_online_flag && gimbal.yaw_motor->dm_motor_online_flag) ? 1U : 0U;
}

/**
 * @brief 检查发射模块在线状态
 */
static uint8_t IsShootModuleOnline(void)
{
    if (shoot.Pluck_motor == NULL || shoot.fric_motor_1 == NULL || shoot.fric_motor_2 == NULL ||
        shoot.fric_motor_3 == NULL || shoot.fric_motor_4 == NULL)
        return 0;

    return (shoot.Pluck_motor->dji_motor_online_flag &&
            shoot.fric_motor_1->dji_motor_online_flag &&
            shoot.fric_motor_2->dji_motor_online_flag &&
            shoot.fric_motor_3->dji_motor_online_flag &&
            shoot.fric_motor_4->dji_motor_online_flag) ? 1U : 0U;
}
static uint8_t VisionCanAutoAim(void)
{
    return (MiniPC_instance.MiniPC_Online_Flag &&
            MiniPC_instance.receive_data.data.dis <= 0.2f) ? 1 : 0;
}
/* ==================== 机器人初始化 ==================== */
void Robot_Init(void)
{
    // 初始化斜坡规划器
    Slope_Init(&slope_X, 0.006f, 0.08f, SLOPE_FIRST_REAL);
    Slope_Init(&slope_Y, 0.006f, 0.08f, SLOPE_FIRST_REAL);
    Slope_Init(&slope_Omega, 3.0f * PI / 1000.0f, 30.0f * PI / 1000.0f, SLOPE_FIRST_REAL);

    // 注册板间通信CAN
    CAN_Init_Config_s can_comm2_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x120,
        .rx_id = 0,
        .rx_len = 8,
    };
    can_comm2 = CANRegister(&can_comm2_config);

    // 初始化控制状态
    robot_cmd.robot_state = ROBOT_STOP;
    robot_cmd.gimbal_mode = GIMBAL_STOP;
    robot_cmd.shoot_mode = SHOOT_STOP;
    robot_cmd.chassis_mode = CHASSIS_STOP;
    robot_cmd.Control_mode = CONTROL_REMOTE;
    robot_cmd.Mid_mode = MID_FRONT;  // 默认前方归中
    robot_cmd.Speed_Up_flag = false;

    Gimbal_Init();
    Shoot_Init();
    Can_Comm_Init();
}

/* ==================== 机器人状态更新 ==================== */
/**
 * @brief 机器人状态更新主函数 - 1ms周期
 */
void Robot_Update(void)
{
    // 获取在线状态
    remote_online = (DR16_instance.dr16_online_flag && !DR16_instance.dr16_data_error_flag) ? 1U : 0U;
    chassis_online = can_comm_instance.can_comm_online_flag ? 1U : 0U;
    imu_online = INS_Info.INS_online_flag ? 1U : 0U;
    gimbal_online = IsGimbalModuleOnline();
    shoot_online = IsShootModuleOnline();
    // 设置默认状态
    robot_cmd.robot_state = ROBOT_RUNNING;
    // 遥控器离线 -> 紧急停止
    if (!remote_online)
    {
        robot_cmd.robot_state = ROBOT_STOP;
        robot_cmd.gimbal_mode = GIMBAL_STOP;
        robot_cmd.chassis_mode = CHASSIS_STOP;
        robot_cmd.shoot_mode = SHOOT_STOP;
        robot_cmd.Speed_Up_flag = false;
    }

    if (robot_cmd.robot_state != ROBOT_STOP)
    {
        // 模块离线 -> 对应模块停止
        if (!gimbal_online)
            robot_cmd.gimbal_mode = GIMBAL_STOP;
        if (!shoot_online)
            robot_cmd.shoot_mode = SHOOT_STOP;
        if(!chassis_online)
            robot_cmd.chassis_mode = CHASSIS_STOP;

        // 根据输入模式更新
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
        default:
            robot_cmd.robot_state = ROBOT_STOP;
            robot_cmd.gimbal_mode = GIMBAL_STOP;
            robot_cmd.chassis_mode = CHASSIS_STOP;
            robot_cmd.shoot_mode = SHOOT_STOP;
            robot_cmd.Speed_Up_flag = false;
            break;
        }

        // 输入映射后再次施加模块离线约束，避免模式函数覆盖停机状态
        if (!gimbal_online)
            robot_cmd.gimbal_mode = GIMBAL_STOP;
        if (!shoot_online)
            robot_cmd.shoot_mode = SHOOT_STOP;
        if (!chassis_online)
            robot_cmd.chassis_mode = CHASSIS_STOP;
    }    
    // 更新云台目标与下板通信
    Gimbal_Target_Update();
    Chassis_Comm_Update();
}


/* ==================== 遥控器模式更新 ==================== */
/**
 * @brief 遥控器模式处理
 * @note  操作映射:
 *        fn1      = 启动摩擦轮
 *        fn1+fn2  = 自瞄模式
 *        fn1+trig = 手动开火
 *        拨轮     = 小陀螺
 */
static void RCUpdate(void)
{
    // 默认: 底盘跟随云台
    robot_cmd.gimbal_mode = GIMBAL_RUNNING_FOLLOW;
    robot_cmd.chassis_mode = CHASSIS_RUNNING_FOLLOW;
    robot_cmd.shoot_mode = SHOOT_READY_NOFRIC;

    // fn1按住: 启动摩擦轮
    if (DR16_instance.control_data.fn_1)
    {
        robot_cmd.shoot_mode = SHOOT_READY;

        // fn1+fn2: 自瞄模式
        if (DR16_instance.control_data.fn_2)
        {
            robot_cmd.gimbal_mode = GIMBAL_RUNNING_AIM;
            robot_cmd.chassis_mode = CHASSIS_RUNNING_NORMAL;
            robot_cmd.shoot_mode = VisionCanAutoAim() ? SHOOT_AIM : SHOOT_READY;
        }
        // fn1+trigger: 手动开火
        if (DR16_instance.control_data.trigger)
        {
            robot_cmd.shoot_mode = SHOOT_FIRE;
        }
    }
}

/* ==================== 键鼠模式更新 ==================== */
/**
 * @brief 键鼠模式处理
 * @note  按键映射:
 *        Ctrl+R = 系统复位
 *        Ctrl+G = 解除功率限制
 *        B      = 摩擦轮开关
 *        F      = 一键掉头(+180度)
 *        E按住  = 底盘分离模式
 *        Q按住  = 小陀螺模式  大于取消跟随（E）
 *        右键长按 = 自瞄
 *        左键点击 = 开火（必须开启摩擦轮）
 *        Shift  = 加速
 */
static void KMUpdate(void)
{
    // 静态变量保存toggle状态
    static uint8_t fric_toggle = 0;

    // 每帧先回到基础态，再叠加按键覆盖，避免模式残留
    robot_cmd.gimbal_mode = GIMBAL_RUNNING_FOLLOW;
    robot_cmd.chassis_mode = CHASSIS_RUNNING_FOLLOW;
    robot_cmd.shoot_mode = fric_toggle ? SHOOT_READY : SHOOT_READY_NOFRIC;
    robot_cmd.Mid_mode = MID_FRONT;
    robot_cmd.Speed_Up_flag = false;
    robot_cmd.rotate_speed = 0.0f;

    // Ctrl+R: 系统复位
    if (DR16_instance.control_data.keys.bits.Ctrl &&
        DR16_instance.control_data.keys.bits.R &&
        !DR16_instance.control_data.last_keys.bits.R)
    {
        NVIC_SystemReset();
    }
    // Ctrl+G: 底盘解限切换
    if (DR16_instance.control_data.keys.bits.Ctrl &&
        DR16_instance.control_data.keys.bits.G &&
        !DR16_instance.control_data.last_keys.bits.G)
    {
        robot_cmd.Unlimit_flag ^= 1U;
    }

    // B: 摩擦轮切换
    if (DR16_instance.control_data.keys.bits.B && !DR16_instance.control_data.last_keys.bits.B)
    {
        fric_toggle ^= 1U;
    }
    // Shift: 加速
    robot_cmd.Speed_Up_flag = DR16_instance.control_data.keys.bits.Shift ? true : false;

    // F: 一键掉头 (Yaw+180度)
    if (DR16_instance.control_data.keys.bits.F && !DR16_instance.control_data.last_keys.bits.F)
    {
        robot_cmd.gimbal_mode = GIMBAL_RUNNING_FOLLOW;  // 切换到跟随模式以执行归中
        robot_cmd.chassis_mode = CHASSIS_RUNNING_FOLLOW;
        robot_cmd.Mid_mode = MID_NONE;  // 就近跟随
        if(imu_online)
        {
            robot_cmd.Gyro_Yaw += 180.0f;  // 增加180度
        }
        else
        {
            robot_cmd.Mech_Yaw += 4096;    // 编码器值增加半圈
        }
    }
    // E按住: 底盘分离模式
    if (DR16_instance.control_data.keys.bits.E && DR16_instance.control_data.last_keys.bits.E)
    {
        robot_cmd.gimbal_mode = GIMBAL_RUNNING_NORMAL;
        robot_cmd.chassis_mode = CHASSIS_RUNNING_NORMAL;
    }
    if (DR16_instance.control_data.keys.bits.Q && DR16_instance.control_data.last_keys.bits.Q)
    {    // Q按住: 小陀螺模式
        robot_cmd.gimbal_mode = GIMBAL_RUNNING_NORMAL;
        robot_cmd.chassis_mode = CHASSIS_RUNNING_SPIN;
    }

    // 右键长按: 自瞄模式
    if (DR16_instance.control_data.long_press_r)
    {
        if (VisionCanAutoAim())
        {// 视觉自瞄模式
            robot_cmd.gimbal_mode = GIMBAL_RUNNING_AIM;
            robot_cmd.shoot_mode = SHOOT_AIM;
        }
        else
        {//视觉数据不可信离线降级到跟随模式
            robot_cmd.gimbal_mode = GIMBAL_RUNNING_FOLLOW;
            robot_cmd.shoot_mode = SHOOT_READY;
        }
    }
    // 左键点击: 手动开火
    if (DR16_instance.control_data.press_l && !DR16_instance.control_data.last_mouse_press_l)
    {
        if (fric_toggle)
        {
            robot_cmd.shoot_mode = SHOOT_FIRE;
        }
    }

}
/* ==================== 云台目标更新 ==================== */
static void Gimbal_Target_Update(void)
{
    float yaw_delta = 0.0f, pitch_delta = 0.0f;
    uint8_t use_mech_ref = 0U;

    if (robot_cmd.gimbal_mode == GIMBAL_RUNNING_AIM && imu_online)
    {// 视觉自瞄模式 - 直接使用视觉数据计算目标角度，IMU在线时使用陀螺仪角度进行增量控制
        float vision_yaw = MiniPC_instance.receive_data.data.Ref_yaw;
        float yaw_diff = vision_yaw - INS_Info.Yaw_Angle;
        int32_t round_offset = 0;
        if (yaw_diff < -240.0f) round_offset = 1;
        else if (yaw_diff > 240.0f) round_offset = -1;
        robot_cmd.Gyro_Yaw = (INS_Info.YawRoundCount + round_offset) * 360.0f + vision_yaw;
        robot_cmd.Gyro_Pitch = INS_Info.Pitch_Angle + MiniPC_instance.receive_data.data.Ref_pitch;
        return;
    }
    // 根据输入模式计算云台目标角度增量
    if (robot_cmd.Control_mode == CONTROL_REMOTE)
    {
        yaw_delta = DR16_instance.control_data.Normalize_ch0 * RC_YAW_SENSITIVITY;
        pitch_delta = DR16_instance.control_data.Normalize_ch1 * RC_PITCH_SENSITIVITY;
    }
    else
    {
        yaw_delta = DR16_instance.control_data.Postion_x * MOUSE_YAW_SENSITIVITY;
        pitch_delta = -DR16_instance.control_data.Postion_y * MOUSE_PITCH_SENSITIVITY;
    }
    // NORMAL模式必须使用机械角闭环；FOLLOW/AIM模式在IMU在线时使用陀螺仪参考
    use_mech_ref = (robot_cmd.gimbal_mode == GIMBAL_RUNNING_NORMAL || !imu_online) ? 1U : 0U;

    if (!use_mech_ref)
    {
        robot_cmd.Gyro_Yaw += yaw_delta;
        robot_cmd.Gyro_Pitch += pitch_delta;
        return;
    }
    
    robot_cmd.Mech_Yaw += (int32_t)(yaw_delta * MECH_YAW_SENSITIVITY);
    robot_cmd.Mech_Pitch = (uint16_t)limit(
        (float)((int32_t)robot_cmd.Mech_Pitch + (int32_t)(pitch_delta * MECH_PITCH_SENSITIVITY)),
        (float)MCH_UP_limit,
        (float)MCH_DOWN_limit);
}

/**
 * @brief 下板通信更新 - 根据当前命令计算底盘速度并发送状态包
 */
static void Chassis_Comm_Update(void)
{
    if (can_comm_instance.can_comm_online_flag == 0)
        return;
    // 更新状态包
    Gimabl_To_Chassis_Action.Gimbal_Online = gimbal_online;
    Gimabl_To_Chassis_Action.Shoot_Online = shoot_online;
    Gimabl_To_Chassis_Action.Vision_Online = MiniPC_instance.MiniPC_Online_Flag;
    Gimabl_To_Chassis_Action.Move_Status = (uint8_t)robot_cmd.chassis_mode;
    Gimabl_To_Chassis_Action.Shoot_Mode = (uint8_t)robot_cmd.shoot_mode;
    Gimabl_To_Chassis_Action.Break_Limitation = Gimbal_To_Chassis.Unlimit_flag;
    Gimbal_To_Chassis.Unlimit_flag = (robot_cmd.Unlimit_flag ? 1U : 0U);

    if (robot_cmd.robot_state == ROBOT_STOP)
    {
        // 停止状态下底盘不动，并且发送停止指令
        Gimbal_To_Chassis.vx = 0;
        Gimbal_To_Chassis.vy = 0;
        Gimbal_To_Chassis.rotate = 0;
        Gimbal_To_Chassis.Unlimit_flag = 0;
        Gimbal_To_Chassis.Close_flag = 1;
    }
    else
    {
        static const float LEVEL_GAIN = 2.0f;
        Chassis_speed_s planningV = {0};
        static Chassis_speed_s slopeningV = {0};

        Gimbal_To_Chassis.Close_flag = 0;

        //  遥控器模式根据摇杆输入计算底盘速度，键鼠模式根据按键输入计算底盘速度
        if (robot_cmd.Control_mode == CONTROL_REMOTE)
        {
            planningV.Y = DR16_instance.control_data.Normalize_ch2 * LEVEL_GAIN * 2.0f;
            planningV.X = DR16_instance.control_data.Normalize_ch3 * LEVEL_GAIN;
            planningV.Omega = DR16_instance.control_data.Normalize_ch1 * LEVEL_GAIN;
        // 拨轮控制小陀螺
        if (fabsf(DR16_instance.control_data.Normalize_wheel) > 0.08f)
        {
        robot_cmd.chassis_mode = CHASSIS_RUNNING_SPIN;
        robot_cmd.rotate_speed = DR16_instance.control_data.Normalize_wheel * (float)rotate_speed_MAX;
        }
        }
        else // 键鼠模式直接使用按键控制速度，Shift加速，Q小陀螺
        {
            int32_t key_x = (DR16_instance.control_data.keys.bits.W ? 1 : 0) -
                            (DR16_instance.control_data.keys.bits.S ? 1 : 0);
            int32_t key_y = (DR16_instance.control_data.keys.bits.A ? 1 : 0) -
                            (DR16_instance.control_data.keys.bits.D ? 1 : 0);
            float speed_scale = LEVEL_GAIN + (robot_cmd.Speed_Up_flag ? 1.0f : 0.0f);
            planningV.X = (float)key_x * speed_scale;
            planningV.Y = (float)key_y * speed_scale;
        }

        // 小陀螺模式增加旋转速度和线速度限制
        if (robot_cmd.chassis_mode == CHASSIS_RUNNING_SPIN)
        {
            planningV.Omega = robot_cmd.rotate_speed / (float)rotate_speed_MAX;
            planningV.X *= 1.4f;
            planningV.Y *= 1.4f;
        }

        // 计算底盘相对于云台的速度，考虑云台当前角度进行坐标变换
        float chassis_offset = (float)(gimbal.yaw_motor->Data.DJI_data.MechanicalAngle - Yaw_Mid_Front) / 1303.80f;
        float sin_theta = arm_sin_f32(chassis_offset);
        float cos_theta = arm_cos_f32(chassis_offset);

        // 使用斜坡规划器平滑速度变化，防止急停急转
        slopeningV.X = Slope_Calc(&slope_X, planningV.X, slopeningV.X);
        slopeningV.Y = Slope_Calc(&slope_Y, planningV.Y, slopeningV.Y);
        slopeningV.Omega = Slope_Calc(&slope_Omega, planningV.Omega, slopeningV.Omega);

        // 坐标变换后发送给底盘，速度单位转换为mm/s和mrad/s
        Gimbal_To_Chassis.vx = (int16_t)((slopeningV.Y * cos_theta - slopeningV.X * sin_theta) * 750.0f);
        Gimbal_To_Chassis.vy = (int16_t)((slopeningV.Y * sin_theta + slopeningV.X * cos_theta) * 750.0f);
        Gimbal_To_Chassis.rotate = (int16_t)(slopeningV.Omega * 2000.0f);
    }

    // 发送CAN数据给底盘
    CAN_Send(can_comm_instance.can_comm, (uint8_t *)&Gimbal_To_Chassis, 10.0f);
    CAN_Send(can_comm2, (uint8_t *)&Gimabl_To_Chassis_Action, 10.0f);
}


