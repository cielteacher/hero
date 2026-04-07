/**
 * @file    Gimbal_Cmd.c
 * @brief   云台决策层 - 输入解析与模式管理
 * 
 * @note    【模块职责】
 *          - 遥控器/键鼠输入解析
 *          - 云台/底盘/发射模式决策
 *          - 目标角度更新 (Gyro_/Mech_)
 *          - 板间CAN通信
 *          - 热量/卡弹检测
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

/* ==================== 内部函数声明 ==================== */
static void Gimbal_Board_Update(void);
static void Chassis_Board_Update(void);
static void RCUpdate(void);
static void KMUpdate(void);
static void Jam_Check(void);
static void Heat_Check(void);
static void UpdateGimbalRef(void);

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

/**
 * @brief 检查是否允许视觉自瞄
 * @return 1=允许 (视觉在线且距离有效)
 */
uint8_t VisionCanAutoAim(void)
{
    return (MiniPC_instance.MiniPC_Online_Flag &&
            MiniPC_instance.receive_data.data.dis <= 0.2f) ? 1U : 0U;
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
}

/* ==================== 机器人状态更新 ==================== */
/**
 * @brief 机器人状态更新主函数 - 1ms周期
 */
void Robot_Update(void)
{
    // 获取在线状态
    uint8_t remote_online = (DR16_instance.dr16_online_flag && !DR16_instance.dr16_data_error_flag) ? 1U : 0U;
    uint8_t chassis_online = can_comm_instance.can_comm_online_flag ? 1U : 0U;
    uint8_t gimbal_online = IsGimbalModuleOnline();
    uint8_t shoot_online = IsShootModuleOnline();

    // 设置默认状态
    robot_cmd.robot_state = ROBOT_RUNNING;
    robot_cmd.gimbal_mode = GIMBAL_RUNNING_FOLLOW;
    robot_cmd.chassis_mode = CHASSIS_RUNNING_FOLLOW;
    robot_cmd.shoot_mode = SHOOT_STOP;
    robot_cmd.Speed_Up_flag = false;
    
    // 更新状态包
    Gimabl_To_Chassis_Action.Gimbal_Online = gimbal_online;
    Gimabl_To_Chassis_Action.Shoot_Online = shoot_online;
    Gimabl_To_Chassis_Action.Vision_Online = MiniPC_instance.MiniPC_Online_Flag;

    // 遥控器或底盘通信离线 -> 紧急停止
    if (!remote_online || !chassis_online)
    {
        robot_cmd.robot_state = ROBOT_STOP;
        robot_cmd.gimbal_mode = GIMBAL_STOP;
        robot_cmd.chassis_mode = CHASSIS_STOP;
        robot_cmd.shoot_mode = SHOOT_STOP;
        return;
    }
    
    // 模块离线 -> 对应模块停止
    if (!gimbal_online)
        robot_cmd.gimbal_mode = GIMBAL_STOP;
    if (!shoot_online)
        robot_cmd.shoot_mode = SHOOT_STOP;
    
    // 根据输入模式更新
    switch (DR16_instance.control_data.input_mode)
    {
    case STOP_INPUT:
        robot_cmd.robot_state = ROBOT_STOP;
        break;
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
        break;
    }

    // 停止状态下关闭所有模块
    if (robot_cmd.robot_state == ROBOT_STOP)
    {
        robot_cmd.gimbal_mode = GIMBAL_STOP;
        robot_cmd.chassis_mode = CHASSIS_STOP;
        robot_cmd.shoot_mode = SHOOT_STOP;
        return;
    }

    /**
     * @brief 云台反馈源决策 - 关键决策段
     *
     * 决策优先级:
     * 1. NORMAL模式始终使用机械角反馈 (FB_MECH)
     *    原因: NORMAL模式用于独立手动控制，需要直接编码器反馈
     * 2. IMU离线触发自动降级为FB_MECH
     *    原因: 无IMU情况下，所有模式都只能用机械反馈
     * 3. IMU在线 + 非NORMAL模式使用陀螺仪反馈 (FB_GYRO)
     *    原因: 跟随/自瞄/小陀螺模式需要IMU支持
     *
     * 逻辑流程:
     *   if (NORMAL模式) || (IMU离线)
     *       -> FB_MECH (机械角反馈)
     *   else
     *       -> FB_GYRO (陀螺仪反馈)
     *
     * 更新策略:
     *   - Cmd层: 每周期做一次决策
     *   - Execute层: 直接使用已决策的gimbal_fb
     *   - 无重复: 避免重复检查IMU状态
     */
    if (robot_cmd.gimbal_mode == GIMBAL_RUNNING_NORMAL || !INS_Info.INS_online_flag)
    {
        // 机械角反馈: 用于直接编码器控制
        robot_cmd.gimbal_fb = GIMBAL_FB_MECH;
    }
    else
    {
        // 陀螺仪反馈: 用于IMU力量的稳定控制
        robot_cmd.gimbal_fb = GIMBAL_FB_GYRO;
    }

    // 更新云台目标角度 (同时维护Gyro_*和Mech_*参考值)
    UpdateGimbalRef();
    
    // 更新板间数据
    Gimbal_Board_Update();
    Chassis_Board_Update();
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
    robot_cmd.shoot_mode = SHOOT_STOP;

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

    // 拨轮控制小陀螺
    if (fabsf(DR16_instance.control_data.Normalize_wheel) > 0.08f)
    {
        robot_cmd.gimbal_mode = GIMBAL_RUNNING_SPIN;
        robot_cmd.chassis_mode = CHASSIS_RUNNING_SPIN;
        robot_cmd.rotate_speed = DR16_instance.control_data.Normalize_wheel * (float)rotate_speed_MAX;
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
 *        Q按住  = 小陀螺模式
 *        右键长按 = 自瞄
 *        左键点击 = 开火
 *        Shift  = 加速
 */
static void KMUpdate(void)
{
    // 静态变量保存toggle状态
    static uint8_t fric_toggle = 0;
    static uint8_t unlimit_toggle = 0;
    static uint8_t last_fric_cnt = 0;
    static uint8_t last_unlimit_cnt = 0;
    static uint8_t last_reset_cnt = 0;
    static uint8_t last_turn_cnt = 0;

    // Ctrl+R: 系统复位
    uint8_t reset_cnt = DR16_instance.control_data.key_count[KEY_PRESS_WITH_CTRL][key_R];
    if (reset_cnt != last_reset_cnt)
    {
        last_reset_cnt = reset_cnt;
        NVIC_SystemReset();
    }

    // Ctrl+G: 底盘解限切换
    uint8_t unlimit_cnt = DR16_instance.control_data.key_count[KEY_PRESS_WITH_CTRL][key_G];
    if (unlimit_cnt != last_unlimit_cnt)
    {
        last_unlimit_cnt = unlimit_cnt;
        unlimit_toggle ^= 1U;
    }

    // B: 摩擦轮切换
    uint8_t fric_cnt = DR16_instance.control_data.key_count[KEY_PRESS][key_B];
    if (fric_cnt != last_fric_cnt)
    {
        last_fric_cnt = fric_cnt;
        fric_toggle ^= 1U;
    }

    // 默认: 跟随模式
    robot_cmd.gimbal_mode = GIMBAL_RUNNING_FOLLOW;
    robot_cmd.chassis_mode = unlimit_toggle ? CHASSIS_RUNNING_UNLIMIT : CHASSIS_RUNNING_FOLLOW;
    robot_cmd.shoot_mode = fric_toggle ? SHOOT_READY : SHOOT_STOP;

    // Shift: 加速
    robot_cmd.Speed_Up_flag = DR16_instance.control_data.keys.bits.Shift ? true : false;

    // F: 一键掉头 (Yaw+180度)
    uint8_t turn_cnt = DR16_instance.control_data.key_count[KEY_PRESS][key_F];
    if (turn_cnt != last_turn_cnt)
    {
        last_turn_cnt = turn_cnt;
        robot_cmd.Mid_mode = MID_NONE;  // 触发就近归中
        robot_cmd.Gyro_Yaw += 180.0f;
    }

    // E按住: 底盘分离模式
    if (DR16_instance.control_data.keys.bits.E && DR16_instance.control_data.last_keys.bits.E)
    {
        robot_cmd.gimbal_mode = GIMBAL_RUNNING_NORMAL;
        robot_cmd.chassis_mode = unlimit_toggle ? CHASSIS_RUNNING_UNLIMIT : CHASSIS_RUNNING_NORMAL;
    }

    // Q按住: 小陀螺模式
    if (DR16_instance.control_data.keys.bits.Q && DR16_instance.control_data.last_keys.bits.Q)
    {
        robot_cmd.gimbal_mode = GIMBAL_RUNNING_SPIN;
        robot_cmd.chassis_mode = CHASSIS_RUNNING_SPIN;
    }

    // 右键长按: 自瞄模式
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
            robot_cmd.gimbal_mode = GIMBAL_RUNNING_FOLLOW;
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

/* ==================== 云台目标角度更新 ==================== */
static void UpdateGimbalRef(void)
{
    if (robot_cmd.gimbal_mode == GIMBAL_RUNNING_AIM && VisionCanAutoAim())
    {
        float vision_yaw = MiniPC_instance.receive_data.data.Ref_yaw;
        float yaw_diff = vision_yaw - INS_Info.Yaw_Angle;
        int32_t round_offset = 0;
        if (yaw_diff < -240.0f) round_offset = 1;
        else if (yaw_diff > 240.0f) round_offset = -1;
        robot_cmd.Gyro_Yaw = (INS_Info.YawRoundCount + round_offset) * 360.0f + vision_yaw;
        robot_cmd.Gyro_Pitch = INS_Info.Pitch_Angle + MiniPC_instance.receive_data.data.Ref_pitch;
        return;
    }
    
    float yaw_delta = 0.0f, pitch_delta = 0.0f;
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
    
    if (robot_cmd.gimbal_mode == GIMBAL_RUNNING_NORMAL)
    {
        robot_cmd.Mech_Yaw += (int32_t)(yaw_delta * MECH_YAW_SENSITIVITY);
        robot_cmd.Mech_Pitch = (uint16_t)limit(
            (float)robot_cmd.Mech_Pitch + pitch_delta * MECH_PITCH_SENSITIVITY,
            (float)MCH_UP_limit, (float)MCH_DOWN_limit);
        return;
    }
    
    robot_cmd.Gyro_Yaw += yaw_delta;
    robot_cmd.Gyro_Pitch += pitch_delta;
}

/* ==================== 云台板数据更新 ==================== */
static void Gimbal_Board_Update(void)
{
    Jam_Check();
    Heat_Check();
    Gimabl_To_Chassis_Action.Move_Status = (uint8_t)robot_cmd.chassis_mode;
    Gimabl_To_Chassis_Action.Shoot_Mode = (uint8_t)robot_cmd.shoot_mode;
    Gimabl_To_Chassis_Action.Break_Limitation = Gimbal_To_Chassis.Unlimit_flag;
}

/**
 * @brief ���̰����ݸ��� (��������ٶȲ�����)
 */
static void Chassis_Board_Update(void)
{
    if (can_comm_instance.can_comm_online_flag == 0)
        return;

    Gimbal_To_Chassis.Unlimit_flag = (robot_cmd.chassis_mode == CHASSIS_RUNNING_UNLIMIT) ? 1U : 0U;

    if (robot_cmd.robot_state == ROBOT_STOP)
    {
        // ֹͣ״̬�������ٶ�
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

        // ���ݿ���ģʽ��ȡ�ٶ�����
        if (robot_cmd.Control_mode == CONTROL_REMOTE)
        {
            planningV.Y = DR16_instance.control_data.Normalize_ch2 * LEVEL_GAIN * 2.0f;
            planningV.X = DR16_instance.control_data.Normalize_ch3 * LEVEL_GAIN;
            planningV.Omega = DR16_instance.control_data.Normalize_ch1 * LEVEL_GAIN;
        }
        else // �������
        {
            int32_t key_x = (DR16_instance.control_data.keys.bits.W ? 1 : 0) -
                            (DR16_instance.control_data.keys.bits.S ? 1 : 0);
            int32_t key_y = (DR16_instance.control_data.keys.bits.A ? 1 : 0) -
                            (DR16_instance.control_data.keys.bits.D ? 1 : 0);
            float speed_scale = LEVEL_GAIN + (robot_cmd.Speed_Up_flag ? 1.0f : 0.0f);
            planningV.X = (float)key_x * speed_scale;
            planningV.Y = (float)key_y * speed_scale;
        }

        // С����ģʽ��ʹ�õ�����ת�ٶȣ��ƶ��ٶȷŴ�
        if (robot_cmd.chassis_mode == CHASSIS_RUNNING_SPIN)
        {
            planningV.Omega = robot_cmd.rotate_speed / (float)rotate_speed_MAX;
            planningV.X *= 1.4f;
            planningV.Y *= 1.4f;
        }

        // ����ת������̨����ϵ -> ��������ϵ
        float chassis_offset = (float)(gimbal.yaw_motor->Data.DJI_data.MechanicalAngle - Yaw_Mid_Front) / 1303.80f;
        float sin_theta = arm_sin_f32(chassis_offset);
        float cos_theta = arm_cos_f32(chassis_offset);

        // б�¹滮ƽ���ٶ�
        slopeningV.X = Slope_Calc(&slope_X, planningV.X, slopeningV.X);
        slopeningV.Y = Slope_Calc(&slope_Y, planningV.Y, slopeningV.Y);
        slopeningV.Omega = Slope_Calc(&slope_Omega, planningV.Omega, slopeningV.Omega);

        // �����������ϵ�µ��ٶ�
        Gimbal_To_Chassis.vx = (int16_t)((slopeningV.Y * cos_theta - slopeningV.X * sin_theta) * 750.0f);
        Gimbal_To_Chassis.vy = (int16_t)((slopeningV.Y * sin_theta + slopeningV.X * cos_theta) * 750.0f);
        Gimbal_To_Chassis.rotate = (int16_t)(slopeningV.Omega * 2000.0f);
    }

    // �����ٶȿ��ư���״̬��
    CAN_Send(can_comm_instance.can_comm, (uint8_t *)&Gimbal_To_Chassis, 10.0f);
    CAN_Send(can_comm2, (uint8_t *)&Gimabl_To_Chassis_Action, 10.0f);
}

/**
 * @brief ������� (����ʱ��ز�����״̬)
 */
static void Jam_Check(void)
{
    static uint16_t stuck_time = 0;

    if (robot_cmd.shoot_mode == SHOOT_FIRE || robot_cmd.shoot_mode == SHOOT_AIM)
    {
        // ��⿨���������ٶȹ��� �� ��������
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

    if (robot_cmd.shoot_mode == SHOOT_FIRE || robot_cmd.shoot_mode == SHOOT_AIM)
    {
        uint16_t remain = can_comm_instance.can_comm_rx_data.heat_limit_remain;
        uint8_t cooling_rate = can_comm_instance.can_comm_rx_data.shoot_barrel_cooling;

        // 剩余热量不足时进入冷却
        if ((float)remain + cooling_rate * 0.001f < SHOOT_UNIT_HEAT_42MM * 1.2f)
        {
            robot_cmd.shoot_mode = SHOOT_COOLING;
        }
    }
}
