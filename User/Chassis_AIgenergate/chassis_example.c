/**
 * @file chassis_example.c
 * @brief 底盘控制使用示例
 * @note 这是一个参考示例，展示如何在项目中集成底盘控制
 */

#include "chassis.h"
#include "DJIMOTOR.h"

// 底盘实例指针（全局）
static Chassis_Instance_Typedef *g_chassis = NULL;

/**
 * @brief 底盘初始化示例
 */
void Chassis_System_Init(void)
{
    // 第1步：初始化4个电机（这些应该在DJIMOTOR模块中完成）
    DJI_Motor_Config motor_config[4] = {
        // 电机0：左前 (FL)
        {
            .Type = DJI3508,
            .Control_Type = SPEED_LOOP,
            .Can_Config = {/* CAN配置信息 */}
        },
        // 电机1：右前 (FR)
        {
            .Type = DJI3508,
            .Control_Type = SPEED_LOOP,
            .Can_Config = {/* CAN配置信息 */}
        },
        // 电机2：右后 (RR)
        {
            .Type = DJI3508,
            .Control_Type = SPEED_LOOP,
            .Can_Config = {/* CAN配置信息 */}
        },
        // 电机3：左后 (RL)
        {
            .Type = DJI3508,
            .Control_Type = SPEED_LOOP,
            .Can_Config = {/* CAN配置信息 */}
        }
    };

    // 初始化电机实例
    DJI_Motor_Instance *motors[4];
    for (int i = 0; i < 4; i++)
    {
        motors[i] = DJI_Motor_Init(&motor_config[i]);
    }

    // 第2步：初始化底盘
    g_chassis = Chassis_Init();

    // 第3步：绑定电机到底盘
    for (int i = 0; i < 4; i++)
    {
        Chassis_Bind_Motor(g_chassis, i, motors[i]);
    }

    // 第4步：设置工作模式
    Chassis_Set_Mode(g_chassis, CHASSIS_NORMAL);
}

/**
 * @brief 底盘控制周期函数（建议周期：5ms或更快，即200Hz以上）
 * @param vx 前后速度 (m/s)，正值为前进
 * @param vy 左右速度 (m/s)，正值为向右
 * @param wz 旋转角速度 (rad/s)，正值为逆时针
 */
void Chassis_Control_Periodic(float vx, float vy, float wz)
{
    if (g_chassis == NULL)
        return;

    // 第1步：更新控制指令（包含速度解算和功率控制）
    Chassis_Control_Update(g_chassis, vx, vy, wz);

    // 第2步：功率控制（通常已在Update中执行，但也可以单独调用）
    Chassis_Power_Control(g_chassis);

    // 第3步：执行力控制（PID速度环，输出电机电流指令）
    Chassis_Force_Control(g_chassis);

    // 第4步：在线检测（推荐每10ms执行一次）
    static uint16_t alive_check_cnt = 0;
    if (++alive_check_cnt >= 2) // 每10ms检查一次（5ms*2）
    {
        Chassis_AliveCheck(g_chassis);
        alive_check_cnt = 0;
    }
}

/**
 * @brief 获取底盘实时功率（用于上位机监控）
 */
float Chassis_Get_Current_Power(void)
{
    if (g_chassis == NULL)
        return 0.0f;
    return g_chassis->motion.power;
}

/**
 * @brief 获取底盘功率余量
 */
float Chassis_Get_Power_Buffer(void)
{
    if (g_chassis == NULL)
        return 0.0f;
    return g_chassis->power_buffer;
}

/**
 * @brief 查询底盘在线状态
 */
uint8_t Chassis_Is_Online(void)
{
    if (g_chassis == NULL)
        return 0;
    return g_chassis->chassis_online_flag;
}

/**
 * @brief 获取底盘实例指针（给其他模块使用）
 */
Chassis_Instance_Typedef *Chassis_Get_Instance(void)
{
    return g_chassis;
}

/**
 * @brief 底盘模式切换示例
 */
void Chassis_Mode_Switch_Example(uint8_t mode_cmd)
{
    if (g_chassis == NULL)
        return;

    switch (mode_cmd)
    {
    case 0: // 放松模式
        Chassis_Set_Mode(g_chassis, CHASSIS_RELAX);
        break;

    case 1: // 普通底盘移动
        Chassis_Set_Mode(g_chassis, CHASSIS_NORMAL);
        // 然后在Chassis_Control_Periodic中调用相应的vx, vy, wz值
        break;

    case 2: // 云台跟随模式
        Chassis_Set_Mode(g_chassis, CHASSIS_FOLLOW_GIMBAL);
        break;

    case 3: // 旋转模式
        Chassis_Set_Mode(g_chassis, CHASSIS_SPIN);
        // 只设置wz值
        break;

    default:
        break;
    }
}

/**
 * @brief 速度指令生成示例（来自遥控器或自动控制）
 */
void Chassis_Velocity_Command_Example(void)
{
    static float cmd_vx = 0.0f;
    static float cmd_vy = 0.0f;
    static float cmd_wz = 0.0f;

    // 模拟从遥控器或其他控制源获取速度指令
    // 实际应用中应从遥控器、AI等来源获取

    // 例如：遥控器摇杆范围 -1 ~ 1，转换为速度范围 -1.0 ~ 1.0 m/s
    // cmd_vx = remote_controller.rx * 1.0f;  // 前后速度
    // cmd_vy = remote_controller.ry * 1.0f;  // 左右速度
    // cmd_wz = remote_controller.rz * 3.14f; // 旋转速度(最大π rad/s)

    // 调用控制周期函数
    Chassis_Control_Periodic(cmd_vx, cmd_vy, cmd_wz);
}

/**
 * @brief 调试函数：打印底盘状态
 */
void Chassis_Debug_Print(void)
{
    if (g_chassis == NULL)
        return;

    // printf("=== Chassis Status ===\n");
    // printf("Online: %d\n", g_chassis->chassis_online_flag);
    // printf("Mode: %d\n", g_chassis->chassis_mode);
    // printf("Vx: %.2f, Vy: %.2f, Wz: %.2f\n", 
    //        g_chassis->motion.vx, g_chassis->motion.vy, g_chassis->motion.wz);
    // printf("Power: %.2fW, Buffer: %.2fW\n",
    //        g_chassis->motion.power, g_chassis->power_buffer);
    // printf("Motor Speeds[RPM]: %d, %d, %d, %d\n",
    //        g_chassis->speed_resolve.motor_speed[0],
    //        g_chassis->speed_resolve.motor_speed[1],
    //        g_chassis->speed_resolve.motor_speed[2],
    //        g_chassis->speed_resolve.motor_speed[3]);
}

/**
 * @brief 紧急停止
 */
void Chassis_Emergency_Stop(void)
{
    if (g_chassis == NULL)
        return;

    Chassis_Set_Mode(g_chassis, CHASSIS_RELAX);
    Chassis_Stop(g_chassis);
}

/* ==================== 任务集成示例 ==================== */

/**
 * @brief 底盘控制任务（建议在FreeRTOS中运行，周期5ms）
 * 
 * void Chassis_Control_Task(void *param)
 * {
 *     Chassis_System_Init();  // 初始化
 *     
 *     while (1)
 *     {
 *         // 获取遥控器或AI指令
 *         float vx = 0.0f, vy = 0.0f, wz = 0.0f;
 *         // ... 从其他任务获取速度指令 ...
 *         
 *         // 执行底盘控制
 *         Chassis_Control_Periodic(vx, vy, wz);
 *         
 *         // 实时监控
 *         if (Chassis_Is_Online() == 0)
 *         {
 *             // 处理底盘离线情况
 *         }
 *         
 *         vTaskDelay(pdMS_TO_TICKS(5));  // 5ms周期
 *     }
 * }
 */
