#include "chassis.h"
#include "string.h"
#include <math.h>

/* 4轮全向底盘的轮子位置（单位：米）
 *       FR (1)
 *     /        \
 *    /          \
 * FL(0)        (2)RR
 *    \          /
 *     \        /
 *      BL (3)
 */

// 轮子到中心的距离
#define WHEEL_BASE 0.3f    // 前后轮距离
#define WHEEL_TRACK 0.3f   // 左右轮距离
#define WHEEL_RADIUS 0.05f // 轮子半径

// 底盘全局实例
static Chassis_Instance_Typedef chassis_instance = {0};
static uint8_t chassis_init_flag = 0;

/**
 * @brief 麦克纳姆轮运动学解算
 *        计算底盘运动参数(vx, vy, wz)到电机速度的映射
 * 
 * @param chassis 底盘实例
 * @note  
 *   电机编号：
 *   0 - FL (Front Left)   左前
 *   1 - FR (Front Right)  右前
 *   2 - RR (Rear Right)   右后
 *   3 - RL (Rear Left)    左后
 */
static void Chassis_Mecanum_Solve(Chassis_Instance_Typedef *chassis)
{
    float vx = chassis->motion.vx;
    float vy = chassis->motion.vy;
    float wz = chassis->motion.wz;
    
    // 轴距和轨距的一半
    float half_L = (WHEEL_BASE + WHEEL_TRACK) / 2.0f;
    
    // 麦克纳姆轮速度解算矩阵
    // 每个轮子的速度 = (vx + vy + wz*L)
    float motor_speeds[CHASSIS_MOTOR_NUM];
    
    // FL (0): 左前轮
    motor_speeds[0] = vx - vy - wz * half_L;
    // FR (1): 右前轮
    motor_speeds[1] = vx + vy + wz * half_L;
    // RR (2): 右后轮
    motor_speeds[2] = vx - vy + wz * half_L;
    // RL (3): 左后轮
    motor_speeds[3] = vx + vy - wz * half_L;
    
    // 将速度转换为电机转速目标值（RPM）
    // 假设1m/s对应的电机转速为某个值，这里简化处理
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++)
    {
        // 速度(m/s)转转速(RPM): speed_rpm = (vx_m_s / wheel_circumference) * 60
        // wheel_circumference = 2*pi*R = 2*3.14159*0.05 ≈ 0.314
        chassis->speed_resolve.motor_speed[i] = (int16_t)(motor_speeds[i] * 60 / (2 * 3.14159f * WHEEL_RADIUS));
    }
}

/**
 * @brief 计算底盘实时功率
 */
static float Chassis_Calculate_Power(Chassis_Instance_Typedef *chassis)
{
    float total_power = 0.0f;
    float total_current = 0.0f;
    
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++)
    {
        if (chassis->motor[i] != NULL)
        {
            // 电流(A) = raw_current / 16384 * 20
            float current = chassis->motor[i]->Data.CurrentFilter * 20.0f / 16384.0f;
            float speed = chassis->motor[i]->Data.SpeedFilter;
            
            total_current += fabs(current);
            
            // 功率 = 转速 * 电流（简化模型，实际应该使用电机常数）
            // P = (rpm/1000) * current (近似计算)
            total_power += (fabs(speed) / 1000.0f) * fabs(current);
        }
    }
    
    chassis->feedback.total_current = total_current;
    chassis->motion.power = total_power;
    
    return total_power;
}

/**
 * @brief 底盘功率限制算法
 *        当功率超过限制时，按比例缩小电机目标速度
 */
static void Chassis_Power_Limit(Chassis_Instance_Typedef *chassis)
{
    float current_power = Chassis_Calculate_Power(chassis);
    
    // 如果功率超过限制，进行降速
    if (current_power > CHASSIS_POWER_LIMIT)
    {
        float scale = CHASSIS_POWER_LIMIT / (current_power + 1.0f);
        
        chassis->motion.vx *= scale;
        chassis->motion.vy *= scale;
        chassis->motion.wz *= scale;
        
        // 重新解算速度
        Chassis_Mecanum_Solve(chassis);
    }
    
    // 更新功率缓冲
    chassis->power_buffer = CHASSIS_POWER_LIMIT - current_power;
}

/**
 * @brief  初始化底盘
 */
Chassis_Instance_Typedef *Chassis_Init(void)
{
    if (chassis_init_flag)
        return &chassis_instance;
    
    // 获取已初始化的电机实例
    // 假设电机已经在其他地方初始化，这里直接绑定
    // 实际应用中应该在这里初始化电机或从外部获取
    
    // 初始化PID控制器
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++)
    {
        // 速度PID参数（可根据实际调整）
        chassis_instance.speed_pid[i].Kp = 20.0f;
        chassis_instance.speed_pid[i].Ki = 0.5f;
        chassis_instance.speed_pid[i].Kd = 0.0f;
        chassis_instance.speed_pid[i].outlimit = 16000;     // DJI电机最大输出
        chassis_instance.speed_pid[i].interlimit = 5000;    // 积分限幅
        chassis_instance.speed_pid[i].DeadBand = 10.0f;     // 死区
    }
    
    // 初始化底盘参数
    chassis_instance.chassis_mode = CHASSIS_RELAX;
    chassis_instance.motion.vx = 0.0f;
    chassis_instance.motion.vy = 0.0f;
    chassis_instance.motion.wz = 0.0f;
    chassis_instance.motion.power = 0.0f;
    chassis_instance.power_buffer = CHASSIS_POWER_LIMIT;
    chassis_instance.chassis_online_flag = 1;
    
    memset(&chassis_instance.speed_resolve, 0, sizeof(Chassis_Speed_Resolve_Typedef));
    memset(&chassis_instance.feedback, 0, sizeof(Chassis_Force_Feedback_Typedef));
    
    chassis_init_flag = 1;
    return &chassis_instance;
}

/**
 * @brief  底盘控制更新
 */
void Chassis_Control_Update(Chassis_Instance_Typedef *chassis, float vx, float vy, float wz)
{
    if (chassis == NULL)
        return;
    
    if (chassis->chassis_mode == CHASSIS_RELAX)
    {
        chassis->motion.vx = 0.0f;
        chassis->motion.vy = 0.0f;
        chassis->motion.wz = 0.0f;
        return;
    }
    
    // 更新运动参数
    chassis->motion.vx = vx;
    chassis->motion.vy = vy;
    chassis->motion.wz = wz;
    
    // 功率控制
    Chassis_Power_Limit(chassis);
    
    // 速度解算
    Chassis_Mecanum_Solve(chassis);
    
    chassis->chassis_last_update_time = DWT_GetTime_ms();
}

/**
 * @brief  底盘速度解算
 */
void Chassis_Speed_Resolve(Chassis_Instance_Typedef *chassis)
{
    if (chassis == NULL)
        return;
    
    Chassis_Mecanum_Solve(chassis);
}

/**
 * @brief  底盘功率控制
 */
void Chassis_Power_Control(Chassis_Instance_Typedef *chassis)
{
    if (chassis == NULL)
        return;
    
    Chassis_Power_Limit(chassis);
}

/**
 * @brief  底盘力控制（电机电流环控制）
 *         根据目标速度计算电机输出电流
 */
void Chassis_Force_Control(Chassis_Instance_Typedef *chassis)
{
    if (chassis == NULL)
        return;
    
    int16_t output[CHASSIS_MOTOR_NUM] = {0};
    
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++)
    {
        if (chassis->motor[i] == NULL)
            continue;
        
        // 获取电机当前速度
        float current_speed = chassis->motor[i]->Data.SpeedFilter;
        float target_speed = (float)chassis->speed_resolve.motor_speed[i];
        float error = target_speed - current_speed;
        
        // 计算PID输出
        PID *pid = &chassis->speed_pid[i];
        pid->last_measure = current_speed;
        
        // 简单的PID计算（标准位置式）
        pid->error_last = error;
        float P = pid->Kp * error;
        
        pid->ITerm += pid->Ki * error;
        if (pid->ITerm > pid->interlimit)
            pid->ITerm = pid->interlimit;
        if (pid->ITerm < -pid->interlimit)
            pid->ITerm = -pid->interlimit;
        
        float D = pid->Kd * (error - pid->error_last);
        
        pid->pid_out = P + pid->ITerm + D;
        
        // 输出限幅
        if (pid->pid_out > pid->outlimit)
            pid->pid_out = pid->outlimit;
        if (pid->pid_out < -pid->outlimit)
            pid->pid_out = -pid->outlimit;
        
        output[i] = (int16_t)pid->pid_out;
        
        // 记录反馈力
        chassis->feedback.force[i] = pid->pid_out;
    }
    
    // 发送电机控制指令
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++)
    {
        if (chassis->motor[i] != NULL)
        {
            DJI_Motor_CAN_TxMessage(chassis->motor[i], &output[i]);
        }
    }
}

/**
 * @brief  底盘在线检测
 */
void Chassis_AliveCheck(Chassis_Instance_Typedef *chassis)
{
    if (chassis == NULL)
        return;
    
    uint8_t all_online = 1;
    uint32_t current_time = DWT_GetTime_ms();
    
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++)
    {
        if (chassis->motor[i] == NULL)
        {
            all_online = 0;
            break;
        }
        
        if (chassis->motor[i]->dji_motor_online_flag == 0)
        {
            all_online = 0;
            break;
        }
        
        // 检测通信超时（>200ms未收到数据）
        uint32_t lost_time = current_time - chassis->motor[i]->djimotor_last_rx_ms;
        if (lost_time > 200)
        {
            all_online = 0;
            break;
        }
    }
    
    chassis->chassis_online_flag = all_online;
    
    // 如果离线，停止所有电机
    if (!all_online)
    {
        Chassis_Stop(chassis);
    }
}

/**
 * @brief  底盘模式设置
 */
void Chassis_Set_Mode(Chassis_Instance_Typedef *chassis, Chassis_Mode_Typedef mode)
{
    if (chassis == NULL)
        return;
    
    chassis->chassis_mode = mode;
    
    if (mode == CHASSIS_RELAX)
    {
        Chassis_Stop(chassis);
    }
}

/**
 * @brief  底盘停止
 */
void Chassis_Stop(Chassis_Instance_Typedef *chassis)
{
    if (chassis == NULL)
        return;
    
    for (int i = 0; i < CHASSIS_MOTOR_NUM; i++)
    {
        if (chassis->motor[i] != NULL)
        {
            DJIMotorStop(chassis->motor[i]);
        }
    }
    
    chassis->motion.vx = 0.0f;
    chassis->motion.vy = 0.0f;
    chassis->motion.wz = 0.0f;
    memset(&chassis->speed_resolve, 0, sizeof(Chassis_Speed_Resolve_Typedef));
}

/**
 * @brief 绑定电机实例到底盘
 * @param chassis 底盘实例
 * @param motor_idx 电机索引 (0-3)
 * @param motor 电机实例指针
 */
void Chassis_Bind_Motor(Chassis_Instance_Typedef *chassis, uint8_t motor_idx, DJI_Motor_Instance *motor)
{
    if (chassis == NULL || motor_idx >= CHASSIS_MOTOR_NUM)
        return;
    
    chassis->motor[motor_idx] = motor;
}
