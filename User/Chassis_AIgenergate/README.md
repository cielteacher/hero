# 底盘控制代码文档

## 概述
本模块实现了基于4个DJI电机（2505/3508）驱动的全向底盘控制系统，包含**速度解算**和**功率控制**功能。

## 核心特性

### 1. 麦克纳姆轮速度解算
- **运动学模型**：通过vx、vy、wz三个速度分量计算各电机的目标转速
- **轮子配置**：4轮全向底盘布局
  - 电机0：左前(FL)
  - 电机1：右前(FR)  
  - 电机2：右后(RR)
  - 电机3：左后(RL)

- **速度映射关系**：
```
motor_speed[0] = vx - vy - wz * L
motor_speed[1] = vx + vy + wz * L
motor_speed[2] = vx - vy + wz * L
motor_speed[3] = vx + vy - wz * L
```

其中L为轴距和轨距的平均值，影响旋转速度的权重。

### 2. 功率控制
- **功率限制**：限制底盘总功率不超过200W（可调整）
- **算法原理**：
  1. 实时计算底盘功率 = Σ(|转速|/1000 × |电流|)
  2. 当功率超过限制时，按比例缩小vx、vy、wz
  3. 重新解算电机速度

- **功率缓冲**：记录可用功率余量，便于上层逻辑调度

### 3. 电机控制
- **速度环PID**：每个电机独立PID调节，跟踪目标转速
- **PID参数**：
  - Kp = 20.0 (比例系数)
  - Ki = 0.5 (积分系数)  
  - Kd = 0.0 (微分系数)
  - 输出限幅：±16000

- **死区**：误差<10RPM时不输出，减少抖动

### 4. 在线检测
- 监测所有4个电机的在线状态
- 检测通信超时（>200ms未收到数据）
- 任一电机离线时自动停止底盘

## API 使用

### 初始化
```c
Chassis_Instance_Typedef *chassis = Chassis_Init();
```

### 绑定电机
```c
// 假设电机已通过DJI_Motor_Init初始化
Chassis_Bind_Motor(chassis, 0, motor_fl);
Chassis_Bind_Motor(chassis, 1, motor_fr);
Chassis_Bind_Motor(chassis, 2, motor_rr);
Chassis_Bind_Motor(chassis, 3, motor_rl);
```

### 控制更新（主要接口）
```c
// vx: 前后速度(m/s)  vy: 左右速度(m/s)  wz: 旋转速度(rad/s)
Chassis_Control_Update(chassis, vx, vy, wz);
```

### 执行控制
```c
// 在控制周期内调用
Chassis_Speed_Resolve(chassis);      // 速度解算（在Update中已调用）
Chassis_Power_Control(chassis);      // 功率控制
Chassis_Force_Control(chassis);      // 电机力控制（输出PWM）
Chassis_AliveCheck(chassis);         // 在线检测
```

### 模式切换
```c
Chassis_Set_Mode(chassis, CHASSIS_NORMAL);  // 普通模式
Chassis_Set_Mode(chassis, CHASSIS_SPIN);    // 旋转模式
Chassis_Set_Mode(chassis, CHASSIS_RELAX);   // 放松模式（停止）
```

## 配置参数

### 可调整的宏定义
```c
#define CHASSIS_MOTOR_NUM 4             // 电机数量
#define CHASSIS_POWER_LIMIT 200.0f      // 功率限制(W)
#define CHASSIS_CURRENT_LIMIT 5000      // 电流限制(mA)
#define WHEEL_BASE 0.3f                 // 前后轮距(m)
#define WHEEL_TRACK 0.3f                // 左右轮距(m)
#define WHEEL_RADIUS 0.05f              // 轮子半径(m)
```

### PID参数调整
在`Chassis_Init()`函数中修改每个电机的PID参数：
```c
chassis_instance.speed_pid[i].Kp = 20.0f;       // 比例系数
chassis_instance.speed_pid[i].Ki = 0.5f;        // 积分系数
chassis_instance.speed_pid[i].Kd = 0.0f;        // 微分系数
chassis_instance.speed_pid[i].outlimit = 16000; // 输出限幅
```

## 数据结构

### 运动参数结构体
```c
typedef struct {
    float vx;      // 前后速度 (m/s)
    float vy;      // 左右速度 (m/s)
    float wz;      // 旋转速度 (rad/s)
    float power;   // 实时功率 (W)
} Chassis_Motion_Typedef;
```

### 反馈数据
```c
typedef struct {
    float force[4];        // 各电机的控制力
    float total_current;   // 总电流(A)
} Chassis_Force_Feedback_Typedef;
```

## 集成建议

1. **初始化顺序**：
   - 先初始化DJI电机模块
   - 再调用Chassis_Init()
   - 最后Chassis_Bind_Motor()绑定电机

2. **控制循环**（推荐频率：500Hz或以上）：
   ```c
   Chassis_Control_Update(chassis, vx, vy, wz);
   Chassis_Force_Control(chassis);
   Chassis_AliveCheck(chassis);
   ```

3. **参数调优**：
   - 根据实际电机特性调整PID参数
   - 根据系统功率调整功率限制
   - 根据底盘物理尺寸调整轮子参数

## 注意事项

- 电机必须已初始化并通过CAN通信正常工作
- 速度单位为m/s，旋转速度单位为rad/s
- 功率限制算法会自动缩放速度指令，可能导致实际运动与指令不符
- 在高速旋转时，功率消耗可能快速增加
- 所有浮点计算都是为了在STM32上高效执行

## 故障排查

| 问题 | 原因 | 解决方案 |
|------|------|--------|
| 底盘不动 | 电机未在线 | 检查CAN通信，调用AliveCheck() |
| 速度响应延迟 | PID参数不合适 | 增大Kp，调整Ki/Kd |
| 功率频繁降速 | 功率限制过低 | 增大CHASSIS_POWER_LIMIT |
| 电机抖动 | 死区或PID振荡 | 调整DeadBand或降低Kp |
