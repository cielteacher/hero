# 底盘控制 - 快速参考指南

## 🎯 最常用的函数

```c
// 1. 初始化
Chassis_Instance_Typedef *chassis = Chassis_Init();

// 2. 绑定电机（4次）
Chassis_Bind_Motor(chassis, 0, motor_FL);
Chassis_Bind_Motor(chassis, 1, motor_FR);
Chassis_Bind_Motor(chassis, 2, motor_RR);
Chassis_Bind_Motor(chassis, 3, motor_RL);

// 3. 在控制循环中调用（5ms周期）
Chassis_Control_Update(chassis, vx, vy, wz);
Chassis_Power_Control(chassis);
Chassis_Force_Control(chassis);
Chassis_AliveCheck(chassis);

// 4. 模式切换
Chassis_Set_Mode(chassis, CHASSIS_NORMAL);    // 普通模式
Chassis_Set_Mode(chassis, CHASSIS_RELAX);     // 停止
```

## 📏 坐标系和速度单位

```
前进方向：vx > 0  (m/s)
向右方向：vy > 0  (m/s)  
逆时针转：wz > 0  (rad/s)

典型范围：
  vx:  -1.0 ~ 1.0 m/s
  vy:  -1.0 ~ 1.0 m/s
  wz:  -3.14 ~ 3.14 rad/s (±π)
```

## ⚙️ 关键宏定义（可在chassis.h中修改）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| CHASSIS_POWER_LIMIT | 200.0f | 功率限制(W) |
| CHASSIS_CURRENT_LIMIT | 5000 | 电流限制(mA) |
| WHEEL_BASE | 0.3f | 前后轮距(m) |
| WHEEL_TRACK | 0.3f | 左右轮距(m) |
| WHEEL_RADIUS | 0.05f | 轮子半径(m) |

## 🎮 遥控器指令转换示例

```c
// 遥控器摇杆范围通常是 -1000 ~ 1000
float remote_vx = remote.rx / 1000.0f;   // -1.0 ~ 1.0 m/s
float remote_vy = remote.ry / 1000.0f;   // -1.0 ~ 1.0 m/s
float remote_wz = remote.rz / 1000.0f * 3.14159f;  // -π ~ π rad/s

Chassis_Control_Update(chassis, remote_vx, remote_vy, remote_wz);
```

## 📊 电机映射

| 索引 | 位置 | 简称 | 说明 |
|------|------|------|------|
| 0 | 左前 | FL | Front Left |
| 1 | 右前 | FR | Front Right |
| 2 | 右后 | RR | Rear Right |
| 3 | 左后 | RL | Rear Left |

```
      FR(1)
    /      \
  FL(0)    RR(2)
    \      /
      RL(3)
```

## 🔧 PID参数调整

默认参数适合大多数应用场景：
```c
Kp = 20.0   // 比例系数（快速响应）
Ki = 0.5    // 积分系数（消除偏差）
Kd = 0.0    // 微分系数（阻尼）
```

**调整方法**：
- 速度跟踪缓慢 → 增大Kp（如25.0）
- 速度振荡 → 减小Kp（如15.0）或增大Kd
- 有稳态误差 → 增大Ki（如1.0）

在`Chassis_Init()`函数中修改：
```c
chassis_instance.speed_pid[i].Kp = 20.0f;  // 修改这里
```

## 📈 性能监控

```c
// 获取实时功率
float power = chassis->motion.power;

// 获取功率余量
float buffer = chassis->power_buffer;

// 检查在线状态
if (chassis->chassis_online_flag == 0) {
    // 处理底盘离线
}

// 查看电机转速（RPM）
int16_t motor0_rpm = chassis->speed_resolve.motor_speed[0];

// 查看总电流（A）
float total_current = chassis->feedback.total_current;
```

## 🚨 故障排查快速表

| 问题 | 可能原因 | 检查项 |
|------|--------|--------|
| 底盘不动 | 电机离线 | `chassis_online_flag` |
| 反应缓慢 | PID参数不合适 | 增大Kp |
| 频繁降速 | 功率限制过低 | 增大POWER_LIMIT |
| 电机抖动 | 控制周期太长或PID振荡 | 减小周期到5ms，减小Kp |
| 功率计算异常 | 电流读取错误 | 检查电机CurrentFilter值 |

## 💡 最佳实践

1. **控制周期**：保持在5ms（200Hz）或更快
2. **指令来源**：从遥控器或AI中获取速度指令
3. **故障处理**：在AliveCheck后检查online_flag
4. **参数调整**：先从默认参数开始，逐步微调
5. **监控功率**：定期检查power_buffer确保有余量

## 🔌 集成检查清单

- [ ] 电机已通过DJI_Motor_Init初始化
- [ ] CAN通信正常工作
- [ ] 四个电机都绑定到底盘
- [ ] 控制循环周期为5ms
- [ ] 在线检测执行正常
- [ ] PID参数已按实际需求调整
- [ ] 功率限制值合理设置

## 📚 获取更多信息

- **完整API**：查看 `chassis.h`
- **实现细节**：查看 `chassis.c`
- **使用示例**：查看 `chassis_example.c`
- **详细文档**：查看 `README.md`
- **实现总结**：查看 `IMPLEMENTATION_SUMMARY.md`

---
**版本**：1.0 | **更新**：2026-03-31
