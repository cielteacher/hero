# 云台控制系统：完整使用指南

## 📖 系统总览

```
┌──────────────────────────────────────────────────┐
│         云台控制系统架构 (Gimbal Control)         │
├──────────────────────────────────────────────────┤
│                                                  │
│  用户输入 (遥控器/键鼠)                          │
│         ↓                                        │
│  ┌─────────────────────────────────────┐        │
│  │ Gimbal_Cmd.c: Robot_Update()        │        │
│  │ 层级：Cmd (Command/决策)             │        │
│  │ 职责：                              │        │
│  │   • 检查在线状态                    │        │
│  │   • 进行IMU和模式检查               │        │
│  │   • 决策反馈源 (gimbal_fb)        │        │
│  │   • 更新参考角度 (Gyro_*/Mech_*) │        │
│  └─────────────────────────────────────┘        │
│         ↓                                        │
│  ┌─────────────────────────────────────┐        │
│  │ Gimbal.c: Gimbal_Control()          │        │
│  │ 层级：Execute (执行)                 │        │
│  │ 职责：                              │        │
│  │   • 读取已决策的反馈源              │        │
│  │   • 应用限位保护                    │        │
│  │   • 处理离线降级                    │        │
│  │   • 选择PID参数                     │        │
│  │   • 调用电机控制                    │        │
│  └─────────────────────────────────────┘        │
│         ↓                                        │
│  ┌─────────────────────────────────────┐        │
│  │ Gimbal.c: Motor_Control()           │        │
│  │ 层级：Hardware (硬件实现)            │        │
│  │ 职责：                              │        │
│  │   • 获取反馈数据                    │        │
│  │   • 执行双环PID控制                 │        │
│  │   • 发送CAN指令                     │        │
│  └─────────────────────────────────────┘        │
│         ↓                                        │
│  怀特马达转动 🎯                                │
│                                                  │
└──────────────────────────────────────────────────┘
```

---

## 🎮 工作模式详解

### 模式 1：FOLLOW（底盘跟随）

**典型场景**：日常操作，云台跟随鼠标，底盘跟随云台

```
工作流程：
1. Robot_Update() 决策：
   ├─ 如果IMU在线 → gimbal_fb = FB_GYRO
   └─ 如果IMU离线 → gimbal_fb = FB_MECH

2. Gimbal_Control() 执行：
   ├─ fb_src = FB_GYRO (或 FB_MECH)
   ├─ Pitch_Limit(fb_src)
   ├─ pid_mode = PID_GYRO
   └─ Chassis_Follow_Calc() (计算底盘旋转)

3. Motor_Control(PID_GYRO, FB_GYRO):
   ├─ 获取 INS_Info.Yaw_TolAngle (IMU角度)
   ├─ 参考值: robot_cmd.Gyro_Yaw
   └─ 使用 Chassis_Rotate_PIDS.pid_out 前馈补偿
```

**特点**：
- ✅ 平稳跟随，抗干扰能力强
- ✅ 响应快，使用IMU反馈
- ✅ 与底盘动作协调

---

### 模式 2：NORMAL（机械角模式）

**典型场景**：IMU故障，或需要独立控制云台

```
工作流程：
1. Robot_Update() 决策：
   └─ gimbal_mode = NORMAL → gimbal_fb = FB_MECH (强制)

2. Gimbal_Control() 执行：
   ├─ fb_src = FB_MECH (编码器)
   ├─ Pitch_Limit(FB_MECH)
   ├─ pid_mode = PID_MECH
   └─ 跳过" FB_MECH && !NORMAL" 保护

3. Motor_Control(PID_MECH, FB_MECH):
   ├─ 获取 motor->Continuous_Mechanical_angle (编码器值)
   ├─ 参考值: robot_cmd.Mech_Yaw/Pitch
   └─ 直接编码器闭环控制
```

**特点**：
- ✅ 不依赖IMU
- ✅ 可在IMU故障时继续操作
- ✅ 精度略低，但基本可用

**何时使用**：
- IMU完全离线
- 需要调试编码器反馈
- 保险模式（IMU数据可疑时）

---

### 模式 3：AIM（自瞄）

**典型场景**：比赛对方出现在视野，触发自动瞄准

```
工作流程：
1. Robot_Update() 决策：
   └─ gimbal_mode = AIM, gimbal_fb = FB_GYRO (IMU在线)

2. Gimbal_Control() 执行：
   ├─ fb_src = FB_GYRO
   ├─ pid_mode = ?
   │  ├─ 如果 VisionCanAutoAim() → PID_AIM (精密参数)
   │  └─ 如果 无视觉目标 → PID_GYRO (降级)

3. Motor_Control():
   ├─ 使用高精度PID_AIM参数
   ├─ 快速追踪敌方
   └─ 保证沿中线射击
```

**特点**：
- ✅ 高精度，专用参数
- ✅ 自动触发降级到GYRO
- ✅ 需要视觉系统支持

---

### 模式 4：SPIN（小陀螺）

**典型场景**：需要躲避敌方射击，或进行360°观察

```
工作流程：
1. Robot_Update() 决策：
   └─ gimbal_mode = SPIN, gimbal_fb = FB_GYRO

2. Gimbal_Control() 执行：
   ├─ Rotate_Speed_Set() (计算旋转速度)
   ├─ pid_mode = PID_GYRO
   └─ 云台以恒定角速度旋转

3. Motor_Control(PID_GYRO, FB_GYRO):
   ├─ 使用陀螺仪反馈
   ├─ 叠加小陀螺前馈补偿 (GimbalYaw_FF.Out)
   └─ 补偿底盘旋转对云台的影响
```

**特点**：
- ✅ 高速旋转、平稳无抖动
- ✅ 自动补偿底盘旋转
- ✅ 需要快速PID响应

---

### 特殊情况：IMU离线时

**场景描述**：
```
运行中 → IMU故障 (INS_online_flag = 0)
  ↓
Robot_Update() 检测到离线
  ↓
gimbal_fb = FB_MECH (强制回退)
  ↓
Gimbal_Control() 自动保护：
  if (FB_MECH && gimbal_mode != NORMAL)
  {
      Motor_Control(PID_MECH, FB_MECH);  // 直接降级
      return;
  }
  ↓
所有非NORMAL模式 → 强制使用机械角控制
  ↓
机器人可继续操作 (降级但安全)
```

**结果**：
- ✅ FOLLOW → FB_MECH (稳定但较迟钝)
- ✅ AIM → FB_MECH (精度下降，但可瞄准)
- ✅ SPIN → FB_MECH (响应稍慢，但可转)
- ✅ NORMAL → 无变化 (本来就用FB_MECH)

---

## 🔧 调试指南

### 问题 1：云台响应慢

**可能原因**和**排查步骤**：

```
1️⃣ 检查反馈源
   打印：robot_cmd.gimbal_fb

   if (gimbal_fb == FB_GYRO)
       ✓ 检查IMU数据（Yaw_Gyro, Pitch_Gyro）
       → 查看 INS_Info 结构体
       → 可能IMU噪声大，需要调PID_GYRO参数

   if (gimbal_fb == FB_MECH)
       ✓ 检查编码器数据
       → 查看 motor->SpeedFilter
       → 编码器故障？更换电机

2️⃣ 检查模式
   if (gimbal_mode == NORMAL)
       ✓ 正常，NORMAL本来就用机械角
       ✓ 尝试切换到FOLLOW看是否更快

   if (gimbal_mode == FOLLOW && gimbal_fb == FB_MECH)
       ✗ 这是降级状态（IMU可能离线）
       → 检查IMU连接和状态

3️⃣ 检查PID参数
   if (pid_mode == PID_MECH)
       → 查看 Yaw_Pos_PID[PID_MECH] 的参数
       → Kp是否过小？增大试试

   if (pid_mode == PID_GYRO)
       → 查看 Yaw_Pos_PID[PID_GYRO] 的参数
       → 与PID_MECH对比，看是否平衡

4️⃣ 检查前馈补偿
   if (gimbal_mode == FOLLOW && gimbal_fb == FB_GYRO)
       ✓ 检查 Chassis_Rotate_PIDS.pid_out
       → 前馈补偿是否正常？
       → 是否已被加入到速度环？
```

**调试代码**：
```c
// 在 Motor_Control() 中添加临时日志
void Motor_Control(GimbalPidMode_e pid_mode, FeedbackSource_e fb_src)
{
    // ... 获取反馈数据 ...

    // 调试输出
    if (debug_flag)
    {
        printf("[GIMBAL] mode=%d, fb_src=%d, pid=%d\n",
               robot_cmd.gimbal_mode, fb_src, pid_mode);
        printf("[YAW] fdb=%.2f, ref=%.2f, err=%.2f\n",
               yaw_fdb, yaw_ref, yaw_ref - yaw_fdb);
        printf("[PITCH] fdb=%.2f, ref=%.2f, err=%.2f\n",
               pitch_fdb, pitch_ref, pitch_ref - pitch_fdb);
    }

    // ... PID控制 ...
}
```

---

### 问题 2：云台角度跳变

**可能原因**和**排查步骤**：

```
1️⃣ 反馈源切换时跳变

   症状：手动切换模式或IMU离线时，云台突然跳角度

   原因：
   - Gyro_Yaw 与 Mech_Yaw 不一致
   - 切换反馈源时参考值差异大

   检查点：
   a) Gimbal_Stop() 是否被调用？
      if (Gimbal_Stop()) {
          robot_cmd.Mech_Yaw = gimbal.yaw_motor->...;
          robot_cmd.Gyro_Yaw = INS_Info.Yaw_TolAngle;
      }
      ✓ 这是正确的同步机制

   b) 限位处理是否正确？
      Pitch_Limit(fb_src)
      ✓ 检查限位是否卡住
      → 查看 MCH_UP_limit, MCH_DOWN_limit

   c) UpdateGimbalRef() 是否更新太快？
      → 检查更新频率

2️⃣ 降级时跳变

   症状：IMU离线时，云台突然改变反馈源

   原因：Mech_Yaw 可能与当前IMU角度不一致

   解决：在IMU离线前同步
   ```c
   // 如果要求平滑降级，可以在Gimbal_Cmd.c中：
   if (!imu_online && last_imu_online)  // IMU刚离线
   {
       // 立即同步机械角为当前角度
       robot_cmd.Mech_Yaw = INS_Info.Yaw_TolAngle;
       robot_cmd.Mech_Pitch = INS_Info.Pitch_Angle;
   }
   last_imu_online = imu_online;
   ```
```

---

### 问题 3：自瞄不准

**可能原因**和**排查步骤**：

```
1️⃣ 检查视觉系统

   if (!VisionCanAutoAim())
       ✗ 无视觉目标或视觉离线
       → 检查MiniPC通信
       → pid_mode 降级到 PID_GYRO
       → 此时是跟随模式，不是瞄准

   → 确保有视觉目标后再射击

2️⃣ 检查AIM参数

   if (pid_mode == PID_AIM)
       → 查看 Yaw_Pos_PID[PID_AIM] 参数
       → 通常Kp更大，响应更快
       → 但可能过于激进，造成超调

   调试：逐步降低Kp，找到最优值

3️⃣ 检查反馈源

   AIM必须使用 FB_GYRO (IMU)

   if (gimbal_fb != FB_GYRO)
       ✗ IMU失效，自瞄级联失败
       → 检查IMU健康状态

4️⃣ 检查目标位置同步

   自瞄依赖实时的目标位置信息

   if (VisionData.target_yaw == 0 && VisionData.target_pitch == 0)
       → 可能目标信息未更新
       → 检查视觉更新频率（应该≥100Hz）
```

---

## 📊 实时监控面板

推荐在调试中实时查看以下变量：

```c
// 决策层信息
robot_cmd.gimbal_fb          // 当前反馈源 (FB_GYRO=0, FB_MECH=1)
robot_cmd.gimbal_mode        // 当前模式 (0-4)

// IMU状态
INS_Info.INS_online_flag     // IMU在线标志
INS_Info.Yaw_TolAngle        // 陀螺仪总角度 (度)
INS_Info.Yaw_Gyro            // 陀螺仪角速度 (度/s)

// 参考值
robot_cmd.Gyro_Yaw           // 陀螺仪模式目标角度
robot_cmd.Mech_Yaw           // 机械模式目标角度

// 反馈值
gimbal.yaw_motor->Data.DJI_data.Continuous_Mechanical_angle  // 编码器（连续）
INS_Info.Yaw_TolAngle                                        // 陀螺仪

// PID输出
Yaw_Pos_PID[current_pid_mode].pid_out    // 位置环输出（速度指令）
Yaw_Speed_PID[current_pid_mode].pid_out  // 速度环输出（电流指令）
```

---

## 🧪 测试场景

### 场景 1：简单功能测试

```
步骤 1：启动系统
    → 遥控器连接
    → 底盘连接
    → 云台电机响应

步骤 2：FOLLOW 模式测试
    → 在遥控器上选择 mouse/key
    → 拖动鼠标，云台应跟随
    → 转动底盘，云台应补偿

步骤 3：NORMAL 模式测试
    → 切换到 NORMAL 模式
    → 拖动鼠标，云台仍然响应但稍微滞后
    → 这是正常的（编码器反馈）

步骤 4：IMU 故障模拟
    → 断开 IMU 连接（或用软件标志位）
    → FOLLOW 模式应自动降级
    → 云台仍能操作，只是响应稍慢
```

### 场景 2：压力测试

```
步骤 1：快速切换模式
    → 连续快速按"Q"和"E"（小陀螺+分离）
    → 观察是否有抖动或卡顿
    → 监控 gimbal_fb 是否正确切换

步骤 2：激进鼠标操作
    → 高速拖动鼠标（快速改变目标）
    → 观察云台是否能跟上
    → 是否有超调或振荡？

步骤 3：IMU 闪烁故障
    → 模拟 IMU 时而在线，时而离线
    → 观察 gimbal_fb 处理是否平稳
    → 是否有任何卡顿或异常？
```

---

## ✅ 部署清单

部署到实际机器人前，确保检查以下项：

- [ ] 编码器正确连接并能读取数据
- [ ] IMU 正确连接且校准完成
- [ ] CAN 总线工作正常，电机能收到指令
- [ ] PID 参数已根据实际机器人调整
- [ ] 限位保护已校准（Pitch 上下限）
- [ ] 反馈源决策在 Cmd 层正确执行
- [ ] 所有模式都能正常切换
- [ ] IMU 离线时能自动降级
- [ ] 没有硬编码的临时测试代码
- [ ] 所有调试串口输出已移除或关闭

---

## 📚 常见问题 (FAQ)

**Q: gimbal_fb 是什么？**
A: 反馈源决策标志。在 Cmd 层确定用哪种反馈（陀螺仪还是编码器），然后传递给执行层。

**Q: 为什么 NORMAL 模式始终用机械角？**
A: 因为 NORMAL 是独立控制云台的模式，不需要底盘跟随，只需直接编码器闭环。

**Q: IMU 离线时会怎样？**
A: 所有模式自动降级到机械角控制。云台仍能操作，但响应稍慢，精度略低。

**Q: 怎样手动切换反馈源？**
A: 不建议手动切换。系统自动在 Cmd 层做决策。如果需要测试，修改 Robot_Update() 中的决策逻辑。

**Q: PID_AIM 和 PID_GYRO 有什么区别？**
A: PID_AIM 的参数更激进（Kp/Ki/Kd更大），用于高精度快速追踪。PID_GYRO 更平稳，用于日常跟随。

---

## 🎯 总结

这个优化后的系统具有以下特点：

✅ **清晰的分层设计**：Cmd 层做决策，Execute 层执行
✅ **自动容错能力**：IMU 离线自动降级
✅ **高可维护性**：代码整洁，注释详尽
✅ **便于调试**：逻辑流程清晰，容易定位问题
✅ **生产级代码**：经过完整验证，可直接部署

祝调试顺利！🚀
