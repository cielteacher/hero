# 底盘控制系统 - 文件索引

## 📂 目录结构

```
Chassis/
├── chassis.h                      # 【核心】头文件 - API接口和数据结构
├── chassis.c                      # 【核心】实现文件 - 完整的控制逻辑
├── chassis_example.c              # 【示例】使用示例代码
├── README.md                      # 【文档】完整的功能和API说明
├── QUICK_REFERENCE.md             # 【文档】快速参考手册
├── IMPLEMENTATION_SUMMARY.md      # 【文档】实现细节和原理
├── DELIVERY_REPORT.txt            # 【文档】交付报告
└── INDEX.md                       # 【文档】本文件
```

## 📖 文件说明

### 核心代码（必需）

#### chassis.h (128 行)

**用途**：头文件，定义所有数据结构和API接口
**包含内容**：
- 宏定义：电机数量、功率限制、电流限制
- 枚举：底盘工作模式
- 数据结构：运动参数、反馈数据、底盘实例
- 函数声明：所有API接口

**关键函数**：
```c
Chassis_Instance_Typedef *Chassis_Init(void);                    // 初始化
void Chassis_Bind_Motor(...);                                     // 绑定电机
void Chassis_Control_Update(chassis, vx, vy, wz);               // 主控制接口
void Chassis_Speed_Resolve(chassis);                             // 速度解算
void Chassis_Power_Control(chassis);                             // 功率控制
void Chassis_Force_Control(chassis);                             // 力控制
void Chassis_AliveCheck(chassis);                                // 在线检测
```

#### chassis.c (370 行)
**用途**：实现文件，包含所有控制逻辑
**关键函数**：
- `Chassis_Mecanum_Solve()` - 麦克纳姆轮运动学解算
- `Chassis_Calculate_Power()` - 实时功率计算
- `Chassis_Power_Limit()` - 功率限制控制
- `Chassis_Init()` - 初始化
- `Chassis_Force_Control()` - PID速度环
- `Chassis_AliveCheck()` - 在线检测

**特点**：
- 完整的算法实现
- 详细的代码注释
- 健壮的错误处理
- 高效的性能优化

### 示例代码（参考）

#### chassis_example.c (164 行)
**用途**：展示如何集成和使用底盘控制系统
**包含**：
- 初始化示例
- 控制周期函数
- 模式切换示例
- 调试函数
- FreeRTOS集成建议

**适用场景**：
- 学习如何使用API
- 快速集成到项目
- 参考任务设计

### 文档文件（说明）

#### README.md (260 行)
**最详细的文档**，包含：
- ✅ 完整的功能说明
- ✅ 麦克纳姆轮数学模型
- ✅ 功率控制算法原理
- ✅ 完整的API文档
- ✅ 参数配置指南
- ✅ 故障排查表
- ✅ 集成建议

**推荐阅读**：首次使用必读

#### QUICK_REFERENCE.md (200 行)
**快速参考卡**，包含：
- 🎯 最常用的函数
- 📏 坐标系和速度单位
- ⚙️ 关键宏定义表
- 🔧 PID参数调整方法
- 📊 电机映射表
- 🚨 故障快速排查表
- 💡 最佳实践

**推荐阅读**：需要快速查找时使用

#### IMPLEMENTATION_SUMMARY.md (200 行)
**实现细节总结**，包含：
- 📋 交付文件清单
- 🎯 实现的核心功能
- 🔧 技术特点
- 📊 关键算法详解
- 🚀 集成步骤
- 📈 性能指标

**推荐阅读**：了解实现细节时使用

#### DELIVERY_REPORT.txt (150 行)
**交付报告**，包含：
- 📦 交付内容清单
- 📋 实现的功能模块
- 🔧 技术特点总结
- 📖 API简览
- 🎯 集成步骤
- ✅ 验收清单

**推荐阅读**：项目交付确认

## 🎯 快速开始指南

### 1️⃣ 新手入门（5分钟）
```
1. 阅读本索引的"快速概览"部分
2. 查看 QUICK_REFERENCE.md 的"最常用函数"
3. 查看 chassis_example.c 的初始化示例
```

### 2️⃣ 深入学习（15分钟）
```
1. 阅读 README.md 的功能说明部分
2. 理解麦克纳姆轮速度解算原理
3. 理解功率控制算法
```

### 3️⃣ 实际集成（30分钟）
```
1. 复制 chassis.h 和 chassis.c 到项目
2. 参考 chassis_example.c 编写初始化代码
3. 在控制循环中调用 Chassis_Control_Update()
4. 根据实际参数调整 PID 和功率限制
```

### 4️⃣ 问题排查（按需）
```
1. 查看 README.md 的"故障排查"表
2. 查看 QUICK_REFERENCE.md 的"故障快速排查表"
3. 启用 chassis_example.c 中的调试函数
```

## 📚 按用途查找文档

### 我想...

#### 快速了解这是什么
→ 读本文件（INDEX.md）的"概述"部分

#### 快速查找API函数
→ 查看 QUICK_REFERENCE.md 或 README.md 的"API文档"

#### 学习如何初始化
→ 查看 chassis_example.c 中的 `Chassis_System_Init()`

#### 了解如何控制底盘
→ 查看 QUICK_REFERENCE.md 的"最常用函数"

#### 学习速度解算原理
→ 查看 README.md 的"麦克纳姆轮速度解算"部分

#### 调整PID参数
→ 查看 QUICK_REFERENCE.md 的"PID参数调整"

#### 处理功率限制
→ 查看 README.md 的"功率控制"部分

#### 排查故障
→ 查看 README.md 或 QUICK_REFERENCE.md 的"故障排查"表

#### 集成到FreeRTOS
→ 查看 chassis_example.c 末尾的"任务集成示例"

#### 了解所有可调参数
→ 查看 README.md 的"配置参数"部分

## 🔗 文件关系图

```
用户代码
    ↓
chassis.h (API接口)
    ↓
chassis.c (实现逻辑)
    ↓
DJImotor.h/c (电机驱动)
    ↓
pid.h/c (PID算法)
    ↓
硬件层 (CAN, DWT等)
```

## ✅ 集成检查清单

使用本代码前，确保已完成以下步骤：

- [ ] 了解了架构和文件结构（阅读本INDEX.md）
- [ ] 查看了快速参考手册（QUICK_REFERENCE.md）
- [ ] 理解了核心API（chassis.h）
- [ ] 查看了使用示例（chassis_example.c）
- [ ] 复制了 chassis.h/c 到项目
- [ ] 项目已包含 DJIMOTOR, pid, bsp_dwt
- [ ] 理解了四个电机的编号和位置
- [ ] 测试了底层电机通信
- [ ] 编译通过，无错误
- [ ] 根据硬件调整了轮子参数
- [ ] 运行了初始化代码
- [ ] 在控制循环中调用了API

## 📞 快速帮助

### 编译错误
- 检查是否包含了所有头文件（DJIMOTOR.h, pid.h）
- 检查是否包含了 bsp_dwt.h 用于 DWT_GetTime_ms()

### 运行时问题
- 底盘不动？→ 检查 chassis_online_flag 和电机通信
- 响应缓慢？→ 增大 Kp 参数
- 频繁降速？→ 增大 CHASSIS_POWER_LIMIT

### 参数调整
- PID参数位置：chassis.c 的 Chassis_Init() 函数
- 功率限制位置：chassis.h 的 #define CHASSIS_POWER_LIMIT
- 轮子参数位置：chassis.c 顶部的宏定义

## 📊 文件统计

| 文件 | 行数 | 类型 | 用途 |
|------|------|------|------|
| chassis.h | 128 | 代码 | 接口定义 |
| chassis.c | 370 | 代码 | 实现 |
| chassis_example.c | 164 | 代码 | 示例 |
| README.md | 260 | 文档 | 详细说明 |
| QUICK_REFERENCE.md | 200 | 文档 | 快速查找 |
| IMPLEMENTATION_SUMMARY.md | 200 | 文档 | 实现总结 |
| DELIVERY_REPORT.txt | 150 | 文档 | 交付确认 |
| **总计** | **~1500** | - | - |

## 🎓 推荐阅读顺序

### 对于有经验的开发者
1. QUICK_REFERENCE.md (5分钟快速上手)
2. chassis.h (理解接口)
3. chassis_example.c (集成示例)
4. 开始编码

### 对于新手
1. README.md (完整理解)
2. QUICK_REFERENCE.md (快速查找)
3. chassis_example.c (学习示例)
4. IMPLEMENTATION_SUMMARY.md (深入理解)
5. 开始编码

### 对于维护者
1. DELIVERY_REPORT.txt (交付清单)
2. IMPLEMENTATION_SUMMARY.md (技术细节)
3. chassis.c (代码审查)
4. README.md (文档审查)

---

**最后更新**：2026-03-31
**版本**：v1.0
**状态**：✅ 完成交付
