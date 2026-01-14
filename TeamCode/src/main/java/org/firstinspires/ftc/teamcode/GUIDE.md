# Prototype2026-Public-2 Operation Guide | 操作指南

> **Bilingual Technical Documentation | 中英双语技术文档**  
> FTC Team 12527 | Last Updated: 2026-01-13

---

## Table of Contents | 目录

1. [Architecture Overview | 架构概览](#1-architecture-overview--架构概览)
2. [Subsystems | 子系统](#2-subsystems--子系统)
   - [MecanumDrivePinpoint | 底盘驱动](#21-mecanumdrivepinpoint--底盘驱动)
   - [Shooter | 发射器](#22-shooter--发射器)
   - [Intake | 进球机构](#23-intake--进球机构)
   - [Transit | 传输机构](#24-transit--传输机构)
   - [Vision | 视觉](#25-vision--视觉)
3. [Commands | 命令](#3-commands--命令)
4. [TeleOp Structure | 手动程序结构](#4-teleop-structure--手动程序结构)
5. [Autonomous Structure | 自动程序结构](#5-autonomous-structure--自动程序结构)
   - [Far Auto | 远起点自动](#51-far-auto--远起点自动)
   - [Near Auto | 近起点自动](#52-near-auto--近起点自动)
6. [Key Algorithms | 核心算法](#6-key-algorithms--核心算法)
7. [Configuration & Tuning | 配置与调参](#7-configuration--tuning--配置与调参)
8. [Control Mapping | 手柄映射](#8-control-mapping--手柄映射)
9. [Complete Method Reference | 完整方法参考](#9-complete-method-reference--完整方法参考)
10. [Troubleshooting Guide | 故障排查指南](#10-troubleshooting-guide--故障排查指南)
11. [Debug Telemetry Reference | 调试遥测参考](#11-debug-telemetry-reference--调试遥测参考)
12. [Quick Debug Checklist | 快速调试清单](#12-quick-debug-checklist--快速调试清单)

---

## 1. Architecture Overview | 架构概览

### Design Philosophy | 设计理念

This codebase follows a **Layered Modular Architecture** with **Command-Based Programming**.

本代码库采用**分层模块化架构**，结合**命令式编程模式**。

```
┌─────────────────────────────────────────────────────────────┐
│                      OpMode (TeleOp/Auto)                   │
│                   OpMode（手动/自动程序）                      │
├─────────────────────────────────────────────────────────────┤
│              Commands (Action Logic)                        │
│              命令层（行为逻辑）                                │
├─────────────────────────────────────────────────────────────┤
│              Subsystems (Hardware Abstraction)              │
│              子系统层（硬件抽象）                              │
├─────────────────────────────────────────────────────────────┤
│              Constants (Configuration)                      │
│              常量层（配置参数）                                │
└─────────────────────────────────────────────────────────────┘
```

### Directory Structure | 目录结构

```
teamcode/
├── subsystems/           # Hardware abstraction | 硬件抽象
│   ├── drive/            # Drivetrain | 底盘
│   ├── shooter/          # Shooter flywheel | 发射器
│   ├── intake/           # Ball intake | 进球
│   ├── transit/          # Ball feeder | 传输
│   └── vision/           # Limelight vision | 视觉
├── commands/             # Action commands | 动作命令
│   └── autocommands/     # Auto-specific commands | 自动专用命令
├── opmodes/              # OpModes | 操作模式
│   ├── teleops/          # TeleOp programs | 手动程序
│   └── autos/            # Autonomous programs | 自动程序
├── controls/             # Gamepad bindings | 手柄绑定
├── utils/                # Utility functions | 工具函数
└── tests/                # Test OpModes | 测试程序
```

### Core Frameworks Used | 使用的核心框架

| Framework | Purpose | 用途 |
|-----------|---------|------|
| **FTCLib** | Command-based architecture | 命令式架构 |
| **Pedro Pathing** | Autonomous path following | 自动路径跟踪 |
| **Pinpoint Odometry** | Robot localization | 机器人定位 |
| **Limelight3A** | AprilTag vision | 视觉识别 |
| **FTC Dashboard** | Real-time tuning | 实时调参 |

---

## 2. Subsystems | 子系统

Each subsystem extends `SubsystemBase` (FTCLib) and implements a `periodic()` method that runs every loop cycle.

每个子系统继承 `SubsystemBase`（FTCLib），并实现每个循环周期都会执行的 `periodic()` 方法。

### 2.1 MecanumDrivePinpoint | 底盘驱动

**File | 文件**: `subsystems/drive/MecanumDrivePinpoint.java`

**Purpose | 用途**: Controls 4 mecanum wheels with GoBilda Pinpoint odometry for localization.

控制4个麦克纳姆轮，使用 GoBilda Pinpoint 里程计进行定位。

#### Key Methods | 关键方法

| Method | Description | 描述 |
|--------|-------------|------|
| `moveRobot(x, y, turn)` | Robot-centric movement | 机器人坐标系移动 |
| `moveRobotFieldRelative(x, y, turn)` | Field-centric movement | 场地坐标系移动 |
| `getAlignTurnPower(vision)` | Auto-aim P controller | 自瞄P控制器 |
| `calculateAdaptiveVelocity(tagId)` | Distance-based velocity | 根据距离计算转速 |
| `calculateAdaptiveServoPosition(tagId)` | Distance-based angle | 根据距离计算角度 |
| `applyBreak()` | Position hold PID | 位置保持PID |

#### Auto-Aim Algorithm | 自瞄算法

The auto-aim system uses a **PID controller** with several enhancements to prevent oscillation:

自瞄系统使用 **PID 控制器**，并加入多项优化以防止抖动：

**1. Low-Pass Filter | 低通滤波**
```java
// Smooth Limelight noise (especially at close range)
// 平滑 Limelight 噪声（近距离尤其明显）
filteredTx = alpha * rawTx + (1 - alpha) * filteredTx;  // alpha = 0.3
```

**2. Hysteresis Deadband | 滞后死区**
```java
// Entry threshold: 3.0° (near) or 0.5° (far)
// Exit threshold: entry + 1.5° (hysteresis)
// Prevents oscillation at deadband boundary
// 进入死区阈值：3.0°(近) 或 0.5°(远)
// 退出死区阈值：进入 + 1.5°（滞后）
// 防止在死区边界抖动
```

**3. Distance-Adaptive PID | 距离自适应 PID**
```java
// Close (<40"): kP = 0.012 (weak, prevent overshoot)
// Far (>100"):  kP = 0.03  (strong, precision)
// Mid: linear interpolation
// 近距离(<40")：kP = 0.012（弱，防过冲）
// 远距离(>100")：kP = 0.03（强，精准）
// 中间距离：线性插值
```

**4. PID Parameters | PID 参数**
| Parameter | Value | Purpose |
|-----------|-------|---------|
| `kP_near` | 0.012 | P gain at close range |
| `kP_far` | 0.03 | P gain at far range |
| `kD_alignH` | 0.008 | D gain (damping) |
| `txFilterAlpha` | 0.3 | Filter coefficient (lower = smoother) |
| `alignHysteresis` | 1.5° | Hysteresis amount |

#### Absolute Position Fusion | 绝对位置融合

The system fuses Vision and Odometry:
- **Vision available**: Update absolute position from Limelight
- **Vision lost**: Dead-reckoning using odometry delta

系统融合视觉和里程计：
- **有视觉数据**：从 Limelight 更新绝对位置
- **视觉丢失**：使用里程计增量进行航位推算

---

### 2.2 Shooter | 发射器

**File | 文件**: `subsystems/shooter/Shooter.java`

**Purpose | 用途**: Controls dual flywheel motors and angle adjustment servo.

控制双飞轮电机和角度调节舵机。

#### State Machine | 状态机

```java
public enum ShooterState {
    STOP(-600,  0.85),   // Idle speed, low angle | 待机转速，低角度
    SLOW(-700,  0.85),   // Near shot | 近射
    MID(-950,   0.29),   // Mid shot | 中射
    FAST(-1420, 0.29);   // Far shot | 远射
}
```

#### Control Algorithm | 控制算法

**Bang-Bang with Feedforward** (not PID):

**Bang-Bang 前馈控制**（不是PID）：

```java
if (currentVel > targetVel) {
    power = 1.0;  // Too slow → max power | 太慢 → 满功率
} else {
    power = |targetVel| / maxVelocityTPS;  // Feedforward | 前馈
}
```

Why not PID? Flywheel momentum makes PID oscillate. Bang-Bang converges faster.

为什么不用PID？飞轮惯性会使PID振荡。Bang-Bang 收敛更快。

#### Adaptive Shooting | 自适应发射

Automatically calculates velocity based on distance to goal.

根据到目标的距离自动计算转速。

**Requirements | 要求:**
- Must see Goal Tag (ID 20 or 24) to activate | 必须看到目标标签（ID 20 或 24）才能启用
- Chassis is disabled during adaptive shooting | 自适应发射期间底盘被禁用

| Distance | Velocity | 距离 | 转速 |
|----------|----------|------|------|
| ≤24.4" | 700 TPS | ≤62cm | 700 TPS |
| 24.4"~77.4" | 700→950 | 62cm~197cm | 线性插值 |
| 77.4"~128.4" | 950→1420 | 197cm~326cm | 线性插值 |
| ≥128.4" | 1420 TPS | ≥326cm | 1420 TPS |

#### Brake Servo | 刹车舵机

Auto-brake cycle:
1. Release shoot button → engage brake
2. Speed drops below -680 TPS → release brake

自动刹车循环：
1. 松开发射键 → 刹车接合
2. 转速降到 -680 TPS 以下 → 松开刹车

---

### 2.3 Intake | 进球机构

**File | 文件**: `subsystems/intake/Intake.java`

**Purpose | 用途**: Continuous ball collection with variable power levels.

持续收集游戏元素，支持多档功率。

#### Power Levels | 功率档位

| Mode | Power | Condition | 模式 | 功率 | 条件 |
|------|-------|-----------|------|------|------|
| Standard | 0.5 | Default | 标准 | 0.5 | 默认 |
| Full | 0.65 | LT held | 全功率 | 0.65 | 按住LT |
| Fast Intake | 0.7 | Auto mode | 快速进球 | 0.7 | 自动模式 |
| Fast Shooting | 0.8 | At target velocity | 快速发射 | 0.8 | 达到目标转速 |
| Transit | 1.0 | During transit | 传输中 | 1.0 | 传输时 |

**Note**: Intake runs continuously (`isRunning = true` by default).

**注意**：进球机构默认持续运转（`isRunning = true`）。

---

### 2.4 Transit | 传输机构

**File | 文件**: `subsystems/transit/Transit.java`

**Purpose | 用途**: Servo that pushes balls into the flywheel.

将球推入飞轮的舵机。

#### States | 状态

```java
public enum TransitState {
    UP(0.87),    // Extended - pushing ball | 伸出 - 推球
    DOWN(0.67);  // Retracted - loading position | 收回 - 装填位置
}
```

The `TransitCommand` only raises transit when `shooter.isShooterAtSetPoint()` returns true.

`TransitCommand` 只在 `shooter.isShooterAtSetPoint()` 返回 true 时抬起传输机构。

---

### 2.5 Vision | 视觉

**File | 文件**: `subsystems/vision/Vision.java`

**Purpose | 用途**: AprilTag detection using Limelight3A.

使用 Limelight3A 进行 AprilTag 检测。

#### AprilTag IDs | AprilTag 编号

| Tag ID | Location | 位置 |
|--------|----------|------|
| 20 | Blue alliance goal | 蓝方目标 |
| 24 | Red alliance goal | 红方目标 |
| 21, 22, 23 | Obelisks (not used) | 方尖碑（不使用） |

#### Key Methods | 关键方法

| Method | Returns | 返回值 |
|--------|---------|--------|
| `getDetectedTagId()` | Tag ID or -1 | 标签ID或-1 |
| `getTx()` | Horizontal offset (°) | 水平偏移（度） |
| `getTy()` | Vertical offset (°) | 垂直偏移（度） |
| `getRobotPose()` | 3D pose from tag | 从标签获取的3D位姿 |
| `getDistanceToTag()` | Distance (inches) | 距离（英寸） |

---

## 3. Commands | 命令

Commands encapsulate discrete actions. They follow the FTCLib `CommandBase` pattern:

命令封装离散动作。遵循 FTCLib `CommandBase` 模式：

```java
public class ExampleCommand extends CommandBase {
    @Override
    public void initialize() { /* Runs once at start | 开始时运行一次 */ }
    
    @Override
    public void execute() { /* Runs every loop | 每个循环运行 */ }
    
    @Override
    public void end(boolean interrupted) { /* Cleanup | 清理 */ }
    
    @Override
    public boolean isFinished() { /* Return true when done | 完成时返回true */ }
}
```

### TeleOp Commands | 手动命令

| Command | Function | 功能 |
|---------|----------|------|
| `TeleOpDriveCommand` | Field-centric drive + auto-aim | 场地坐标系驾驶 + 自瞄 |
| `TransitCommand` | Fire when shooter ready | 转速就绪时发射 |

### Auto Commands | 自动命令

| Command | Function | 功能 |
|---------|----------|------|
| `AutoDriveCommand` | Follow PathChain with timeout | 跟踪路径链（带超时） |
| `AutoTransitCommand` | Fire with position check | 位置检查后发射 |
| `AutoAlignCommand` | Turn to align with goal tag | 转向对准目标标签 |
| `HoldPositionCommand` | Hold current position | 保持当前位置 |

---

## 4. TeleOp Structure | 手动程序结构

**Entry Point | 入口点**: `opmodes/teleops/TeleOp.java`

```
TeleOp.java
    │
    ├── Robot.java (TeleOp Container - NOT for Auto!)
    │   ├── MecanumDrivePinpoint  ← Only in TeleOp | 仅手动模式
    │   ├── Shooter
    │   ├── Transit
    │   ├── Intake
    │   └── Vision
    │
    ├── DriverControls.bind() (Gamepad bindings | 手柄绑定)
    │
    └── TeleOpDriveCommand (Default drive command | 默认驾驶命令)
```

### Execution Flow | 执行流程

1. `initialize()`: Create Robot, bind controls, register drive command
2. `run()`: CommandScheduler executes all active commands + telemetry update

---

## 5. Autonomous Structure | 自动程序结构

All autonomous programs extend `AutoCommandBase`:

所有自动程序继承 `AutoCommandBase`：

```java
public abstract class AutoCommandBase extends LinearOpMode {
    protected Follower follower;  // Pedro Pathing
    protected Shooter shooter;
    protected Transit transit;
    protected Intake intake;
    protected Vision vision;
    
    public abstract Command runAutoCommand();  // Define your sequence | 定义序列
    public abstract Pose getStartPose();       // Define start position | 定义起始位置
}
```

### 5.1 Far Auto | 远起点自动

**Files | 文件**: `BlueFar.java`, `RedFar.java`, `BlueFarShanghai.java`, etc.

**Strategy | 策略**: Start far from goal, push samples into scoring zone.

从远离目标的位置出发，将样本推入得分区。

#### Path Structure | 路径结构

```
Start → Shoot (preload) → Sample1 → PushZone → Sample2 → Shoot → ... → Park
起点   → 发射（预装）    → 样本1  → 推球区    → 样本2  → 发射  → ... → 停靠
```

#### Typical Far Auto Sequence | 典型远起点自动序列

```java
new SequentialCommandGroup(
    // 1. Initialize intake
    new InstantCommand(() -> intake.startIntake()),
    
    // 2. Path 1: Start → Shoot Pose (with shooter acceleration)
    new InstantCommand(() -> shooter.setShooterState(FAST)),
    new AutoDriveCommand(follower, path1, 3000),  // 3s timeout
    new TransitCommand(transit, shooter).withTimeout(1300),
    
    // 3. Path 2: Shoot → Sample pickup
    new InstantCommand(() -> shooter.setShooterState(STOP)),
    new AutoDriveCommand(follower, path2, 5000),
    
    // 4. Path 3: Push sample to zone
    new AutoDriveCommand(follower, path3, 4000),
    
    // 5. Path 4: Return to shoot
    new InstantCommand(() -> shooter.setShooterState(FAST)),
    new AutoDriveCommand(follower, path4, 4000),
    new TransitCommand(transit, shooter).withTimeout(1300),
    
    // 6. Park
    new AutoDriveCommand(follower, pathPark, 3000)
);
```

### 5.2 Near Auto | 近起点自动

**Files | 文件**: `BlueNear.java`, `RedNear.java`, etc.

**Strategy | 策略**: Start near goal, pick up samples and shoot directly.

从靠近目标的位置出发，拾取样本并直接发射。

#### Path Structure | 路径结构

```
Start → Shoot → Sample1 → Intermediate → Shoot → Sample2 → Shoot → ... → Park
起点   → 发射  → 样本1   → 中间点      → 发射  → 样本2   → 发射  → ... → 停靠
```

#### Key Differences | 关键区别

| Aspect | Far Auto | Near Auto |
|--------|----------|-----------|
| Distance | 128" to goal | 24" to goal |
| Speed | FAST (1420 TPS) | SLOW (700 TPS) |
| Strategy | Push samples | Direct pickup |
| Path type | Mostly straight | Mostly curved |

| 方面 | 远起点 | 近起点 |
|------|--------|--------|
| 距离 | 128" 到目标 | 24" 到目标 |
| 转速 | FAST (1420 TPS) | SLOW (700 TPS) |
| 策略 | 推球 | 直接拾取 |
| 路径类型 | 多为直线 | 多为曲线 |

---

## 6. Key Algorithms | 核心算法

### 6.1 Adaptive Shooting Curve | 自适应发射曲线

Piecewise linear interpolation based on 3 calibrated data points:

基于3个校准数据点的分段线性插值：

```
TPS
1420 ─────────────────────────●
     │                     ╱
 950 │            ●───────╱
     │          ╱
 700 ●─────────╱
     │
     └────────────────────────── Distance (inches)
     0   24.4   77.4   128.4
```

### 6.2 Field-Centric Drive | 场地坐标系驾驶

```java
// Rotate joystick input by robot heading
// 将摇杆输入按机器人朝向旋转
double heading = getHeading();
double rotatedX = x * cos(heading) - y * sin(heading);
double rotatedY = x * sin(heading) + y * cos(heading);
```

### 6.3 Position Hold | 位置保持

When no joystick input:
- Store last position
- Apply P control to return to that position

无摇杆输入时：
- 存储上次位置
- 应用P控制返回该位置

```java
double errorX = targetX - currentX;
double errorY = targetY - currentY;
double errorH = Util.normalizeAngleRadians(targetH - currentH);

moveRobotFieldRelative(
    kP_XY * errorY,
    kP_XY * errorX,
    kP_H * errorH
);
```

---

## 7. Configuration & Tuning | 配置与调参

### Constants Files | 常量文件

| File | Contents | 内容 |
|------|----------|------|
| `DriveConstants.java` | Motor names, PID, deadband | 电机名称、PID、死区 |
| `ShooterConstants.java` | Velocities, servo positions | 转速、舵机位置 |
| `IntakeConstants.java` | Power levels | 功率档位 |
| `TransitConstants.java` | Servo positions | 舵机位置 |
| `Constants.java` | Pedro Pathing config | Pedro Pathing 配置 |

### FTC Dashboard Tuning | FTC Dashboard 调参

All `@Config` annotated classes can be tuned in real-time:
1. Connect to `http://192.168.43.1:8080/dash`
2. Find class under "Configuration"
3. Modify values live

所有带 `@Config` 注解的类可以实时调参：
1. 连接 `http://192.168.43.1:8080/dash`
2. 在 "Configuration" 下找到类
3. 实时修改参数

---

## 8. Control Mapping | 手柄映射

### Driver Controller | 主手柄

| Button | Function | 功能 |
|--------|----------|------|
| **Left Stick** | Strafe | 平移 |
| **Right Stick** | Turn | 转向 |
| **Left Stick Button** | Reset heading | 重置朝向 |
| **LB** | Slow shot (700 TPS) + Auto-aim | 近射 + 自瞄 |
| **RB** | Mid shot (950 TPS) + Auto-aim | 中射 + 自瞄 |
| **RT** | Fast shot (1420 TPS) + Auto-aim | 远射 + 自瞄 |
| **LT** | Full power intake + Transit fire | 全功率进球 + 发射 |
| **A** | Auto-aim only (no shoot) | 仅自瞄（不发射） |
| **X** | Adaptive fire (Blue goal) - requires Goal Tag visible, disables chassis | 自适应发射（蓝方）- 需看到目标标签，禁用底盘 |
| **B** | Adaptive fire (Red goal) - requires Goal Tag visible, disables chassis | 自适应发射（红方）- 需看到目标标签，禁用底盘 |
| **D-Pad Up** | Reverse intake | 反转进球 |
| **D-Pad Down** | Manual brake | 手动刹车 |
| **D-Pad L/R** | Fine rotation | 微调转向 |

---

## Quick Reference Card | 快速参考卡

### Shooter Velocities | 发射器转速

| State | TPS | Distance | 状态 | 转速 | 距离 |
|-------|-----|----------|------|------|------|
| STOP | -600 | - | 停止 | -600 | - |
| SLOW | -700 | ≤24" | 慢速 | -700 | ≤62cm |
| MID | -950 | ~77" | 中速 | -950 | ~196cm |
| FAST | -1420 | ≥128" | 快速 | -1420 | ≥325cm |

### Servo Positions | 舵机位置

| Servo | Low | High | 舵机 | 低位 | 高位 |
|-------|-----|------|------|------|------|
| Shooter Angle | 0.85 (near) | 0.29 (far) | 发射角度 | 0.85 | 0.29 |
| Transit | 0.67 (down) | 0.87 (up) | 传输 | 0.67 | 0.87 |
| Brake | 0.81 (released) | 0.85 (engaged) | 刹车 | 0.81 | 0.85 |

---

## 9. Complete Method Reference | 完整方法参考

### 9.1 MecanumDrivePinpoint Methods | 底盘方法

**File | 文件**: `subsystems/drive/MecanumDrivePinpoint.java`

| Method | Description | 描述 |
|--------|-------------|------|
| `moveRobot(forward, strafe, turn)` | Robot-centric movement | 机器人坐标系移动 |
| `moveRobotFieldRelative(forward, strafe, turn)` | Field-centric movement | 场地坐标系移动 |
| `stop()` | Stop all motors | 停止所有电机 |
| `reset(heading)` | Reset heading offset | 重置朝向偏移 |
| `resetHeading()` | Reset heading to 0 | 重置朝向为0 |
| `getPose()` | Get current Pose2D from Pinpoint | 获取当前位姿 |
| `getYawOffset()` | Get current yaw offset | 获取偏航偏移 |
| `setGamepad(on)` | Set gamepad active flag | 设置手柄活动标志 |
| `isHeadingAtSetPoint(heading)` | Check if at target heading | 检查是否到达目标朝向 |
| `getAngleToTarget(x, y)` | Calculate angle to point | 计算到目标点的角度 |
| **Auto-Aim Methods | 自瞄方法** | |
| `getAlignTurnPower(vision)` | Get turn power for auto-aim | 获取自瞄转向功率 |
| `resetAutoAimOffset()` | Reset auto-aim state | 重置自瞄状态 |
| `getSearchTurnPower(vision, speed)` | Get turn power for tag search | 获取搜索标签的转向功率 |
| `foundLastAlignedTag(vision)` | Check if found last aligned tag | 检查是否找到上次对准的标签 |
| `getLastAlignedTagId()` | Get last aligned tag ID | 获取上次对准的标签ID |
| `hasLastAlignedTag()` | Check if has aligned history | 检查是否有对准历史 |
| `clearLastAlignedTag()` | Clear aligned tag history | 清除对准历史 |
| **Vision Calibration | 视觉校准** | |
| `visionCalibrate(vision, alliance)` | Calibrate odometry from vision | 从视觉校准里程计 |
| `hasVisionCalibrated()` | Check if calibrated | 检查是否已校准 |
| `getAbsolutePose()` | Get absolute pose | 获取绝对位姿 |
| **Absolute Position | 绝对位置** | |
| `updateAbsolutePositionFromVision(vision)` | Update from vision | 从视觉更新位置 |
| `updateAbsolutePositionFromOdometry()` | Update from odometry | 从里程计更新位置 |
| `getAbsoluteX()` | Get absolute X (inches) | 获取绝对X坐标 |
| `getAbsoluteY()` | Get absolute Y (inches) | 获取绝对Y坐标 |
| `getAbsoluteHeading()` | Get absolute heading (radians) | 获取绝对朝向 |
| `hasAbsolutePosition()` | Check if has valid position | 检查是否有有效位置 |
| `resetAbsolutePosition()` | Reset to (0,0,0) | 重置为(0,0,0) |
| `distanceToPoint(x, y)` | Distance to point (inches) | 到点的距离 |
| `distanceToGoal(tagId)` | Distance to goal (inches) | 到目标的距离 |
| **Adaptive Shooting | 自适应发射** | |
| `calculateAdaptiveVelocity(tagId)` | Calculate velocity for distance | 根据距离计算转速 |
| `calculateAdaptiveServoPosition(tagId)` | Calculate servo for distance | 根据距离计算舵机位置 |
| `getAdaptiveSegment(tagId)` | Get segment name | 获取区间名称 |
| `isAutoFireAllowed(tx)` | Check if aligned enough to fire | 检查是否对准可发射 |

### 9.2 Shooter Methods | 发射器方法

**File | 文件**: `subsystems/shooter/Shooter.java`

| Method | Description | 描述 |
|--------|-------------|------|
| `setShooterState(state)` | Set STOP/SLOW/MID/FAST state | 设置状态 |
| `getVelocity()` | Get current velocity (TPS) | 获取当前转速 |
| `getTargetVelocity()` | Get target velocity | 获取目标转速 |
| `isShooterAtSetPoint()` | Check if at target velocity | 检查是否达到目标转速 |
| **Adaptive Velocity | 自适应转速** | |
| `setAdaptiveVelocity(velocity)` | Set adaptive velocity | 设置自适应转速 |
| `getAdaptiveVelocity()` | Get adaptive velocity | 获取自适应转速 |
| `setAdaptiveServoPosition(pos)` | Set adaptive servo position | 设置自适应舵机位置 |
| `getAdaptiveServoPosition()` | Get adaptive servo position | 获取自适应舵机位置 |
| **Brake Control | 刹车控制** | |
| `engageBrake()` | Engage brake servo | 接合刹车 |
| `releaseBrake()` | Release brake servo | 松开刹车 |
| `toggleBrake()` | Toggle brake state | 切换刹车状态 |
| `isBrakeEngaged()` | Check brake state | 检查刹车状态 |
| `manualEngageBrake()` | Manual brake override | 手动刹车覆盖 |
| `manualReleaseBrake()` | Release manual override | 释放手动覆盖 |
| `startAutoBrakeCycle()` | Start auto-brake on release | 开始自动刹车循环 |
| `cancelAutoBrakeCycle()` | Cancel auto-brake | 取消自动刹车循环 |
| `isAutoBrakeCycleActive()` | Check auto-brake status | 检查自动刹车状态 |

### 9.3 Intake Methods | 进球方法

**File | 文件**: `subsystems/intake/Intake.java`

| Method | Description | 描述 |
|--------|-------------|------|
| `startIntake()` | Start intake motor | 启动进球电机 |
| `stopIntake()` | Stop intake motor | 停止进球电机 |
| `toggle()` | Toggle running state | 切换运转状态 |
| `isRunning()` | Check if running | 检查是否运转中 |
| `setShooting(bool)` | Set shooting mode | 设置发射模式 |
| `isShooting()` | Check shooting mode | 检查发射模式 |
| `setFullPower(bool)` | Set full power mode | 设置全功率模式 |
| `setFastIntaking(bool)` | Set fast intake mode | 设置快速进球模式 |
| `setReversed(bool)` | Set reverse mode | 设置反转模式 |
| `setFastShooting(bool)` | Set fast shooting mode | 设置快速发射模式 |
| `getVelocity()` | Get motor velocity | 获取电机转速 |

### 9.4 Transit Methods | 传输方法

**File | 文件**: `subsystems/transit/Transit.java`

| Method | Description | 描述 |
|--------|-------------|------|
| `setTransitState(state)` | Set UP/DOWN state | 设置状态 |

### 9.5 Vision Methods | 视觉方法

**File | 文件**: `subsystems/vision/Vision.java`

| Method | Description | 描述 |
|--------|-------------|------|
| **Core Methods | 核心方法** | |
| `getDetectedTagId()` | Get detected tag ID (-1 if none) | 获取检测到的标签ID |
| `getDetectedAlliance()` | Get alliance from tag | 从标签获取联盟颜色 |
| `hasTarget()` | Check if any tag visible | 检查是否有可见标签 |
| `isAllianceTag(alliance)` | Check if tag matches alliance | 检查标签是否匹配联盟 |
| `getRobotPose()` | Get robot Pose3D from tag | 从标签获取机器人位姿 |
| `stop()` | Stop Limelight polling | 停止Limelight轮询 |
| **Alignment Data | 对准数据** | |
| `getTx()` | Horizontal offset (degrees) | 水平偏移（度） |
| `getTy()` | Vertical offset (degrees) | 垂直偏移（度） |
| `getDistanceToTag()` | Distance to tag (inches) | 到标签距离（英寸） |
| **Debug Methods | 调试方法** | |
| `getStatus()` | Get LLStatus object | 获取状态对象 |
| `isConnected()` | Check Limelight connection | 检查连接状态 |
| `getPipelineIndex()` | Get current pipeline | 获取当前管线 |
| `getFps()` | Get current FPS | 获取当前帧率 |
| `isResultValid()` | Check result validity | 检查结果有效性 |
| `getNumTagsDetected()` | Count detected tags | 检测到的标签数量 |
| `setPipeline(index)` | Switch pipeline | 切换管线 |
| `getRawTagId()` | Get raw tag (no filtering) | 获取原始标签ID |
| `getTagArea()` | Get tag area | 获取标签面积 |
| `getRawRobotPose()` | Get raw pose (no filtering) | 获取原始位姿 |
| `getDebugState()` | Get debug string | 获取调试字符串 |

### 9.6 Utility Methods | 工具方法

**File | 文件**: `utils/Util.java`

| Method | Description | 描述 |
|--------|-------------|------|
| `Pose2DToPose(pose2d)` | Convert Pose2D to Pose | 转换Pose2D到Pose |
| `epsilonEqual(a, b, epsilon)` | Compare with tolerance | 带容差比较 |
| `normalizeAngleDegrees(angle)` | Normalize angle to [-180, 180] | 归一化角度到[-180, 180] |
| `normalizeAngleRadians(angle)` | Normalize angle to [-π, π] | 归一化角度到[-π, π] |
| `visionPoseToPinpointPose(pose3d)` | Convert Limelight pose | 转换Limelight位姿 |
| `debugVisionConversion(pose3d)` | Debug conversion steps | 调试转换步骤 |

---

## 10. Troubleshooting Guide | 故障排查指南

### 10.1 Driving Issues | 驾驶问题

| Problem | Possible Cause | Solution | File & Method |
|---------|----------------|----------|---------------|
| **Robot drifts when stopped | 机器人停止时漂移** | Deadband too low | Increase `deadband` | `DriveConstants.java` → `deadband` |
| **Field-centric is reversed | 场地坐标系反向** | Wrong heading | Press Left Stick to reset | `MecanumDrivePinpoint.java` → `reset()` |
| **Strafing is uneven | 平移不均匀** | Strafing balance wrong | Adjust `strafingBalance` | `DriveConstants.java` → `strafingBalance` |
| **Motors don't move | 电机不动** | Wrong motor names | Check hardware map names | `DriveConstants.java` → motor names |
| **Rotation is reversed | 转向反向** | Motor direction | Check `setDirection()` | `MecanumDrivePinpoint.java` constructor |

| 问题 | 可能原因 | 解决方案 | 文件 & 方法 |
|------|----------|----------|-------------|
| **机器人停止时漂移** | 死区太低 | 增加 `deadband` | `DriveConstants.java` → `deadband` |
| **场地坐标系反向** | 朝向错误 | 按左摇杆重置 | `MecanumDrivePinpoint.java` → `reset()` |
| **平移不均匀** | 平移平衡错误 | 调整 `strafingBalance` | `DriveConstants.java` → `strafingBalance` |
| **电机不动** | 电机名称错误 | 检查硬件映射名称 | `DriveConstants.java` → 电机名称 |
| **转向反向** | 电机方向 | 检查 `setDirection()` | `MecanumDrivePinpoint.java` 构造函数 |

### 10.2 Shooter Issues | 发射器问题

| Problem | Possible Cause | Solution | File & Method |
|---------|----------------|----------|---------------|
| **Shooter won't reach velocity | 发射器达不到转速** | Power too low / Motors weak | Check motor / Increase feedforward | `Shooter.java` → `periodic()` |
| **Velocity oscillates | 转速振荡** | PID instead of Bang-Bang | Use Bang-Bang control | `Shooter.java` → `periodic()` |
| **Ball doesn't fire | 球不发射** | Velocity not reached | Check `isShooterAtSetPoint()` | `Shooter.java` → `isShooterAtSetPoint()` |
| **Wrong velocity for distance | 距离对应转速错误** | Calibration data | Update distance/velocity constants | `ShooterConstants.java` → `nearDistance`, `midDistance`, `farDistance` |
| **Brake doesn't engage | 刹车不接合** | Servo position wrong | Adjust `brakeServoEngagedPos` | `ShooterConstants.java` → brake positions |
| **Servo angle wrong | 舵机角度错误** | Servo position values | Adjust `shooterServoDownPos/MidPos/UpPos` | `ShooterConstants.java` → servo positions |
| **Adaptive X/B not working | X/B自适应不工作** | No Goal Tag visible (ID 20/24) | Aim at goal tag first | `DriverControls.java` → X/B button logic |

| 问题 | 可能原因 | 解决方案 | 文件 & 方法 |
|------|----------|----------|-------------|
| **发射器达不到转速** | 功率太低/电机弱 | 检查电机/增加前馈 | `Shooter.java` → `periodic()` |
| **转速振荡** | 使用PID而非Bang-Bang | 使用Bang-Bang控制 | `Shooter.java` → `periodic()` |
| **球不发射** | 转速未达到 | 检查 `isShooterAtSetPoint()` | `Shooter.java` → `isShooterAtSetPoint()` |
| **距离对应转速错误** | 校准数据 | 更新距离/转速常量 | `ShooterConstants.java` → 距离常量 |
| **刹车不接合** | 舵机位置错误 | 调整 `brakeServoEngagedPos` | `ShooterConstants.java` → 刹车位置 |
| **舵机角度错误** | 舵机位置值 | 调整舵机位置常量 | `ShooterConstants.java` → 舵机位置 |
| **X/B自适应不工作** | 看不到目标标签(ID 20/24) | 先瞄准目标标签 | `DriverControls.java` → X/B 按键逻辑 |

### 10.3 Vision Issues | 视觉问题

| Problem | Possible Cause | Solution | File & Method |
|---------|----------------|----------|---------------|
| **No tag detected | 检测不到标签** | Limelight not started | Check `limelight.start()` | `Vision.java` → constructor |
| **FPS is 0 | 帧率为0** | Limelight disconnected | Check USB/network | `Vision.java` → `getFps()` |
| **Wrong tag ID | 标签ID错误** | Wrong pipeline | Switch to pipeline 0 | `Vision.java` → `setPipeline(0)` |
| **Pose is null | 位姿为null** | No tag in view | Aim at goal tag | `Vision.java` → `getRobotPose()` |
| **Distance wrong | 距离错误** | Coordinate conversion | Check `getDistanceToTag()` | `Vision.java` → `getDistanceToTag()` |
| **Pose offset wrong | 位姿偏移错误** | Vision conversion params | Adjust `visionXOffset/YOffset` | `Util.java` → vision offset constants |

| 问题 | 可能原因 | 解决方案 | 文件 & 方法 |
|------|----------|----------|-------------|
| **检测不到标签** | Limelight未启动 | 检查 `limelight.start()` | `Vision.java` → 构造函数 |
| **帧率为0** | Limelight断开 | 检查USB/网络 | `Vision.java` → `getFps()` |
| **标签ID错误** | 管线错误 | 切换到管线0 | `Vision.java` → `setPipeline(0)` |
| **位姿为null** | 看不到标签 | 瞄准目标标签 | `Vision.java` → `getRobotPose()` |
| **距离错误** | 坐标转换 | 检查 `getDistanceToTag()` | `Vision.java` → `getDistanceToTag()` |
| **位姿偏移错误** | 视觉转换参数 | 调整偏移常量 | `Util.java` → 视觉偏移常量 |

### 10.4 Auto-Aim Issues | 自瞄问题

| Problem | Possible Cause | Solution | File & Method |
|---------|----------------|----------|---------------|
| **Jitters at close range | 近距离抖动** | kP too high for close | Reduce `kP_near` | `MecanumDrivePinpoint.java` → `kP_near` |
| **Jitters at all distances | 各距离都抖** | tx noise | Reduce `txFilterAlpha` (0.2~0.15) | `MecanumDrivePinpoint.java` → `txFilterAlpha` |
| **Oscillates at deadband | 死区边界振荡** | Hysteresis too small | Increase `alignHysteresis` | `MecanumDrivePinpoint.java` → `alignHysteresis` |
| **Auto-aim too slow (near) | 近距离太慢** | kP too low | Increase `kP_near` | `MecanumDrivePinpoint.java` → `kP_near` |
| **Auto-aim too slow (far) | 远距离太慢** | kP too low | Increase `kP_far` | `MecanumDrivePinpoint.java` → `kP_far` |
| **Response too sluggish | 响应迟钝** | Filter too strong | Increase `txFilterAlpha` (0.4~0.5) | `MecanumDrivePinpoint.java` → `txFilterAlpha` |
| **Far shot offset wrong | 远射偏移错误** | Offset calibration | Adjust `farOffsetDegrees` | `MecanumDrivePinpoint.java` → `farOffsetDegrees` |
| **Offset triggers too early | 偏移触发太早** | Distance threshold | Adjust `farDistanceThreshold` | `MecanumDrivePinpoint.java` → `farDistanceThreshold` |

| 问题 | 可能原因 | 解决方案 | 文件 & 方法 |
|------|----------|----------|-------------|
| **近距离抖动** | 近距离kP太高 | 减少 `kP_near` | `MecanumDrivePinpoint.java` → `kP_near` |
| **各距离都抖** | tx噪声 | 减少 `txFilterAlpha` (0.2~0.15) | `MecanumDrivePinpoint.java` → `txFilterAlpha` |
| **死区边界振荡** | 滞后太小 | 增加 `alignHysteresis` | `MecanumDrivePinpoint.java` → `alignHysteresis` |
| **近距离太慢** | kP太低 | 增加 `kP_near` | `MecanumDrivePinpoint.java` → `kP_near` |
| **远距离太慢** | kP太低 | 增加 `kP_far` | `MecanumDrivePinpoint.java` → `kP_far` |
| **响应迟钝** | 滤波太强 | 增加 `txFilterAlpha` (0.4~0.5) | `MecanumDrivePinpoint.java` → `txFilterAlpha` |
| **远射偏移错误** | 偏移校准 | 调整 `farOffsetDegrees` | `MecanumDrivePinpoint.java` → `farOffsetDegrees` |
| **偏移触发太早** | 距离阈值 | 调整 `farDistanceThreshold` | `MecanumDrivePinpoint.java` → `farDistanceThreshold` |

### 10.5 Intake Issues | 进球问题

| Problem | Possible Cause | Solution | File & Method |
|---------|----------------|----------|---------------|
| **Intake doesn't run | 进球机构不转** | `isRunning` is false | Call `startIntake()` | `Intake.java` → `startIntake()` |
| **Power too low | 功率太低** | Wrong power level | Adjust power constants | `IntakeConstants.java` → power values |
| **Intake reversed | 进球方向反了** | Motor direction | Check `setDirection()` | `Intake.java` → constructor |

| 问题 | 可能原因 | 解决方案 | 文件 & 方法 |
|------|----------|----------|-------------|
| **进球机构不转** | `isRunning` 为 false | 调用 `startIntake()` | `Intake.java` → `startIntake()` |
| **功率太低** | 功率档位错误 | 调整功率常量 | `IntakeConstants.java` → 功率值 |
| **进球方向反了** | 电机方向 | 检查 `setDirection()` | `Intake.java` → 构造函数 |

### 10.6 Transit Issues | 传输问题

| Problem | Possible Cause | Solution | File & Method |
|---------|----------------|----------|---------------|
| **Transit doesn't move | 传输不动** | Servo position wrong | Adjust `transitUpPos/DownPos` | `TransitConstants.java` → servo positions |
| **Ball not pushed | 球不推出** | Shooter not at speed | Check `isShooterAtSetPoint()` | `TransitCommand.java` → `execute()` |
| **Transit fires too early | 传输触发太早** | Velocity tolerance | Adjust `shooterEpsilon` | `ShooterConstants.java` → `shooterEpsilon` |

| 问题 | 可能原因 | 解决方案 | 文件 & 方法 |
|------|----------|----------|-------------|
| **传输不动** | 舵机位置错误 | 调整舵机位置常量 | `TransitConstants.java` → 舵机位置 |
| **球不推出** | 发射器未达速 | 检查 `isShooterAtSetPoint()` | `TransitCommand.java` → `execute()` |
| **传输触发太早** | 转速容差 | 调整 `shooterEpsilon` | `ShooterConstants.java` → `shooterEpsilon` |

### 10.7 Autonomous Issues | 自动问题

| Problem | Possible Cause | Solution | File & Method |
|---------|----------------|----------|---------------|
| **Robot doesn't move | 机器人不动** | Path not built | Check `buildPaths()` | Auto file → `buildPaths()` |
| **Wrong starting position | 起始位置错误** | Start pose | Check `getStartPose()` | Auto file → `getStartPose()` |
| **Path goes wrong way | 路径走错方向** | Pose coordinates | Check X/Y/heading values | Auto file → pose definitions |
| **Timeout before finish | 超时未完成** | Timeout too short | Increase `withTimeout()` value | Auto file → command timeouts |
| **Follower oscillates | 跟踪器振荡** | PID tuning | Adjust Pedro Pathing constants | `Constants.java` → PID values |

| 问题 | 可能原因 | 解决方案 | 文件 & 方法 |
|------|----------|----------|-------------|
| **机器人不动** | 路径未构建 | 检查 `buildPaths()` | 自动文件 → `buildPaths()` |
| **起始位置错误** | 起始位姿 | 检查 `getStartPose()` | 自动文件 → `getStartPose()` |
| **路径走错方向** | 位姿坐标 | 检查X/Y/朝向值 | 自动文件 → 位姿定义 |
| **超时未完成** | 超时太短 | 增加 `withTimeout()` 值 | 自动文件 → 命令超时 |
| **跟踪器振荡** | PID调参 | 调整 Pedro Pathing 常量 | `Constants.java` → PID值 |

---

## 11. Debug Telemetry Reference | 调试遥测参考

### TeleOp Telemetry | 手动遥测数据

Available in `TeleOp.java`:

| Data | Meaning | 含义 |
|------|---------|------|
| `X`, `Y`, `Heading` | Robot odometry position | 机器人里程计位置 |
| `Absolute X/Y` | Vision-fused position | 视觉融合位置 |
| `Tag ID` | Detected AprilTag | 检测到的标签 |
| `tx`, `ty` | Vision offset (degrees) | 视觉偏移（度） |
| `Distance to Goal` | Calculated distance | 计算距离 |
| `Segment` | Adaptive velocity segment | 自适应速度区间 |
| `Shooter TPS` | Current shooter velocity | 当前发射器转速 |
| `Target TPS` | Target shooter velocity | 目标发射器转速 |
| `CAN FIRE` | Aligned enough to fire | 是否对准可发射 |

### Vision Debug | 视觉调试

Use `vision.getDebugState()` to diagnose:

| Output | Meaning | 含义 |
|--------|---------|------|
| `result=NULL` | No Limelight data | 无Limelight数据 |
| `result.isValid=FALSE` | Invalid result | 结果无效 |
| `fiducialResults=EMPTY` | No tags detected | 未检测到标签 |
| `area=X < 0.001` | Tag too small (filtered) | 标签太小（被过滤） |
| `getRobotPoseFieldSpace=NULL` | Pose calculation failed | 位姿计算失败 |
| `OK: pose available` | Everything working | 一切正常 |

---

## 12. Quick Debug Checklist | 快速调试清单

### Before Match | 比赛前

- [ ] Limelight connected? `vision.isConnected()` = true
- [ ] Correct pipeline? `vision.getPipelineIndex()` = 0
- [ ] See goal tag? `vision.getDetectedTagId()` = 20 or 24
- [ ] Shooter reaches velocity? `shooter.isShooterAtSetPoint()` = true
- [ ] Intake running? `intake.isRunning()` = true
- [ ] Field-centric correct? Reset heading before start

### During Match Issues | 比赛中问题

| Symptom | Quick Check | 症状 | 快速检查 |
|---------|-------------|------|----------|
| Can't shoot | Check shooter TPS in telemetry | 不能发射 | 检查遥测中的发射器转速 |
| Auto-aim not working | Check if tag is visible | 自瞄不工作 | 检查是否能看到标签 |
| Robot drifts | Reset heading (Left Stick) | 机器人漂移 | 重置朝向（左摇杆） |
| Balls not feeding | Check transit servo | 球不传输 | 检查传输舵机 |

---

*End of Guide | 指南结束*


