# Prototype2026-Public-2 Operation Guide | 操作指南

> **Bilingual Technical Documentation | 中英双语技术文档**  
> FTC Team 12527 | Last Updated: 2026-02-04 (Shooter Optimization & Auto-Aim Fixes)

---

## Table of Contents | 目录

1. [Architecture Overview | 架构概览](#1-architecture-overview--架构概览)
   - [Robot Container | 机器人容器](#robot-container--机器人容器)
2. [Subsystems | 子系统](#2-subsystems--子系统)
   - [MecanumDrivePinpoint | 底盘驱动](#21-mecanumdrivepinpoint--底盘驱动)
   - [Shooter | 发射器](#22-shooter--发射器)
   - [Intake | 进球机构](#23-intake--进球机构)
   - [Transit | 传输机构](#24-transit--传输机构)
   - [Vision | 视觉](#25-vision--视觉)
   - [Turret | 云台](#26-turret--云台)
3. [Commands | 命令](#3-commands--命令)
   - [WaitForTurretCommand](#waitforturretcommand)
4. [TeleOp Structure | 手动程序结构](#4-teleop-structure--手动程序结构)
   - [SoloEmergency | 紧急备用](#soloemergency---backup-opmode--紧急备用程序)
5. [Autonomous Structure | 自动程序结构](#5-autonomous-structure--自动程序结构)
   - [Far Auto | 远起点自动](#51-far-auto--远起点自动)
   - [Near Auto | 近起点自动](#52-near-auto--近起点自动)
6. [Key Algorithms | 核心算法](#6-key-algorithms--核心算法)
7. [Configuration & Tuning | 配置与调参](#7-configuration--tuning--配置与调参)
   - [AutoConstants](#autoconstants)
   - [TeleOpConstants](#teleopconstants)
8. [Control Mapping | 手柄映射](#8-control-mapping--手柄映射)
9. [Complete Method Reference | 完整方法参考](#9-complete-method-reference--完整方法参考)
   - [DashboardUtil](#97-dashboardutil--dashboard-工具)
   - [FunctionalButton](#98-functionalbutton--函数式按钮)
   - [Units](#99-units--单位转换)
   - [DriverControls](#910-drivercontrols--驾驶员控制绑定)
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
│   ├── vision/           # Limelight vision | 视觉
│   └── turret/           # Turret/Gimbal | 云台
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

### Robot Container | 机器人容器

**File | 文件**: `subsystems/Robot.java`

**Purpose | 用途**: Central container for all TeleOp subsystems. NOT for Autonomous!

为手动程序初始化并持有所有子系统引用。不用于自动程序！

```java
public class Robot {
    public final MecanumDrivePinpoint drive;  // Only in TeleOp
    public final Shooter shooter;
    public final Transit transit;
    public final Intake intake;
    public final Vision vision;   // Can be disabled (ENABLE_VISION)
    public final Turret turret;   // Can be disabled (ENABLE_TURRET)
}
```

| Flag | Default | Description |
|------|---------|-------------|
| `ENABLE_VISION` | `false` | Enable Limelight vision |
| `ENABLE_TURRET` | `true` | Enable turret motor |

> **Important | 重要**: For Autonomous, use `AutoCommandBase` which uses `Follower` instead of `MecanumDrivePinpoint`.
> 自动程序使用 `AutoCommandBase`，它用 `Follower` 而不是 `MecanumDrivePinpoint`。

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
| `calculateAdaptiveVelocity(tagId)` | Distance-based velocity | 根据距离计算转速 |
| `calculateAdaptiveServoPosition(tagId)` | Distance-based angle | 根据距离计算角度 |

#### Absolute Position Fusion | 绝对位置融合

The system fuses Vision and Odometry with **turret angle compensation**:

系统融合视觉和里程计，**带云台角度补偿**：

**When Limelight sees goal tag (20 or 24) | 当 Limelight 看到目标标签时:**
1. Get Limelight's field position (from vision) | 获取 Limelight 在场地中的位置
2. Calculate Limelight offset from chassis center (based on turret angle) | 根据云台角度计算 Limelight 相对底盘中心的偏移
3. Chassis position = Limelight position - offset | 底盘位置 = Limelight 位置 - 偏移

**Turret Geometry | 云台几何:**
- Turret center: 47mm behind chassis center (~1.85") | 云台中心在底盘中心后方 47mm
- Limelight: 140.86mm from turret center (~5.55") | Limelight 距离云台中心 140.86mm

**When no tag visible | 看不到标签时:**
- Dead-reckoning using odometry delta | 使用里程计增量进行航位推算

---

### 2.2 Shooter | 发射器

**File | 文件**: `subsystems/shooter/Shooter.java`

**Purpose | 用途**: Controls dual flywheel motors and angle adjustment servo.

控制双飞轮电机和角度调节舵机。

#### State Machine | 状态机

```java
public enum ShooterState {
    STOP(-600,  0.5),    // Idle speed, mid angle | 待机转速，中位角度
    SLOW(-700,  0.04),   // Near shot, low angle | 近射，低角度
    MID(-950,   0.5),    // Mid shot, mid angle | 中射，中位角度
    FAST(-1420, 1.0);    // Far shot, high angle | 远射，高角度
}
```

#### Control Algorithm | 控制算法

**Pseudo Closed-loop with Feedforward + Motor Braking + Firing Boost** (not PID):

**伪闭环前馈控制 + 电机刹车 + 发射增益**（不是PID）：

```java
double deadband = 15000;  // Stability zone
double feedforward = (targetVel / maxVelocityTPS) * 1.3;  // Corrected

if (error > 50000) {
    power = 1.0;  // Far from target: full power
} else if (error > deadband) {
    power = isMidMode ? 0.7 : 0.85;  // Approach power
} else if (error < -overspeedThreshold) {
    power = isMidMode ? -0.3 : -0.5;  // Motor brake
} else {
    power = feedforward;  // Maintain
}

// Firing boost: +8% power 0.5s after transit opens
if (boostActive) {
    power = lockedPower + 0.08;  // Bypass pseudo closed-loop
}
```

| State | Condition | Power |
|-------|-----------|-------|
| Far from target | `error > 50000` | 1.0 (full) |
| Approaching | `error > 15000` | 0.85 (SLOW/FAST) / 0.70 (MID) |
| Overspeed | `error < -30000` | -0.5 (SLOW/FAST) / -0.3 (MID) |
| Near target | otherwise | feedforward (×1.3 corrected) |
| **Firing boost** | 0.5s after fire | locked + 8% |

> **Updated 2026-02-04**: Added deadband, feedforward correction, mode-specific params, firing boost

Why not PID? Flywheel momentum makes PID oscillate. Pseudo Closed-loop converges faster.

为什么不用PID？飞轮惯性会使PID振荡。伪闭环收敛更快。

#### Adaptive Shooting | 自适应发射 (⚠️ DISABLED | 已禁用)

> **Note**: Adaptive shooting is currently commented out. See `DriverControls.java`.
> **注意**: 自适应发射功能当前已注释禁用。

Automatically calculates velocity based on distance to goal.

根据到目标的距离自动计算转速。

**Requirements | 要求:**
- Must see Goal Tag (ID 20 or 24) to activate | 必须看到目标标签（ID 20 或 24）才能启用

| Distance | Velocity | 距离 | 转速 |
|----------|----------|------|------|
| ≤24.4" | 700 TPS | ≤62cm | 700 TPS |
| 24.4"~77.4" | 700→950 | 62cm~197cm | 线性插值 |
| 77.4"~128.4" | 950→1420 | 197cm~326cm | 线性插值 |
| ≥128.4" | 1420 TPS | ≥326cm | 1420 TPS |

---

### 2.3 Intake | 进球机构

**File | 文件**: `subsystems/intake/Intake.java`

**Purpose | 用途**: Continuous ball collection with variable power levels.

持续收集游戏元素，支持多档功率。

#### Power Levels | 功率档位

| Mode | Power | Condition | 模式 | 功率 | 条件 |
|------|-------|-----------|------|------|------|
| Standard | 0.5 | Default | 标准 | 0.5 | 默认 |
| Full (LT) | **1.0** | LT held | 全功率 | **1.0** | 按住LT |
| Fast Intake | 0.7 | Auto mode | 快速进球 | 0.7 | 自动模式 |
| Fast Shooting | 0.8 | At target velocity | 快速发射 | 0.8 | 达到目标转速 |
| Transit | 1.0 | During transit | 传输中 | 1.0 | 传输时 |

**Note**: Intake runs continuously (`isRunning = true` by default).

**注意**：进球机构默认持续运转（`isRunning = true`）。

---

### 2.4 Transit | 传输机构

**File | 文件**: `subsystems/transit/Transit.java`

**Purpose | 用途**: Servo that pushes balls into the flywheel, with a limit servo that auto-follows.

将球推入飞轮的舵机，附带自动跟随的限位舵机。

#### States | 状态

```java
public enum TransitState {
    UP(0.36),    // Extended - pushing ball | 伸出 - 推球
    DOWN(0.62);  // Retracted - loading position | 收回 - 装填位置
}
```

#### Limit Servo | 限位舵机

The `limitServo` automatically follows the transit state:
- **Transit UP** → Limit servo **OPEN** (0.6)
- **Transit DOWN** → Limit servo **CLOSED** (0.3)

限位舵机自动跟随传输状态：
- **传输抬起** → 限位舵机**打开** (0.6)
- **传输放下** → 限位舵机**关闭** (0.3)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `limitServoName` | `"limitServo"` | Hardware name |
| `limitOpenPos` | 0.6 | Open position (when transit UP) |
| `limitClosedPos` | 0.3 | Closed position (when transit DOWN) |

The `TransitCommand` only raises transit when `shooter.isShooterAtSetPoint()` returns true.

`TransitCommand` 只在 `shooter.isShooterAtSetPoint()` 返回 true 时抬起传输机构。

---

### 2.5 Vision | 视觉 (⚠️ DISABLED | 已禁用)

**File | 文件**: `subsystems/vision/Vision.java`

> **Note**: Vision subsystem is currently disabled. `ENABLE_VISION = false` in `Robot.java`.
> **注意**: 视觉子系统当前已禁用。

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

### 2.6 Turret | 云台 (⚠️ DISABLED | 已禁用)

**File | 文件**: `subsystems/turret/Turret.java`

> **Note**: Turret subsystem is currently disabled. `ENABLE_TURRET = false` in `Robot.java`.
> **注意**: 云台子系统当前已禁用。

**Purpose | 用途**: Controls a single motor for turret/gimbal rotation with position PID and goal tracking.

控制单个电机用于云台旋转，支持位置PID和目标跟踪。

#### Lock Modes | 锁定模式

```java
public enum LockMode {
    MANUAL,     // No lock, manual power control | 手动控制
    SOFT_LOCK,  // Hold at fixed angle (default: 0°) | 固定角度
    HARD_LOCK   // Track goal position | 跟踪目标
}
```

**SOFT_LOCK (软锁定)**:
- Turret holds at 0° (forward) using position PID
- Good for stable shooting position
- 云台保持在0°（正前方），使用位置PID

**HARD_LOCK (硬锁定)**:
- Turret auto-calculates angle to aim at goal
- Two tracking modes based on what tag is visible:

| Condition | Mode | Description |
|-----------|------|-------------|
| See OUR tag (e.g., Blue sees ID 20) | **TX Tracking** | Use tx offset for precise aiming |
| See OTHER tag (e.g., Blue sees ID 24) | **Inertial** | Calculate angle from position |
| No tag visible | **Inertial** | Calculate angle from position |
| Target > 100° (unreachable) | **Unwind** | Return to 0°, cannot be interrupted |

**Alliance-specific aiming | 联盟特定瞄准:**
- SoloBlue: `targetTagId = 20`, always aims at blue basket (4, 140)
- SoloRed: `targetTagId = 24`, always aims at red basket (140, 140)

**Unwind Protection | 回正保护:**
- When target angle > 100°, turret returns to 0° (forward)
- This prevents wire tangling (no slip ring)
- Unwind has HIGHEST priority - cannot be interrupted by any control
- 当目标角度超过100°时，云台返回0°以防止线缆缠绕

#### Key Methods | 关键方法

| Method | Description | 描述 |
|--------|-------------|------|
| **Basic Control | 基础控制** | |
| `setPower(power)` | Set motor power (-1 to 1) ⚠️ Blocked during unwind | 设置电机功率 |
| `stop()` | Stop the motor ⚠️ Blocked during unwind | 停止电机 |
| `rotateLeft(speed)` | Rotate left | 向左旋转 |
| `rotateRight(speed)` | Rotate right | 向右旋转 |
| **Encoder | 编码器** | |
| `getAngleDegrees()` | Get current angle (degrees) | 获取当前角度 |
| `getAngleRadians()` | Get current angle (radians) | 获取当前角度（弧度） |
| `resetEncoder()` | Reset encoder to zero | 重置编码器 |
| `isCalibrated()` | Check if calibrated | 检查是否已校准 |
| **Lock Modes | 锁定模式** | |
| `enableSoftLock()` | Enable soft lock at 0° ⚠️ Blocked during unwind | 启用软锁定（0°） |
| `enableSoftLock(angle)` | Enable soft lock at angle | 启用软锁定（指定角度） |
| `enableHardLockBlue()` | Hard lock to blue goal (tag 20) | 硬锁定蓝框 |
| `enableHardLockRed()` | Hard lock to red goal (tag 24) | 硬锁定红框 |
| `disableLock()` | Return to manual control ⚠️ Blocked during unwind | 返回手动控制 |
| `getLockMode()` | Get current lock mode | 获取当前锁定模式 |
| **Position Tracking | 位置跟踪** | |
| `updateRobotPosition(x, y, heading)` | Update robot position | 更新机器人位置 |
| `updateTx(tx, valid, tagId)` | Update TX value with tag ID | 更新TX值和标签ID |
| `calculateAngleToGoal()` | Calculate angle to goal | 计算到目标角度 |
| `getDistanceToGoal()` | Get distance to goal | 获取到目标距离 |
| `isOnTarget()` | Check if aimed at goal | 检查是否瞄准目标 |
| **Tracking State | 跟踪状态** | |
| `isTxTrackingActive()` | Check if TX tracking is active | 检查TX跟踪是否激活 |
| `getTrackingModeString()` | Get current mode ("TX_TRACKING"/"INERTIAL"/"UNWINDING") | 获取当前模式 |
| `isUnwinding()` | Check if turret is unwinding | 检查是否正在回正 |
| `forceStopUnwind()` | ⚠️ Emergency cancel unwind | 紧急取消回正 |
| `getCurrentDetectedTagId()` | Get currently detected tag ID | 获取当前检测到的标签 |
| `getTargetTagId()` | Get target tag ID (20 or 24) | 获取目标标签ID |
| **Geometry | 几何** | |
| `getLimelightDistanceFromCenterMM()` | Limelight to chassis center | Limelight到底盘中心距离 |

#### Configuration | 配置

| Parameter | Default | Description |
|-----------|---------|-------------|
| `turretMotorName` | `"turretMotor"` | Motor hardware name (encoder wired to this port) |
| `ENCODER_CPR` | 8192 | REV Through Bore Encoder V2 counts per revolution |
| `GEAR_RATIO` | 116/22 ≈ 5.27 | Motor turns 116× for turret to turn 22× |
| `minAngleDeg` | **-190°** | Left limit (updated 2026-02-03) |
| `maxAngleDeg` | **190°** | Right limit (updated 2026-02-03) |
| `unwindThreshold` | 100° | Unwind to 0° if target exceeds this |
| `kP` | 0.0004 | Position P gain ✅ Tuned 2026-02-02 |
| `kI` | 0.0 | Not used |
| `kD` | 0.0000185 | Position D gain (damping) |
| `kF` | 0.058 | Static friction (direction-aware!) |
| `tolerance` | 100 ticks | ~0.83° position tolerance |
| `reverseMotor` | true | Motor direction REVERSED |
| `blueGoalX/Y` | (4, 140) | Blue basket position |
| `redGoalX/Y` | (140, 140) | Red basket position |

#### Encoder Configuration | 编码器配置

The turret uses a **REV Through Bore Encoder V2** mounted on the motor shaft.

云台使用安装在电机轴上的 **REV Through Bore Encoder V2**。

- **Hardware**: REV Through Bore Encoder V2 (REV-11-3174)
- **Mounting**: On motor shaft, wired to motor encoder port on Hub
- **Reading Method**: `turretMotor.getCurrentPosition()`
- **CPR**: 8192 counts per revolution (incremental mode)

**Wiring | 接线:**
- Encoder 信号线接到 Control Hub / Expansion Hub 的电机编码器端口
- 通过 `turretMotor.getCurrentPosition()` 读取（不需要单独的 encoder 配置）

#### Gear Ratio | 齿比

**Actual Gear Ratio**: Motor turns 116 times → Turret turns 22 times

**实际齿轮比**: 电机转 116 圈 → 云台转 22 圈

```
GEAR_RATIO = 116 / 22 ≈ 5.2727
```

```
Turret Angle = (Encoder Ticks / ENCODER_CPR / GEAR_RATIO) × 360°
```

| GEAR_RATIO | Meaning | 含义 |
|------------|---------|------|
| 5.27 (116/22) | **Current setup** | **当前设置** |
| 1.0 | Direct drive (1:1) | 直连 |
| 10.0 | Motor spins 10× for turret to spin 1× | 10:1 减速 |

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
| `TeleOpDriveCommand` | Field-centric drive (manual) | 场地坐标系驾驶（手动） |
| `TransitCommand` | Fire when shooter ready | 转速就绪时发射 |

### Auto Commands | 自动命令

| Command | Function | 功能 |
|---------|----------|------|
| `AutoDriveCommand` | Follow PathChain with timeout | 跟踪路径链（带超时） |
| `AutoTransitCommand` | Fire with position check | 位置检查后发射 |
| `AutoAlignCommand` | Turn to align with goal tag | 转向对准目标标签 |
| `HoldPositionCommand` | Hold current position | 保持当前位置 |
| `WaitForTurretCommand` | Wait for turret to reach target | 等待云台到达目标角度 |

#### WaitForTurretCommand

Waits for turret to reach its target angle before proceeding. Used in auto to ensure turret is aimed before shooting.

等待云台到达目标角度后再继续。用于自动程序确保云台瞄准后再射击。

```java
// Usage example | 使用示例
new WaitForTurretCommand(turret)           // Default 1s timeout
new WaitForTurretCommand(turret, 1500)     // Custom 1.5s timeout
```

**Finish Conditions | 结束条件**:
1. `turret.isOnTarget()` returns true | 云台到位
2. Timeout reached | 超时

---

## 4. TeleOp Structure | 手动程序结构

### Available OpModes | 可用操作模式

| OpMode | Name | Purpose | 用途 |
|--------|------|---------|------|
| `Solo.java` | "Solo" | **Chassis auto-aim**, turret fixed at 0° | **底盘自瞄**，云台固定0° |
| `SoloBlue.java` | "Solo Blue" | **Turret auto-aim** for Blue alliance | **云台自瞄**，瞄蓝框 |
| `SoloRed.java` | "Solo Red" | **Turret auto-aim** for Red alliance | **云台自瞄**，瞄红框 |
·| `SoloEmergency.java` | "!!! EMERGENCY !!!" | **Backup** - No sensors, pure open-loop | **紧急备用** - 无传感器纯开环 |

### OpMode Comparison | 操作模式对比

| Feature | Solo | SoloBlue | SoloRed |
|---------|------|----------|---------|
| **Turret Mode** | 🔒 Always SOFT_LOCK | Can toggle | Can toggle |
| **A Button (Soft Lock)** | ✅ Chassis auto-aim | ✅ Chassis auto-aim | ✅ Chassis auto-aim |
| **A Button (Hard Lock)** | ❌ N/A | ❌ Disabled | ❌ Disabled |
| **Hard Lock Turret** | ❌ Not supported | ✅ Aim at blue (4,140) | ✅ Aim at red (140,140) |
| **Alliance** | None | BLUE | RED |

**Key Behavior | 关键行为:**
- **Soft Lock + A Button** → Chassis rotates to aim at goal (turret stays at 0°)
- **Hard Lock** → Turret auto-rotates to aim (chassis manual control only)

### SoloEmergency - Backup OpMode | 紧急备用程序

**File | 文件**: `opmodes/teleops/SoloEmergency.java`

**Name**: "!!! EMERGENCY !!!" (group: !Emergency)

**When to Use | 何时使用**:
- Pinpoint odometry not working | 里程计不工作
- Limelight disconnected | Limelight 断开
- Subsystem initialization crash | 子系统初始化崩溃
- Need robot-centric drive | 需要机器人坐标系驾驶

**Features | 特点**:

| Feature | Description | 特点 | 说明 |
|---------|-------------|------|------|
| **Drive** | Robot-centric (no field-centric) | **驾驶** | 机器人坐标系 |
| **Shooter** | Pure open-loop power (no velocity feedback) | **发射** | 纯开环功率 |
| **Transit** | 1 second spin-up wait before fire | **传输** | 1秒预热后发射 |
| **Turret** | D-Pad manual control only | **云台** | 仅D-Pad手动控制 |

**Open-Loop Power Settings | 开环功率设置**:

| Mode | Power | Servo Angle |
|------|-------|-------------|
| SLOW | 0.30 | 0.04 |
| MID | 0.45 | 0.5 |
| FAST | 0.60 | 1.0 |
| IDLE | 0.27 | 0.5 |

**Controls | 操作**:
- Left Stick: Move (robot-centric) | 移动（机器人坐标系）
- Right Stick: Turn | 转向
- LB: Slow shot | 近射
- RB: Mid shot | 中射
- RT: Fast shot | 远射
- LT: Intake + Fire (after 1s spin-up) | 进球 + 发射
- D-Pad Up: Reverse intake | 反转进球
- D-Pad L/R: Turret manual | 云台手动

> ⚠️ **WARNING**: This mode has no safety features! Use only when main programs fail.
> 此模式无安全功能！仅在主程序失效时使用。

**Entry Point | 入口点**: `opmodes/teleops/Solo.java` (or `SoloBlue.java`, `SoloRed.java`)

```
Solo.java / SoloBlue.java / SoloRed.java
    │
    ├── Robot.java (TeleOp Container - NOT for Auto!)
    │   ├── MecanumDrivePinpoint  ← Only in TeleOp | 仅手动模式
    │   ├── Shooter
    │   ├── Transit
    │   ├── Intake
    │   ├── Vision (⚠️ disabled)
    │   └── Turret (⚠️ disabled)
    │
    ├── DriverControls.bind() (Gamepad bindings | 手柄绑定)
    │
    └── TeleOpDriveCommand (Default drive command | 默认驾驶命令)
```

### Execution Flow | 执行流程

1. `initialize()`: Create Robot, bind controls, register drive command, set turret alliance (Blue/Red only)
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
    protected Turret turret;      // Turret control
    
    public abstract Command runAutoCommand();  // Define your sequence | 定义序列
    public abstract Pose getStartPose();       // Define start position | 定义起始位置
}
```

#### Auto-Start Features | 自动启动功能

When auto starts (`waitForStart()`):

自动程序启动时：

```java
// Intake runs at full power for entire auto
intake.setFullPower(true);
intake.startIntake();
```

#### Auto Cleanup | 自动清理

When auto ends (`onAutoStopped()`):

自动程序结束时：

```java
intake.stopIntake();
intake.setFullPower(false);
shooter.setShooterState(Shooter.ShooterState.STOP);
turret.enableSoftLock(0);  // Return to forward
```

#### Safety Check | 安全检查

The base class includes an automatic position bounds check:

基类包含自动位置边界检查：

```java
// If X or Y < -10 inches, something is seriously wrong
if (currentPose.getX() < -10 || currentPose.getY() < -10) {
    // Emergency stop!
    // 紧急停止！
}
```

**When triggered | 触发时**:
- Robot immediately stops all motion
- Error message displayed: "自动定位错误！"
- Robot stays stopped until manually terminated
- Prevents runaway from localization errors

机器人立即停止，显示错误信息，直到手动终止。

### 5.1 Far Auto | 远起点自动

**Files | 文件**: `BlueFarAuto.java`, `RedFarAuto.java`

**Strategy | 策略**: Continuous intake/shoot cycles from far starting position.

从远起点位置持续执行采集/发射循环。

#### Available Programs | 可用程序

| Program | Alliance | Starting Position |
|---------|----------|-------------------|
| Blue Far Auto | Blue | (54.97, 9.10) |
| Red Far Auto | Red | (89.03, 9.10) |

#### Path Structure | 路径结构

```
Start → Intake Approach (curve) → Intake Position → [Wait] → Shoot Position → [Shoot]
     ↓
     └── LOOP: Intake → [Wait] → Shoot (repeats until auto ends)
```

#### Key Positions | 关键位置

| Position | Blue (x, y) | Red (x, y) |
|----------|-------------|------------|
| START | (54.97, 9.10) | (89.03, 9.10) |
| INTAKE | (9.60, 8.13) | (134.40, 8.13) |
| SHOOT | (54.97, 9.10) | (89.03, 9.10) |

#### Turret Control | 云台控制

| Alliance | Turret Angle | Target |
|----------|--------------|--------|
| Blue | +21.3° (right) | Blue basket (4, 140) |
| Red | -21.3° (left) | Red basket (140, 140) |

#### Sequence | 序列

```java
new SequentialCommandGroup(
    new AutoDriveCommand(follower, path1),  // Curve to approach
    new AutoDriveCommand(follower, path2),  // To intake
    intakeWaitCommand(),                     // Wait 1s
    
    // Turret starts moving BEFORE drive to shoot
    new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
    new AutoDriveCommand(follower, path3),  // To shoot (turret moving)
    shootAfterTurretReady(),                 // Wait for turret → shoot
    
    // LOOP: Repeats 10 times
    oneCycleCommand(),
    // ...
);
```

### 5.2 Near Auto | 近起点自动

**Files | 文件**: `BlueNearAuto.java`, `RedNearAuto.java`, `BlueNearInfinite.java`, `RedNearInfinite.java`

**Strategy | 策略**: Sample collection from multiple positions with shooting cycles.

从多个位置采集样本并发射。

#### Available Programs | 可用程序

| Program | Alliance | Description |
|---------|----------|-------------|
| Blue Near Auto | Blue | 9-path sample collection |
| Red Near Auto | Red | Mirrored from Blue |
| Blue Near Infinite | Blue | Extended with 3x Sample 2 cycles |
| Red Near Infinite | Red | Mirrored from Blue Infinite |
| Red Near Auto 2 | Red | Alternative path + RedNear continuation |

#### BlueNearAuto Path Structure | 路径结构

```
Start → Shoot → Sample1(far) → Intake1 → [Shoot] → Intake2 → [Shoot] → Sample2(bottom) → [Shoot] → Final
```

#### BlueNearInfinite Path Structure | 扩展版路径结构

```
Start → Shoot → Sample1 → Intake1 → [Shoot]
     ↓
     └── Sample2 Cycle (x3): Shoot → Sample2(145°) → [Shoot]
     ↓
     └── BlueNear Path 5+: Intake2 → [Shoot] → Sample3 → [Shoot] → Final
```

#### Key Positions (Blue) | 关键位置（蓝方）

| Position | Coordinates | Heading |
|----------|-------------|---------|
| START | (25.68, 127.97) | 90° |
| SHOOT | (45.20, 101.15) | 180° |
| SAMPLE1 | (8.79, 59.30) | 180° |
| INTAKE1 | (16.45, 69.66) | 180° |
| SAMPLE2 | (12.08, 61.02) | 145° |
| INTAKE2 | (16.17, 83.46) | 180° |
| SAMPLE3 | (14.19, 35.31) | 180° |
| FINAL | (15.96, 101.11) | 180° |

#### Coordinate Mirroring | 坐标对称

All Red variants are mirrored from Blue at x=72:
- `new_x = 144 - old_x`
- `heading 180° → 0°`
- `heading 145° → 35°`

#### Turret Control at SHOOT_POSE | 射击位置云台控制

When robot reaches SHOOT_POSE, turret aims at basket:

当机器人到达射击位置时，云台瞄准篮筐：

| Alliance | Turret Angle | Target |
|----------|--------------|--------|
| Blue | +43.3° (right) | Blue basket (4, 140) |
| Red | -43.3° (left) | Red basket (140, 140) |

**Shoot Sequence (Parallel Optimized) | 射击序列（并行优化）**:

Turret starts moving during path to shoot position, not after arrival.

云台在移动到射击点的过程中就开始转向，而不是到达后再转。

```
1. turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)  // Set target BEFORE drive
2. AutoDriveCommand (turret moves during drive)   // Parallel movement
3. WaitForTurretCommand (wait until at target)    // Precise check
4. shooter.setShooterState(SLOW)                  // Start shooting
5. Wait SHOOT_WAIT_MS (1500ms)                    // Shoot duration
6. shooter.setShooterState(STOP)                  // Stop
7. turret.enableSoftLock(0)                       // Return forward
```

**WaitForTurretCommand**: Uses `turret.isAtTarget()` with same tolerance as PIDF control.

**WaitForTurretCommand**: 使用与 PIDF 控制相同的 tolerance 检测云台是否到位。

#### Dashboard Parameters | 仪表盘参数

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TURRET_SHOOT_ANGLE_DEG` | ±43.3° (Near) / ±21.3° (Far) | Turret aim angle |
| `TURRET_TIMEOUT_MS` | 1000ms | Max wait for turret |
| `SHOOT_WAIT_MS` | 1500ms (Near) / 3000ms (Far) | Shooting duration |
| `INTAKE_WAIT_MS` | 500ms (Near) / 1000ms (Far) | Collection time |

#### Key Differences | 关键区别

| Aspect | Far Auto | Near Auto |
|--------|----------|-----------|
| Distance | ~55" to goal | ~25" to goal |
| Speed | SLOW | SLOW |
| Strategy | Continuous loops | Multi-position collection |
| Path type | Mostly straight | Mostly curved |

| 方面 | 远起点 | 近起点 |
|------|--------|--------|
| 距离 | ~55" 到目标 | ~25" 到目标 |
| 策略 | 持续循环 | 多点采集 |
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
| `TurretConstants.java` | Turret PIDF, limits, gear ratio | 云台PIDF、限位、齿比 |
| `Constants.java` | Pedro Pathing config | Pedro Pathing 配置 |
| `AutoConstants.java` | Auto positions (poses) | 自动程序位置 |
| `TeleOpConstants.java` | Trigger thresholds | 触发阈值 |

#### AutoConstants

**File | 文件**: `opmodes/autos/AutoConstants.java`

Stores all coordinates and poses used in autonomous programs.

存储自动程序中使用的所有坐标和位姿。

**Key Constants | 关键常量**:

| Category | Constants |
|----------|-----------|
| **Start Poses** | `BLUE_START_POSE`, `RED_START_POSE`, `BLUE_FAR_START_POSE`, `RED_FAR_START_POSE` |
| **Scoring** | `BLUE_BASKET_POSE`, `RED_BASKET_POSE` |
| **Samples** | `BLUE_SAMPLE_1_POSE`, `BLUE_SAMPLE_2_POSE`, `BLUE_SAMPLE_3_POSE` (+ RED mirrors) |
| **Gates** | `BLUE_GATE_POSE`, `RED_GATE_POSE` |
| **Safety** | `POSITION_LOWER_BOUND = -10.0` (emergency stop trigger) |

**Coordinate Mirror | 坐标镜像**:
- `new_x = 144 - old_x`
- Heading 180° → 0°

#### TeleOpConstants

**File | 文件**: `opmodes/teleops/TeleOpConstants.java`

Trigger thresholds for gamepad controls. Dashboard tunable (`@Config`).

手柄触发阈值。可通过 Dashboard 调整（`@Config`）。

| Parameter | Default | Description |
|-----------|---------|-------------|
| `slowShootTriggerThreshold` | 0.5 | RT threshold for fast shot |
| `transitFireTriggerThreshold` | 0.3 | LT threshold for transit fire |
| `intakeFullPowerTriggerThreshold` | 0.5 | LT threshold for full power intake |

### FTC Dashboard Tuning | FTC Dashboard 调参

All `@Config` annotated classes can be tuned in real-time:
1. Connect to `http://192.168.43.1:8080/dash`
2. Find class under "Configuration"
3. Modify values live

所有带 `@Config` 注解的类可以实时调参：
1. 连接 `http://192.168.43.1:8080/dash`
2. 在 "Configuration" 下找到类
3. 实时修改参数

### Test OpModes | 测试程序

| OpMode | Purpose | 用途 |
|--------|---------|------|
| `Drive Only Test` | Drivetrain-only test (4 motors + Pinpoint) | 仅底盘测试 |
| `DashTuner` | Generic PIDF tuning with encoder safety | 通用PIDF调参 + 编码器保护 |
| `Turret Motor Tuner` | Turret-specific PIDF tuning ✅ | 云台专用PIDF调参 |
| `Chassis Heading Tuner` | Chassis rotation PIDF tuning (MecanumDrive) | 底盘旋转PIDF调参 |
| `Shooter PID Tuner` | Shooter velocity PIDF tuning (Dashboard only) | 发射器速度PIDF调参 |
| `Tuning` | Pedro Pathing comprehensive tuning (menu) | Pedro Pathing综合调参（菜单） |
| `PathTunerOpMode` | Path visualization on Panels Dashboard | 路径可视化调试 |
| `ColorSensorTest` | REV Color/Distance sensor test | 颜色/距离传感器测试 |

#### Drive Only Test

Standalone drivetrain test with field-centric drive. Same logic as main TeleOp but without other subsystems.

独立底盘测试，场地相对驾驶。与主 TeleOp 逻辑相同但不包含其他子系统。

**Controls | 操作**:
- Left Stick: Move (field-centric) | 移动（场地相对）
- Right Stick: Turn | 旋转
- Left Stick Click: Reset heading | 重置朝向

#### DashTuner

General-purpose PIDF tuning OpMode via FTC Dashboard. Supports up to 4 motors and 4 servos simultaneously.

通用 PIDF 调参程序，通过 FTC Dashboard 控制。支持同时调试最多 4 个电机和 4 个舵机。

**Dashboard Parameters | Dashboard 参数**:
| Parameter | Type | Description | 说明 |
|-----------|------|-------------|------|
| `motorName[0-3]` | String | Motor config names | 电机配置名 |
| `servoName[0-3]` | String | Servo config names | 舵机配置名 |
| `closeLoop[0-3]` | boolean | Enable PIDF | 启用闭环 |
| `isVelocityCloseLoop[0-3]` | boolean | Velocity (true) or Position (false) | 速度/位置闭环 |
| `motorTarget[0-3]` | double | Target velocity (TPS) or position (ticks) | 目标速度/位置 |
| `servoTarget[0-3]` | double | Servo position 0-1 | 舵机位置 |
| `PIDFs[0-3].kP/kI/kD/kF` | double | PIDF coefficients | PIDF 系数 |

**🛡️ Encoder Safety Protection | 编码器安全保护**:

Position closed-loop mode includes automatic encoder stuck detection:

位置闭环模式包含自动编码器卡死检测：

| Parameter | Default | Description | 说明 |
|-----------|---------|-------------|------|
| `ENCODER_TIMEOUT` | 0.5s | Time before triggering | 触发前等待时间 |
| `ENCODER_MIN_CHANGE` | 5 ticks | Minimum expected change | 最小变化阈值 |
| `resetEncoderError` | false | Set true to clear error | 设为 true 清除错误 |

**Protection Behavior | 保护行为**:
1. Motor outputs power > 0.05 (trying to move) | 电机输出功率 > 0.05（尝试运动）
2. Encoder change < 5 ticks for 0.5 seconds | 编码器变化 < 5 ticks 持续 0.5 秒
3. **EMERGENCY STOP** - Motor disabled until reset | **紧急停止** - 电机禁用直到重置
4. Dashboard shows: `!!! ENCODER ERROR !!!` | Dashboard 显示错误
5. Reset: Set `resetEncoderError = true` or restart OpMode | 重置方法

> ⚠️ **Why this matters | 为什么重要**: If encoder is disconnected/broken but PIDF keeps outputting power, motor could overheat or damage mechanism. This protection prevents that.
> 
> 如果编码器断开/损坏但 PIDF 持续输出功率，电机可能过热或损坏机构。此保护可防止这种情况。

#### Turret Motor Tuner ✅

Dedicated PIDF tuner for turret motor. All control via Dashboard.

云台电机专用 PIDF 调参程序。全部通过 Dashboard 控制。

**Dashboard Parameters | Dashboard 参数**:
| Parameter | Description | 说明 |
|-----------|-------------|------|
| `enabled` | Enable motor control | 启用电机控制 |
| `targetPosition` | Target position (ticks) | 目标位置 |
| `reverseMotor` | Reverse motor direction | 反转电机方向 |
| `tolerance` | Position tolerance (ticks) | 位置容差 |
| `kP/kI/kD/kF` | PIDF coefficients | PIDF 系数 |

**Direction-Aware kF | 方向感知 kF**:
- kF automatically changes sign based on error direction | kF 根据误差方向自动变号
- Positive error → +kF, Negative error → -kF | 正误差→+kF，负误差→-kF
- This compensates static friction in BOTH directions | 补偿双向静摩擦

**Tuned Values (2026-02-02) | 调试值**:
```
kP = 0.0004
kI = 0.0
kD = 0.0000185
kF = 0.058
tolerance = 100 ticks (~0.83°)
reverseMotor = true
```

#### Chassis Align Tuner

Heading PIDF tuner for chassis rotation. Uses MecanumDrivePinpoint. Based on PedroPathing's HeadingTuner design.

底盘旋转 PIDF 调参程序。使用 MecanumDrivePinpoint。基于 PedroPathing 的 HeadingTuner 设计。

**How it Works | 工作原理**:
1. Robot locks to starting heading (0°) on start | 启动时锁定当前朝向
2. PIDF constantly tries to maintain this heading | PIDF 持续维持该朝向
3. Turn robot by hand → PIDF resists | 用手转动机器人 → PIDF 抵抗
4. Tune parameters until response is smooth | 调参直到响应平滑

**Dashboard Parameters | Dashboard 参数** (✅ Tuned 2026-02-03):
| Parameter | Default | Description |
|-----------|---------|-------------|
| `enabled` | **true** | Enable rotation control (default ON) |
| `tolerance` | 2.0 | Degrees tolerance |
| `maxPower` | **1.0** | Max rotation power |
| `kP` | **0.03** | Proportional ✅ |
| `kI` | 0.0 | Integral coefficient |
| `kD` | **0.003** | Derivative ✅ |
| `kF` | 0.0 | Static friction (not needed) |
| `iMax` | 50.0 | Max integral accumulation |
| `iZone` | 30.0 | Disable I if error > iZone |

**No Gamepad Controls** - Pure Dashboard tuning  
**无手柄控制** - 纯 Dashboard 调参

**Note**: Turret motor is auto-locked (BRAKE mode) during this test.  
**Direction-Aware kF**: Applied based on output direction - `fTerm = signum(output) * kF`

#### Shooter PID Tuner

Velocity PIDF tuner for shooter flywheel. Pure Dashboard control.

发射器飞轮速度 PIDF 调参程序。纯 Dashboard 控制。

**Dashboard Parameters | Dashboard 参数**:
| Parameter | Default | Description |
|-----------|---------|-------------|
| `enabled` | false | Enable velocity control |
| `targetVelocity` | 1000 | Target velocity (TPS) |
| `tolerance` | 50 | Velocity tolerance (TPS) |
| `kP` | 0.0005 | Proportional |
| `kI` | 0.0 | Integral |
| `kD` | 0.0 | Derivative |
| `kF` | 0.00036 | Feedforward (velocity proportional) |

**Usage | 使用方法**:
1. Set `targetVelocity` (e.g., 1500)
2. Set `enabled = true`
3. Observe velocity response on Dashboard graph
4. Tune kF first (feedforward), then kP for correction

#### Tuning (Pedro Pathing)

**File | 文件**: `tests/Tuning.java`

Comprehensive tuning OpMode from Pedro Pathing library. Select which component to tune via Dashboard menu.

Pedro Pathing 库的综合调参程序。通过 Dashboard 菜单选择要调试的组件。

**Available Tuners | 可用调试项**:

| Tuner | Purpose | 用途 |
|-------|---------|------|
| `HeadingTuner` | Heading PID | 朝向 PID |
| `TranslationalTuner` | XY translation PID | XY 平移 PID |
| `ForwardVelocityTuner` | Forward velocity | 前进速度 |
| `StrafeVelocityTuner` | Strafe velocity | 平移速度 |
| `TurnVelocityTuner` | Turn velocity | 转向速度 |
| `DriveVelocityTuner` | Drive velocity | 驱动速度 |
| `LocalizationTest` | Odometry test | 里程计测试 |

**Usage | 使用方法**:
1. Run "Tuning" OpMode
2. Use gamepad to select tuner (see on-screen menu)
3. Start selected tuner
4. Adjust parameters in Dashboard

#### PathTunerOpMode

**File | 文件**: `test/PathTunerOpMode.java`

Visualizes auto paths on Panels Dashboard for debugging path definitions.

在 Panels Dashboard 上可视化自动路径，用于调试路径定义。

**Features | 功能**:
- View BezierLine and BezierCurve paths | 查看直线和曲线路径
- Dynamic path testing via Dashboard | 通过 Dashboard 动态测试路径
- Path playback | 路径回放

#### ColorSensorTest

**File | 文件**: `tests/ColorSensorTest.java`

Test OpMode for REV Color/Distance sensor. Displays RGBA values, distance, and HSV conversion.

REV 颜色/距离传感器测试程序。显示 RGBA 值、距离和 HSV 转换。

**Telemetry Output | 遥测输出**:
- RGB and Alpha values | RGB 和透明度值
- Distance (cm) | 距离
- HSV conversion | HSV 转换

---

## 8. Control Mapping | 手柄映射

### Driver Controller (Gamepad1) | 主手柄

| Button | Function | Status | 功能 | 状态 |
|--------|----------|--------|------|------|
| **Left Stick** | Strafe | ✅ | 平移 | ✅ |
| **Right Stick** | Turn | ✅ | 转向 | ✅ |
| **Left Stick Button** | Reset heading | ✅ | 重置朝向 | ✅ |
| **Right Stick Button** | Toggle turret lock | ❌ Disabled | 切换云台锁定 | ❌ 禁用 |
| **A** | Chassis auto-aim (SOFT_LOCK only) | ⚠️ Disabled | 底盘自瞄（仅软锁） | ⚠️ 禁用 |
| **Y** | Turret go to home (physical 0°) | ✅ | 云台回到原点 | ✅ |
| **B** | Set current turret position as home | ✅ | 设置当前位置为原点 | ✅ |
| **LB** | Slow shot (700 TPS) | ✅ | 近射 | ✅ |
| **RB** | Mid shot (950 TPS) | ✅ | 中射 | ✅ |
| **RT** | Fast shot (1420 TPS) | ✅ | 远射 | ✅ |
| **LT** | Full power intake (1.0) + Transit fire | ✅ | 全功率进球 + 发射 | ✅ |
| **X** | Adaptive fire (Blue) | ❌ Disabled | 自适应发射（蓝） | ❌ 禁用 |
| **D-Pad Up** | Reverse intake | ✅ | 反转进球 | ✅ |
| **D-Pad Left** (hold) | Turret CCW (power 0.5) | ✅ | 云台逆时针转（开环） | ✅ |
| **D-Pad Right** (hold) | Turret CW (power -0.5) | ✅ | 云台顺时针转（开环） | ✅ |
| **D-Pad Down** | ~~Manual brake~~ | ❌ Removed | ~~手动刹车~~ | ❌ 已删除 |

### Secondary Controller (Gamepad2) | 副手柄

| Button | Function | 功能 |
|--------|----------|------|
| **Right Stick Button** | Set current turret position as home | 设置当前云台位置为原点 |

> **Note | 注意**: Emergency disable combos have been removed. Gamepad2 is now only used for turret home calibration.
> 紧急禁用组合键已被删除。Gamepad2 现在仅用于云台原点校准。

#### A Button Behavior | A 键行为

| Mode | A Button Effect | A 键效果 |
|------|-----------------|----------|
| **SOFT_LOCK** | Chassis auto-aim (rotate to face goal) | 底盘自瞄（转向对准目标） |
| **HARD_LOCK** | ❌ No effect (turret handles aiming) | ❌ 无效果（云台负责瞄准） |
| **MANUAL** | ❌ No effect | ❌ 无效果 |

> **Note**: A button triggers chassis auto-aim ONLY. Shoot buttons (LB/RB/RT) do NOT trigger auto-aim.
> **注意**: A 键仅触发底盘自瞄。射击键不触发自瞄。

#### Turret Manual Control (Solo) | 云台手动控制

| Control | Action | 控制 | 动作 |
|---------|--------|------|------|
| **D-Pad Left** (hold) | Rotate CCW at power 0.5 | 按住 | 逆时针转（0.5功率） |
| **D-Pad Right** (hold) | Rotate CW at power -0.5 | 按住 | 顺时针转（-0.5功率） |
| **Release** | Stop (power 0) | 松开 | 停止 |
| **Y** | Go to home position | 按下 | 回到原点 |
| **B** | Set current position as home | 按下 | 设置当前为原点 |

> **Open Loop**: D-Pad control is pure open-loop (direct power, no PID).  
> **开环控制**: D-Pad 是纯开环控制（直接设置功率，无 PID）。

> **Home Function**: Encoder preserves absolute position across restarts. Press B to set current position as "home", then Y to return to it.  
> **归位功能**: Encoder 会在重启后保留绝对位置。按 B 设置当前位置为"原点"，按 Y 返回原点。

---

## Quick Reference Card | 快速参考卡

### Shooter Velocities | 发射器转速

| State | TPS | 状态 | 转速 |
|-------|-----|------|------|
| STOP | 600 | 停止/怠速 | 600 |
| SLOW | **950** | 近射 | **950** |
| MID | **1500** | 中射 | **1500** |
| FAST | **2100** | 远射 | **2100** |

> **Note**: Velocities are now positive values (updated 2026-02-03)  
> **注意**: 转速现在使用正值（2026-02-03 更新）

### Servo Positions | 舵机位置

| Servo | Position | Description | 舵机 | 位置 | 说明 |
|-------|----------|-------------|------|------|------|
| Shooter Angle | 0.04 | Near (SLOW) | 发射角度 | 0.04 | 近射 |
| Shooter Angle | 0.5 | Mid (STOP/MID) | 发射角度 | 0.5 | 中位/待机 |
| Shooter Angle | 1.0 | Far (FAST) | 发射角度 | 1.0 | 远射 |
| Transit | 0.62 (down) | Loading | 传输 | 0.62 | 装填 |
| Transit | 0.36 (up) | Firing | 传输 | 0.36 | 发射 |
| Limit | 0.3 (closed) | Transit down | 限位 | 0.3 | 传输放下时 |
| Limit | 0.6 (open) | Transit up | 限位 | 0.6 | 传输抬起时 |

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
| **Vision Calibration | 视觉校准** | |
| `visionCalibrate(vision, alliance)` | Calibrate odometry from vision | 从视觉校准里程计 |
| `hasVisionCalibrated()` | Check if calibrated | 检查是否已校准 |
| `getAbsolutePose()` | Get absolute pose | 获取绝对位姿 |
| **Absolute Position | 绝对位置** | |
| `updateAbsolutePositionFromVisionWithTurret(vision, turretAngleRad)` | **NEW** Update with turret compensation | 带云台补偿的视觉更新 |
| `updateAbsolutePositionFromVision(vision)` | ~~Deprecated~~ Legacy (no turret) | 已弃用，无云台补偿 |
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
| `getVelocity()` | Get current velocity from **right motor** (TPS) | 从**右电机**获取当前转速 |
| `getTargetVelocity()` | Get target velocity | 获取目标转速 |
| `isShooterAtSetPoint()` | Check if at target velocity | 检查是否达到目标转速 |

> **Note**: Velocity is read from right motor (`rightShooter.getVelocity()`). Right motor runs negative power, so velocity is already negative.
> **注意**: 转速从右电机读取。右电机功率为负，所以速度已经是负值。
| **Adaptive Velocity | 自适应转速** | |
| `setAdaptiveVelocity(velocity)` | Set adaptive velocity | 设置自适应转速 |
| `getAdaptiveVelocity()` | Get adaptive velocity | 获取自适应转速 |
| `setAdaptiveServoPosition(pos)` | Set adaptive servo position | 设置自适应舵机位置 |
| `getAdaptiveServoPosition()` | Get adaptive servo position | 获取自适应舵机位置 |

> **Note**: Brake servo has been removed from the codebase.
> **注意**: 刹车舵机已从代码中删除。

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

### 9.7 DashboardUtil | Dashboard 工具

**File | 文件**: `utils/DashboardUtil.java`

Utility for drawing robot visualization on FTC Dashboard field overlay.

在 FTC Dashboard 场地叠加层上绘制机器人可视化的工具。

| Method | Description | 描述 |
|--------|-------------|------|
| `drawRobot(packet, pose)` | Draw robot circle + heading on field | 在场地上绘制机器人圆圈+朝向 |

**Parameters | 参数**:
- `ROBOT_RADIUS = 9.0` inches
- `ROBOT_COLOR = "#3F51B5"` (Blue)

### 9.8 FunctionalButton | 函数式按钮

**File | 文件**: `utils/FunctionalButton.java`

Custom Button class that accepts a `BooleanSupplier` for flexible trigger conditions. Used for combining multiple gamepad inputs.

接受 `BooleanSupplier` 的自定义按钮类，用于灵活的触发条件。用于组合多个手柄输入。

```java
// Example: Fire only when LT AND (LB OR RB OR RT) pressed
new FunctionalButton(
    () -> leftTrigger && (leftBumper || rightBumper || rightTrigger)
).whenHeld(new TransitCommand(...));
```

### 9.9 Units | 单位转换

**File | 文件**: `utils/Units.java`

Comprehensive unit conversion utilities.

全面的单位转换工具。

| Method | Description | 描述 |
|--------|-------------|------|
| `metersToFeet(m)` | Meters → Feet | 米 → 英尺 |
| `feetToMeters(ft)` | Feet → Meters | 英尺 → 米 |
| `metersToInches(m)` | Meters → Inches | 米 → 英寸 |
| `inchesToMeters(in)` | Inches → Meters | 英寸 → 米 |
| `degreesToRadians(deg)` | Degrees → Radians | 度 → 弧度 |
| `radiansToDegrees(rad)` | Radians → Degrees | 弧度 → 度 |
| `radiansToRotations(rad)` | Radians → Rotations | 弧度 → 圈数 |
| `degreesToRotations(deg)` | Degrees → Rotations | 度 → 圈数 |
| `rotationsToDegrees(rot)` | Rotations → Degrees | 圈数 → 度 |
| `rotationsToRadians(rot)` | Rotations → Radians | 圈数 → 弧度 |
| `rotationsPerMinuteToRadiansPerSecond(rpm)` | RPM → rad/s | 转/分 → 弧度/秒 |
| `millisecondsToSeconds(ms)` | ms → s | 毫秒 → 秒 |
| `secondsToMilliseconds(s)` | s → ms | 秒 → 毫秒 |
| `mmToInches(mm)` | mm → Inches | 毫米 → 英寸 |
| `inchesToMm(in)` | Inches → mm | 英寸 → 毫米 |

### 9.10 DriverControls | 驾驶员控制绑定

**File | 文件**: `controls/DriverControls.java`

Centralizes all gamepad button bindings. Called from TeleOp `initialize()`.

集中管理所有手柄按钮绑定。从 TeleOp 的 `initialize()` 调用。

**Method Signature | 方法签名**:
```java
public static void bind(GamepadEx gamepad, Robot robot, boolean[] isAuto)
```

**Bindings Created | 创建的绑定**:

| Button | Action |
|--------|--------|
| Left Stick Button | Reset heading |
| Left Bumper | SLOW shot (hold) |
| Right Bumper | MID shot (hold) |
| Right Trigger ≥ 0.5 | FAST shot (hold) |
| Left Trigger ≥ 0.3 + Shoot | Transit fire |
| D-Pad Up | Reverse intake |
| Left Trigger ≥ 0.5 | Full power intake |

> **Note**: Turret lock mode toggle (Right Stick Button) is currently commented out/disabled.
> 云台锁定模式切换（右摇杆按钮）当前已注释/禁用。

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
| **Velocity oscillates | 转速振荡** | Using PID instead of Pseudo Closed-loop | Use Pseudo Closed-loop | `Shooter.java` → `periodic()` |
| **Ball doesn't fire | 球不发射** | Velocity not reached | Check `isShooterAtSetPoint()` | `Shooter.java` → `isShooterAtSetPoint()` |
| **Wrong velocity for distance | 距离对应转速错误** | Calibration data | Update distance/velocity constants | `ShooterConstants.java` → `nearDistance`, `midDistance`, `farDistance` |
| **Servo angle wrong | 舵机角度错误** | Servo position values | Adjust `shooterServoDownPos/MidPos/UpPos` | `ShooterConstants.java` → servo positions |

| 问题 | 可能原因 | 解决方案 | 文件 & 方法 |
|------|----------|----------|-------------|
| **发射器达不到转速** | 功率太低/电机弱 | 检查电机/增加前馈 | `Shooter.java` → `periodic()` |
| **转速振荡** | 使用PID而非伪闭环 | 使用伪闭环控制 | `Shooter.java` → `periodic()` |
| **球不发射** | 转速未达到 | 检查 `isShooterAtSetPoint()` | `Shooter.java` → `isShooterAtSetPoint()` |
| **距离对应转速错误** | 校准数据 | 更新距离/转速常量 | `ShooterConstants.java` → 距离常量 |
| **舵机角度错误** | 舵机位置值 | 调整舵机位置常量 | `ShooterConstants.java` → 舵机位置 |

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

### 10.4 Intake Issues | 进球问题

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

### 10.5 Transit Issues | 传输问题

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

### 10.6 Autonomous Issues | 自动问题

| Problem | Possible Cause | Solution | File & Method |
|---------|----------------|----------|---------------|
| **Robot doesn't move | 机器人不动** | Path not built | Check `buildPaths()` | Auto file → `buildPaths()` |
| **Wrong starting position | 起始位置错误** | Start pose | Check `getStartPose()` | Auto file → `getStartPose()` |
| **Path goes wrong way | 路径走错方向** | Pose coordinates | Check X/Y/heading values | Auto file → pose definitions |
| **Timeout before finish | 超时未完成** | Timeout too short | Increase `withTimeout()` value | Auto file → command timeouts |
| **Follower oscillates | 跟踪器振荡** | PID tuning | Adjust Pedro Pathing constants | `Constants.java` → PID values |
| **"自动定位错误！" error | 定位错误** | Position < -10 | Localization failed, check odometry/vision | `AutoCommandBase.java` → safety check |

| 问题 | 可能原因 | 解决方案 | 文件 & 方法 |
|------|----------|----------|-------------|
| **机器人不动** | 路径未构建 | 检查 `buildPaths()` | 自动文件 → `buildPaths()` |
| **起始位置错误** | 起始位姿 | 检查 `getStartPose()` | 自动文件 → `getStartPose()` |
| **路径走错方向** | 位姿坐标 | 检查X/Y/朝向值 | 自动文件 → 位姿定义 |
| **超时未完成** | 超时太短 | 增加 `withTimeout()` 值 | 自动文件 → 命令超时 |
| **跟踪器振荡** | PID调参 | 调整 Pedro Pathing 常量 | `Constants.java` → PID值 |
| **"自动定位错误！"** | 位置 < -10 | 定位失败，检查里程计/视觉 | `AutoCommandBase.java` → 安全检查 |

### 10.7 Turret Issues | 云台问题

| Problem | Possible Cause | Solution | File & Method |
|---------|----------------|----------|---------------|
| **Turret doesn't move | 云台不动** | Motor name wrong | Check hardware map | `TurretConstants.java` → `turretMotorName` |
| **Angle reading wrong | 角度读数错误** | Encoder not calibrated | Call `resetEncoder()` | `Turret.java` → `resetEncoder()` |
| **Can't switch modes | 无法切换模式** | Turret is unwinding | Wait for unwind to complete | Check `isUnwinding()` |
| **Hard Lock doesn't aim | 硬锁定不瞄准** | No absolute position | Check `hasAbsolutePosition()` | `MecanumDrivePinpoint.java` |
| **TX tracking not working | TX跟踪不工作** | Seeing wrong tag | Check `getTrackingModeString()` | Should be "TX_TRACKING" |
| **Always using inertial | 一直用惯性** | tagId ≠ targetTagId | Check `getCurrentDetectedTagId()` vs `getTargetTagId()` |
| **PID oscillates | PID振荡** | kP too high | Reduce `kP` | `TurretConstants.java` → PID values |
| **Turret keeps unwinding | 一直回正** | Target behind robot | Target > 100°, normal behavior | `unwindThreshold` |
| **Turret hits limit | 云台撞限位** | Software limit wrong | Adjust `minAngleDeg/maxAngleDeg` | `TurretConstants.java` |

| 问题 | 可能原因 | 解决方案 | 文件 & 方法 |
|------|----------|----------|-------------|
| **云台不动** | 电机名称错误 | 检查硬件映射 | `TurretConstants.java` → 电机名称 |
| **角度读数错误** | 编码器未校准 | 调用 `resetEncoder()` | `Turret.java` → `resetEncoder()` |
| **无法切换模式** | 云台正在回正 | 等待回正完成 | 检查 `isUnwinding()` |
| **硬锁定不瞄准** | 没有绝对位置 | 检查 `hasAbsolutePosition()` | `MecanumDrivePinpoint.java` |
| **TX跟踪不工作** | 看到的是对方tag | 检查 `getTrackingModeString()` | 应该显示 "TX_TRACKING" |
| **一直用惯性导航** | tagId ≠ targetTagId | 检查当前tag和目标tag | 对比两个ID |
| **PID振荡** | kP过高 | 减小 `kP` | `TurretConstants.java` → PID值 |
| **一直回正** | 目标在机器人后方 | 目标角度>100°，正常行为 | `unwindThreshold` |
| **云台撞限位** | 软件限位错误 | 调整角度限制 | `TurretConstants.java` |

---

## 11. Debug Telemetry Reference | 调试遥测参考

### TeleOp Telemetry | 手动遥测数据

Available in `Solo.java` / `SoloBlue.java` / `SoloRed.java`:

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
| **Turret | 云台** | |
| `Lock Mode` | MANUAL/SOFT_LOCK/HARD_LOCK | 锁定模式 |
| `Tracking Mode` | TX_TRACKING/INERTIAL/UNWINDING | 跟踪模式 |
| `Angle` | Current turret angle (°) | 当前云台角度 |
| `Target` | Target turret angle (°) | 目标云台角度 |
| `On Target` | Hard lock: aiming at goal? | 硬锁定：是否瞄准目标 |
| `Dist to Goal` | Hard lock: distance to goal | 硬锁定：到目标距离 |
| `Detected Tag` | Currently detected tag ID | 当前检测到的标签 |
| `Target Tag` | Alliance target tag (20/24) | 联盟目标标签 |
| `TX Active` | Is TX tracking active? | TX跟踪是否激活 |
| `Unwinding` | Is turret unwinding to 0°? | 是否正在回正 |

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
- [ ] Turret calibrated? `turret.isCalibrated()` = true (reset encoder when forward)
- [ ] Turret in Soft Lock? Check telemetry shows SOFT_LOCK

### During Match Issues | 比赛中问题

| Symptom | Quick Check | 症状 | 快速检查 |
|---------|-------------|------|----------|
| Can't shoot | Check shooter TPS in telemetry | 不能发射 | 检查遥测中的发射器转速 |
| Vision not working | Check if tag is visible | 视觉不工作 | 检查是否能看到标签 |
| Robot drifts | Reset heading (Left Stick) | 机器人漂移 | 重置朝向（左摇杆） |
| Balls not feeding | Check transit servo | 球不传输 | 检查传输舵机 |

---

*End of Guide | 指南结束*


