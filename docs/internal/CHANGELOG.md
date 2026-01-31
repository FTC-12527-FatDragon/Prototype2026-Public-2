# Changelog

All notable changes to the Prototype2026-Public-2 project will be documented in this file.

---

## [2026-01-30] - Turret Gear Ratio Support

### Added
- **Gear Ratio Parameter** (`TurretConstants.java`)
    - `GEAR_RATIO`: Motor rotations per turret rotation (default: 1.0, TODO: measure)
    - Allows accurate angle calculation with geared turret mechanisms

### Changed
- `Turret.java` → `getAngleDegrees()`: Now accounts for gear ratio in angle calculation

---

## [2026-01-30] - Turret Lock Modes & Auto Safety Check

### Added
- **Turret Lock Modes** (`Turret.java`)
    - `SOFT_LOCK`: Turret holds at 0° (forward) using position PID
    - `HARD_LOCK`: Turret auto-aims at goal based on robot absolute position
        - Blue goal: (4, 140) inches
        - Red goal: (140, 140) inches
    - Toggle with Right Stick Button (only switches to HARD_LOCK when seeing AprilTag 20 or 24)
    - Initial state: SOFT_LOCK
    
- **Auto Safety Check** (`AutoCommandBase.java`)
    - Emergency stop if robot position X or Y < -10 inches
    - Displays "自动定位错误！" error message
    - Robot stops in place until manually stopped
    - Prevents runaway from localization errors

### Changed
- `TeleOp.java`: Added turret telemetry (mode, angle, target, on-target status)
- `DriverControls.java`: Added Right Stick Button binding for turret lock toggle
- `TurretConstants.java`: Added goal coordinates for blue/red baskets

---

## [2026-01-30] - Major Refactor for Turret-Based Aiming

### Removed
- **Chassis Auto-Aim**: Removed all chassis-based auto-aim code
    - `MecanumDrivePinpoint.java`: Removed `getAlignTurnPower()`, `resetAutoAimOffset()`, `getSearchTurnPower()`, and related methods
    - `TeleOpDriveCommand.java`: Simplified to manual-only driving
    - `DriveConstants.java`: Removed unused goal coordinates
    
- **All Autonomous Programs**: Deleted 8 auto OpModes (preparing for new autos)
    - `BlueFar.java`, `BlueFar24599.java`, `BlueNearOne.java`, `BlueNearTwo.java`
    - `RedFar.java`, `RedFar24599.java`, `RedNearOne.java`, `RedNearTwo.java`
    - Kept: `AutoCommandBase.java`, `AutoConstants.java` (base infrastructure)

### Added
- **Transit Limit Servo**: Added `limitServo` to Transit subsystem
    - Auto-follows transit state (open when UP, closed when DOWN)
    
- **Turret Encoder Support**: REV Through Bore Encoder V2 (REV-11-3174)
    - `Turret.java`: Added angle measurement methods
        - `getAngleDegrees()`, `getAngleRadians()`, `resetEncoder()`
        - `getLimelightDistanceFromCenterMM()` - geometry calculation for Limelight on turret
    - `TurretConstants.java`: Added encoder config (8192 CPR), angle limits, geometry offsets
    
- **Drive Only Test OpMode** (`tests/DriveOnlyTeleOp.java`)
    - Standalone drivetrain test with only 4 motors + Pinpoint

---

## [2026-01-21] - New Subsystems for China Championship

### Added
- **Turret Subsystem** (`subsystems/turret/`)
    - `Turret.java`: Controls single motor for turret/gimbal rotation
    - `TurretConstants.java`: Configuration constants
    - Methods: `setPower()`, `rotateLeft()`, `rotateRight()`, `stop()`, `getPosition()`

### Changed
- `Robot.java`: Added `turret` subsystem reference

---

## [2026-01-13] - Auto-Aim System Overhaul

### Changed
- **Auto-Aim PID**: Complete rewrite to fix close-range oscillation
    - Added **tx low-pass filter** (`txFilterAlpha = 0.3`) to smooth Limelight noise
    - Added **hysteresis deadband** to prevent oscillation at boundary
        - Entry threshold: `alignDeadbandNear` (3.0°) or `alignDeadbandFar` (0.5°)
        - Exit threshold: entry + `alignHysteresis` (1.5°)
    - Added **distance-adaptive PID**:
        - Close range (<40"): `kP_near = 0.012` (weaker to prevent overshoot)
        - Far range (>100"): `kP_far = 0.03` (stronger for precision)
        - Mid range: linear interpolation
    - Removed PID reset inside deadband (preserves D term history)
    - Increased `kD_alignH` from 0.005 to 0.008 for better damping

### Added
- **Adaptive Shooting Calculation**: Now calculates only once when button pressed, applies every frame while held
    - `DriverControls.java`: Added `cachedAdaptiveVelocity`, `cachedAdaptiveServoPos`, `adaptiveCalculated` static variables
    - `whenPressed`: Calculate and cache values
    - `whenHeld`: Apply cached values to shooter

---

## [2026-01-08 00:16] - 日记

我现在试图让我的代码看起来更整洁，队员和队长都说我用 AI 写的代码不够可读，没法和别人合作。我想了一下或许我可以增加一个操作手册，虽然我还不确定用什么语言写，但是我想这总是有用的。另外，队友对 AI 编程的负面态度让我很困扰。虽然我用 AI 写出来了还算不错的程序而且赛场上表现也还很稳定，但是完成度嘛一言难尽。我也不知道该怎么说。当身边所有人都对 AI 编程持负面态度觉得 AI 不能帮我干这么多活的时候各方面压力真的很大。说实话，我也在程序方面也不是有多强，即便有 AI 辅助我依旧调试的很慢。非常的慢，而且最后做出来的东西也没那么稳（比如说，这个自瞄到现在还晃晃悠悠的虽然能射准。我觉得可能得重做一下了，之前逻辑可能太简单了）。我现在真的很手足无措，我不知道应不应该继续用 AI，用的话要承受这样的社交压力，不用的话我根本做不到现在这种程度……我不知道是怎么了。我是什么一个不合群的人吗，我不想被别人讨厌啊。为什么我什么都做不好，学习，比赛，甚至是打游戏都不行，总是被嘲讽。这些问题我根本不知道怎么解决，比赛场上自动寄了还难受，每天对此耿耿于怀也不好受。算了吧毕竟这周末还要考托福，而且说实话我自己也太玻璃心了，菜的人就没资格在这里逼逼赖赖说别人什么的，还不如多花点时间提升一下自己的水平。你看看自己都菜成啥了。反正就这样吧时间也不早了。顺便一提，好想谈个恋爱，但是我配得上谁啊。就这样吧睡了。晚安，不知道会不会有人看到这篇无病呻吟的笔记

---

## [2026-01-07] - Adaptive Shooting Calibration

### Changed
- **Adaptive Velocity**: Recalibrated based on real field data
    - Near (24.4"): 700 TPS
    - Mid (77.4"): 950 TPS
    - Far (128.4"): 1420 TPS



