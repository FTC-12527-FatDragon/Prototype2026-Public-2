# Changelog

All notable changes to the Prototype2026-Public-2 project will be documented in this file.

---

## [2026-02-04] - Shooter Pseudo Closed-loop Optimization & Auto-Aim Fixes

### Changed
- **Shooter Pseudo Closed-loop** (`Shooter.java`) - Major optimization:
    - Added **deadband** (±15000 TPS) for stability zone
    - Added **feedforward correction** (×1.3) to compensate motor efficiency loss
    - Added **mode-specific parameters**:
        - MID mode: approach power 0.7, brake power -0.3 (gentler)
        - SLOW/FAST: approach power 0.85, brake power -0.5
    - New logic: `error > 50000 → 1.0`, `error > 15000 → approach`, `else → feedforward`

- **Firing Boost** (`Shooter.java`) - New feature:
    - 0.5 seconds after transit opens, add +8% power boost
    - Locked power mode: bypasses pseudo closed-loop during boost
    - Compensates for velocity drop when ball exits
    - Resets on state change or STOP

- **Chassis Auto-Aim PID** (`MecanumDrivePinpoint.java`):
    - Fixed PID output direction (negated)
    - Fixed `getHeading()` method (was missing)

- **Near Auto Starting Angles** (all Near auto files):
    - BlueNearAuto: 90° → **143.5°**
    - BlueNearInfinite: 90° → **143.5°**
    - RedNearAuto: 90° → **36.5°** (mirrored)
    - RedNearAuto2: 90° → **36.5°** (mirrored)
    - RedNearInfinite: 90° → **36.5°** (mirrored)

### Added
- **Auto Dashboard Visualization** (`AutoCommandBase.java`):
    - Robot position displayed on FTC Dashboard Field view
    - Green circle + red heading arrow
    - Coordinates converted from corner-origin (0-144) to center-origin (-72 to 72)

- **ShooterAutoTuner** - Extended test durations:
    - Feedforward test: 12s → **60s** (1 minute)
    - Step Response: 5s → **30s**
    - Verification: 6s → **30s**
    - Steady state analysis uses last 5 seconds of data

### Fixed
- **Shooter `isShooterAtSetPoint()`** - Added 0.3s stability requirement before allowing fire
- **Shooter state tracking** - Added `lastShooterState` for boost reset on mode change

---

## [2026-02-03] - ChassisAlignTuner PID Tuned

### Fixed
- **ChassisAlignTuner** - Fixed PID direction (was going wrong way)
    - Changed `error = -currentHeading` → `error = currentHeading`
    - Now robot correctly resists when pushed

### Tuned (ChassisAlignTuner)
- **kP** = 0.03 ✅
- **kI** = 0.0
- **kD** = 0.003 ✅
- **kF** = 0.0 (not needed)
- **maxPower** = 1.0
- **tolerance** = 2.0°

---

## [2026-02-03] - Complete Documentation Update

### Added (GUIDE.md)
- **Robot Container** section - explains `Robot.java` and TeleOp vs Auto usage
- **SoloEmergency** section - full documentation for emergency backup OpMode
- **WaitForTurretCommand** section - auto command documentation
- **RedNearAuto2** - added to available auto programs list
- **AutoConstants** section - auto program coordinates reference
- **TeleOpConstants** section - trigger thresholds reference
- **Test Programs** - Added Tuning, PathTunerOpMode, ColorSensorTest descriptions
- **DashboardUtil** section - Dashboard visualization utility
- **FunctionalButton** section - custom button class for complex triggers
- **Units** section - unit conversion utilities
- **DriverControls** section - gamepad binding reference

### Updated (GUIDE.md)
- Table of Contents - added all new sections
- Test OpModes table - added 3 more entries
- Constants Files table - added AutoConstants, TeleOpConstants, TurretConstants

---

## [2026-02-03] - ChassisAlignTuner Rewrite

### Changed
- **ChassisAlignTuner** - Complete rewrite using MecanumDrivePinpoint
    - Based on PedroPathing's HeadingTuner design
    - Uses `MecanumDrivePinpoint` for motor control and Pinpoint for heading
    - No Follower (avoids conflicts with TeleOp MecanumDrive)
    - PIDF parameters converted from PedroPathing (radians → degrees):
        - `kP = 0.014` (was 0.8 in radians)
        - `kI = 0.0`
        - `kD = 0.00035` (was 0.02 in radians)
        - `kF = 0.03` (direction-aware static friction)
    - Default `enabled = true` (starts immediately)
    - Robot locks to starting heading, user turns robot by hand to test PIDF resistance
    - Removed all gamepad controls (pure Dashboard tuning)
    - Dashboard shows: currentHeading, error, turnPower, onTarget, PIDF values

### Usage
1. Run "Chassis Heading Tuner"
2. Robot automatically tries to maintain starting heading
3. Turn robot by hand → PIDF resists and returns to 0°
4. Adjust kP/kD/kF on Dashboard until response is smooth

---

## [2026-02-03] - Bug Fixes & Code Cleanup

### Fixed
- **Auto Path Files** - Point → Pose Fix
    - All 7 auto files were using non-existent `Point` class
    - Changed to use `Pose` directly (matches Pedro Pathing library API)
    - Files fixed: `BlueFarAuto`, `RedFarAuto`, `BlueNearAuto`, `RedNearAuto`, `BlueNearInfinite`, `RedNearInfinite`, `RedNearAuto2`

- **Shooter Velocity Reading** (`Shooter.java`)
    - Changed from left motor to right motor for velocity sensing
    - Velocity convention: all positive values now (matching original Prototype2026-Public)
    - Power application: `leftShooter.setPower(power)`, `rightShooter.setPower(-power)`
    - Velocity reading: `return -rightShooter.getVelocity()` (returns positive)

- **Intake Motor Direction** (`Intake.java`)
    - Changed from `REVERSE` to `FORWARD`
    - Fixed reversed intake direction issue

- **Turret Software Limits** (`Turret.java`)
    - Fixed D-Pad Right not working issue
    - Corrected power clamping logic for `reverseMotor = true`
    - Positive power now correctly decreases angle, negative increases angle

- **Turret Software Limit Range** (`TurretConstants.java`)
    - Changed from ±95° to **±190°** (total 380° range)
    - User requested larger range

### Changed
- **Shooter Velocity Setpoints** (`ShooterConstants.java`)
    - `slowVelocity`: 700 → **950** TPS
    - `midVelocity`: 950 → **1500** TPS (user adjusted from 1290)
    - `fastVelocity`: 1420 → **2100** TPS (user adjusted from 1930)

- **Shooter Braking Logic** (`ShooterConstants.java`)
    - `motorBrakeThreshold`: 200 → **100** TPS (brake when 100 TPS over target)
    - `motorBrakePower`: 0.3 → **0.5** (stronger reverse braking)

### Added
- **ShooterPIDTuner** (`tests/ShooterPIDTuner.java`)
    - New test program for Shooter velocity PIDF tuning
    - Pure Dashboard control (no gamepad)
    - Implements full PIDF loop for velocity control
    - Parameters: `targetVelocity`, `enabled`, `kP`, `kI`, `kD`, `kF`, `tolerance`

### Removed
- **Emergency Disable Controls** (`SoloBlue.java`, `SoloRed.java`)
    - Removed LT+LB (Intake disable), RT+RB (Shooter disable), LB+RB (Turret disable)
    - Gamepad2 now only has: **Right Stick Button = Set Home**

### Current Gamepad2 Functions (SoloBlue/SoloRed)
| Button | Function |
|--------|----------|
| **Right Stick Button** | Set current turret position as home |
| All other buttons | Unused |

---

## [2026-02-03] - Parallel Turret Movement Optimization

### Added
- **WaitForTurretCommand** (`commands/autocommands/WaitForTurretCommand.java`)
    - Waits for turret to reach target angle using `turret.isAtTarget()`
    - Uses same tolerance as Turret PIDF control (`TurretConstants.positionTolerance`)
    - Configurable timeout (default 1000ms)
    - Finishes when turret at target OR timeout reached

### Changed
- **All 6 Auto Programs** - Parallel Turret Movement
    - Turret starts moving BEFORE path to shoot position begins
    - Robot movement and turret rotation happen simultaneously (saves ~300ms per shot)
    - Shooting only starts after `WaitForTurretCommand` confirms turret is at target

- **Far Auto Series** - Added Turret Control
    - `BlueFarAuto`: `TURRET_SHOOT_ANGLE_DEG = +21.3°` (aim right)
    - `RedFarAuto`: `TURRET_SHOOT_ANGLE_DEG = -21.3°` (aim left)

- **Shoot Sequence (All Autos)**
    - Old: `Arrive → Set turret → Wait 300ms → Shoot` (sequential)
    - New: `Set turret → Drive (turret moving) → Wait for turret → Shoot` (parallel)

### Technical Details
```java
// New parallel approach
new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
new AutoDriveCommand(follower, pathToShoot),  // Turret moves during drive
new WaitForTurretCommand(turret, TURRET_TIMEOUT_MS),  // Precise wait
// ... shoot sequence
```

### Turret Angles Summary
| Auto Program | Angle | Direction |
|--------------|-------|-----------|
| BlueFarAuto | +21.3° | Right |
| RedFarAuto | -21.3° | Left |
| BlueNearAuto | +43.3° | Right |
| RedNearAuto | -43.3° | Left |
| BlueNearInfinite | +43.3° | Right |
| RedNearInfinite | -43.3° | Left |

---

## [2026-02-02] - Auto Turret Control & Intake Enhancement

### Added
- **AutoCommandBase** - Turret Subsystem Integration
    - Added `protected Turret turret;` member variable
    - Turret initialized in `initialize()` method
    - All auto programs now have access to turret control

- **Auto Intake Full Power**
    - Intake runs at full power (`setFullPower(true)`) throughout entire auto
    - Starts immediately after `waitForStart()`
    - Stops in `onAutoStopped()` cleanup

- **Auto Cleanup (`onAutoStopped()`)**
    - Intake: `stopIntake()` + `setFullPower(false)`
    - Shooter: `setShooterState(STOP)`
    - Turret: `enableSoftLock(0)` (return to forward)

### Changed
- **Near Auto Series** - Turret Aiming at SHOOT_POSE
    - All 4 Near programs now control turret during shooting
    - Blue versions: `TURRET_SHOOT_ANGLE_DEG = +43.3°` (aim right)
    - Red versions: `TURRET_SHOOT_ANGLE_DEG = -43.3°` (aim left)
    - New `TURRET_SETTLE_MS = 300ms` for turret stabilization

- **shootCommand() Sequence** (Near autos)
    1. Set turret to `TURRET_SHOOT_ANGLE_DEG`
    2. Wait `TURRET_SETTLE_MS` (300ms)
    3. Start shooter (SLOW mode)
    4. Wait `SHOOT_WAIT_MS` (1500ms)
    5. Stop shooter
    6. Return turret to 0° (forward)

### Dashboard Parameters (Near Auto)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `TURRET_SHOOT_ANGLE_DEG` | ±43.3° | Turret angle for shooting |
| `TURRET_SETTLE_MS` | 300ms | Turret stabilization time |
| `SHOOT_WAIT_MS` | 1500ms | Shooting duration |
| `INTAKE_WAIT_MS` | 500ms | Intake collection time |

---

## [2026-02-02] - Autonomous Path Programs

### Added
- **BlueFarAuto** (`opmodes/autos/BlueFarAuto.java`)
    - Blue Alliance Far side continuous intake/shoot auto
    - Loops intake → shoot cycles until auto ends
    - Key positions: START (54.97, 9.10), INTAKE (9.60, 8.13), SHOOT (54.97, 9.10)

- **RedFarAuto** (`opmodes/autos/RedFarAuto.java`)
    - Red Alliance Far side (mirrored from BlueFarAuto at x=72)
    - Heading mirrored: 180° → 0°

- **BlueNearAuto** (`opmodes/autos/BlueNearAuto.java`)
    - Blue Alliance Near side sample collection auto
    - 9-path sequence collecting samples from multiple positions
    - 3 shoot cycles with sample collection

- **RedNearAuto** (`opmodes/autos/RedNearAuto.java`)
    - Red Alliance Near side (mirrored from BlueNearAuto at x=72)

- **BlueNearInfinite** (`opmodes/autos/BlueNearInfinite.java`)
    - Extended Blue Near auto with Sample 2 collection repeated 3 times
    - New 6-path intro + BlueNear Path 5+ continuation
    - Angled intake at 145° for Sample 2

- **RedNearInfinite** (`opmodes/autos/RedNearInfinite.java`)
    - Red Alliance version (mirrored, 35° angle for Sample 2)

### Technical Details
- All paths use **Pedro Pathing** library (`com.pedropathing:ftc:2.0.4`)
- Path types: `BezierLine` (straight), `BezierCurve` (with control points)
- `setLinearHeadingInterpolation()` for smooth heading changes
- `PathChain` rebuilt dynamically for repeated cycles (only usable once)
- `AutoDriveCommand` wraps follower for Command-based integration

### Coordinate Mirroring (x=72)
| Position | Blue x | Red x | Formula |
|----------|--------|-------|---------|
| Any | x | 144-x | Mirror |
| Heading 180° | 180° | 0° | Mirror |
| Heading 145° | 145° | 35° | 180-145=35 |

---

## [2026-02-02] - Turret Home Function & Open Loop Control

### Added
- **Turret Absolute Position Memory** (`Turret.java`)
    - Encoder no longer force-reset on startup (preserves absolute position)
    - `homeEncoderPosition`: Stores the physical 0° encoder value
    - `startupEncoderPosition`: Records encoder value when program starts
    - New methods:
        - `goToHome()`: Returns turret to physical 0° position
        - `setCurrentAsHome()`: Sets current position as new 0°
        - `getRawEncoderPosition()`: Gets raw encoder value
        - `getHomeEncoderPosition()`: Gets home encoder value

### Changed
- **Solo.java** - Turret Control Overhaul
    - D-Pad control changed from **closed-loop to open-loop**:
        - D-Pad Left: `setPower(0.5)` (counter-clockwise)
        - D-Pad Right: `setPower(-0.5)` (clockwise)
        - Release: `setPower(0)` (stop)
    - Added home control buttons:
        - **Y button**: Go to home position (physical 0°)
        - **B button**: Set current position as new home
    - Removed closed-loop variables (`turretTargetTicks`, `TICKS_PER_DEGREE`)

- **TurretConstants.java**
    - `positionTolerance`: Changed from `100.0` (ticks) to `2.0` (degrees)
    - Fixed unit mismatch bug (SOFT_LOCK uses degrees, not ticks)

- **Turret.java**
    - Added `isCalibrated = true` in constructor (was defaulting to false)
    - This bug caused SOFT_LOCK PID to never execute

- **ChassisAlignTuner.java** - Upgraded to PIDF
    - Added `kF` parameter (default 0.05) for static friction compensation
    - Changed from `PIDController` to `PIDFController`
    - Fixed `calculate()` parameters: `(0, error)` → `(currentHeading, targetHeading)`
    - Now consistent with Turret PIDF logic

### Fixed
- **Intake.java**: Fixed broken comment at end of file (split across two lines)
- **Turret SOFT_LOCK**: PID now actually executes (was blocked by `isCalibrated = false`)
- **Turret tolerance**: Fixed unit mismatch (was comparing degrees to 100 "ticks")

---

## [2026-02-02] - Turret PIDF Tuning & Test Programs

### Added
- **TurretMotorTuner** (`tests/TurretMotorTuner.java`)
    - Dedicated test program for turret PIDF tuning
    - Dashboard-only control (no manual gamepad input)
    - Direction-aware kF (static friction compensation in both directions)
    - `reverseMotor` option via `setDirection()` 
    - Gamepad A to reset encoder
    
- **ChassisAlignTuner** (`tests/ChassisAlignTuner.java`)
    - Heading PID tuner for chassis rotation
    - Uses Pinpoint odometry for heading measurement
    - Auto-locks turret during test (`turretMotor.setBrake()`)
    - Dashboard control + D-Pad ±15° adjustment

### Tuned Parameters (TurretConstants.java)
| Parameter | Value | Notes |
|-----------|-------|-------|
| `kP` | 0.0004 | Position control |
| `kI` | 0.0 | Not used |
| `kD` | 0.0000185 | Damping |
| `kF` | 0.058 | Static friction (direction-aware) |
| `tolerance` | 100 ticks | ~0.83° |
| `reverseMotor` | true | Motor direction REVERSED |

### Changed
- **Turret.java**: Updated PID logic to match TurretMotorTuner
    - Motor direction set via `TurretConstants.reverseMotor`
    - kF now direction-aware: `feedforward = (error > 0) ? kF : -kF`
    - Removed `minOutputPower` friction compensation (kF handles this now)
    - Applies to both SOFT_LOCK and HARD_LOCK modes

### Direction-Aware kF Logic
```java
// Disable built-in F in PIDF controller
positionPIDF.setPIDF(kP, kI, kD, 0);
double pidPower = positionPIDF.calculate(current, target);

// Add F manually with correct direction
double feedforward = (error > 0) ? kF : -kF;
output = pidPower + feedforward;
```

---

## [2026-02-01] - DashTuner Encoder Safety Protection

### Added
- **Encoder Stuck Detection** (`DashTuner.java`)
    - Monitors encoder readings during position closed-loop
    - If motor outputs power >0.05 but encoder doesn't change for 0.5s → **EMERGENCY STOP**
    - Motor is permanently disabled until reset
    - Prevents motor/mechanism damage from disconnected or broken encoders

- **Dashboard Configurable Parameters**
    | Parameter | Default | Description |
    |-----------|---------|-------------|
    | `ENCODER_TIMEOUT` | 0.5s | Time without encoder change before triggering |
    | `ENCODER_MIN_CHANGE` | 5 ticks | Minimum change to consider "moving" |
    | `resetEncoderError` | false | Set to true to clear error and re-enable motors |

### Error Display
- FTC Dashboard: `!!! ENCODER ERROR [motor#] !!!`
- Driver Station telemetry: Detailed error with last/current position

### Recovery
1. Dashboard: Set `resetEncoderError = true` (auto-resets to false)
2. Or restart OpMode

---

## [2026-02-01] - Pseudo Closed-loop Motor Braking

### Added
- **Motor Braking** (`Shooter.java`, `ShooterConstants.java`)
    - When overspeed exceeds 200 TPS, reverse motor to brake
    - No physical brake needed - uses motor reverse torque
    - New constants:
        - `motorBrakeThreshold = 200` TPS
        - `motorBrakePower = 0.3` (reverse power)

### Control Logic
| State | Condition | Action |
|-------|-----------|--------|
| Too slow | `currentVel > targetVel` | Full power (1.0) |
| Too fast > 200 TPS | `currentVel < targetVel - 200` | Reverse brake (-0.3) |
| Near target | Otherwise | Feedforward maintain |

---

## [2026-02-01] - TX Tracking Basket Offset & Code Cleanup

### Added
- **AprilTag Coordinates** (`TurretConstants.java`)
    - `blueTagX/Y = (17, 131)` - Blue tag (ID 20) position
    - `redTagX/Y = (127, 131)` - Red tag (ID 24) position
    - Tags are in front of baskets, not at basket center!

- **TX Offset Compensation** (`Turret.java`)
    - New method: `calculateTxOffsetToBasket()`
    - TX tracking now aims at **basket center** instead of AprilTag
    - Both TX tracking and inertial navigation aim at the same point

### Changed
- **Terminology**: "Bang-Bang" → "Pseudo Closed-loop"
    - Updated in `Shooter.java` and `ShooterConstants.java`
    - More accurate description of the control method

- **All Chinese comments translated to English**
    - `SoloEmergency.java` - 20+ comments
    - `TransitConstants.java` - 4 comments
    - `AutoCommandBase.java` - 1 error message
    - `Units.java` - 2 easter egg comments

### Coordinate Summary
| Target | Blue | Red |
|--------|------|-----|
| **AprilTag** | (17, 131) | (127, 131) |
| **Basket** | (4, 140) | (140, 140) |

---

## [2026-02-01] - Gamepad2 Emergency Disable System

### Added
- **Emergency Disable Feature** for all major subsystems
    - `Intake.java`: Added `disabled` flag, `setDisabled()`, `isDisabled()`, `toggleDisabled()`
    - `Shooter.java`: Added `disabled` flag, `setDisabled()`, `isDisabled()`, `toggleDisabled()`
    - `Turret.java`: Added `disabled` flag, `setDisabled()`, `isDisabled()`, `toggleDisabled()`
    - When disabled: `periodic()` immediately sets motor power to 0 and returns
    
- **Gamepad2 Controls** in `Solo.java`, `SoloBlue.java`, `SoloRed.java`
    - Secondary gamepad (`gamepadEx2`) for emergency control
    - Edge detection to prevent continuous toggling
    - Telemetry display of disable status

### Control Mapping (Gamepad2)
| Combo | Target | Effect |
|-------|--------|--------|
| **LT + LB** | Intake | Toggle disable (motor → 0) |
| **RT + RB** | Shooter | Toggle disable (motors → 0) |
| **LB + RB** | Turret | Toggle disable (motor → 0) |

### Usage
1. Press combo once → subsystem disabled (will not respond to any commands)
2. Press combo again → subsystem re-enabled
3. Check telemetry: `=== EMERGENCY DISABLE (GP2) ===` shows OK/DISABLED

---

## [2026-02-01] - TeleOp Restructure & OpMode Split

### Renamed
- **TeleOp.java** → **Solo.java**
    - Class name and file name changed
    - Display name remains "Solo"

### Added
- **SoloBlue.java** - Alliance-specific TeleOp for Blue
    - Sets `turret.setAlliance(Turret.Alliance.BLUE)`
    - Hard Lock aims at blue basket (4, 140)
    - TX tracking only when seeing tag 20
    
- **SoloRed.java** - Alliance-specific TeleOp for Red
    - Sets `turret.setAlliance(Turret.Alliance.RED)`
    - Hard Lock aims at red basket (140, 140)
    - TX tracking only when seeing tag 24

### Changed
- **Chassis Auto-Aim Trigger** (`TeleOpDriveCommand.java`)
    - Now triggered by **A button ONLY**
    - Shoot buttons (LB/RB/RT) no longer trigger auto-aim
    - Only works in SOFT_LOCK mode (disabled in HARD_LOCK)
    
- **Solo.java** - Simplified for chassis auto-aim
    - Turret always in SOFT_LOCK (fixed at 0°)
    - Chassis handles auto-aim via A button
    - Removed all hard-lock related code
    
### OpMode Usage Guide
| Match Type | OpMode |
|------------|--------|
| Practice/Testing | Solo |
| Blue Alliance | SoloBlue |
| Red Alliance | SoloRed |

---

## [2026-02-01] - Turret Auto-Aim Logic Overhaul

### Added
- **Alliance-Specific TX Tracking** (`Turret.java`)
    - TX tracking only activates when seeing YOUR alliance's AprilTag
    - SoloBlue: Only use TX when seeing tag 20 (blue goal)
    - SoloRed: Only use TX when seeing tag 24 (red goal)
    - Seeing OTHER alliance's tag → use inertial navigation to aim at YOUR goal
    - `targetTagId` variable set automatically by `setAlliance()`

- **Unwind Priority Protection** (`Turret.java`)
    - Unwind (returning to 0°) now has HIGHEST priority
    - Cannot be interrupted by any external control
    - All control methods blocked during unwind:
        - `setPower()`, `stop()`, `rotateLeft/Right()`
        - `enableSoftLock()`, `enableHardLock*()`, `disableLock()`
    - Unwind completes only when turret reaches near 0°
    - Force inertial navigation during unwind (no TX tracking)

- **New Debug Methods** (`Turret.java`)
    - `getCurrentDetectedTagId()`: Get currently detected tag ID
    - `getTargetTagId()`: Get target tag ID (20 or 24)
    - `isTxTrackingActive()`: Check if TX tracking is active
    - `getTrackingModeString()`: Returns "TX_TRACKING", "INERTIAL", or "UNWINDING"
    - `forceStopUnwind()`: Emergency cancel unwind (use with caution!)

### Changed
- **updateTx() Method** (`Turret.java`)
    - Now requires tagId parameter: `updateTx(tx, valid, tagId)`
    - Old method deprecated but still works (assumes tagId = -1)

---

## [2026-02-01] - Turret Gear Ratio & REV Encoder

### Changed
- **Turret Encoder Configuration** (`TurretConstants.java`, `Turret.java`)
    - Uses REV Through Bore Encoder V2 mounted on motor shaft
    - `ENCODER_CPR = 8192` (REV Through Bore Encoder V2 incremental mode)
    - Encoder wired to motor encoder port, read via `turretMotor.getCurrentPosition()`
    
- **Gear Ratio Configuration** (`TurretConstants.java`)
    - `GEAR_RATIO = 116.0 / 22.0 ≈ 5.2727`
    - Motor shaft turns 116 times → Turret turns 22 times
    
### Removed
- `turretEncoderName` constant (encoder shares motor port)
- Separate `encoderPort` variable in `Turret.java`

### Hardware Setup
- REV Through Bore Encoder V2 (REV-11-3174) on motor shaft
- Encoder signal wired to same motor port on Control Hub
- Motor configured as DcMotorEx to access encoder

---

## [2026-02-01] - Turret-Compensated Absolute Position

### Added
- **Turret-Compensated Vision Position** (`MecanumDrivePinpoint.java`)
    - New method: `updateAbsolutePositionFromVisionWithTurret(vision, turretAngleRad)`
    - Calculates correct chassis center position when Limelight is mounted on turret
    - Accounts for turret geometry:
        - Turret center: 47mm behind chassis center (~1.85")
        - Limelight: 140.86mm from turret center (~5.55")
    - Transforms Limelight offset from chassis coordinates to field coordinates

### Changed
- **Absolute Position Update Logic** (commented code in `TeleOpDriveCommand.java`, `TeleOp.java`)
    - Now uses `updateAbsolutePositionFromVisionWithTurret()` with turret angle parameter
    - Falls back to 0° if turret not calibrated
    
### Deprecated
- `updateAbsolutePositionFromVision(vision)` - Legacy method without turret compensation
    - Still works but assumes Limelight at chassis center (incorrect for turret setup)

### Fixed
- **DriverControls.java**: Removed calls to deleted brake methods in adaptive shooting code
    - `cancelAutoBrakeCycle()` and `startAutoBrakeCycle()` replaced with comments

---

## [2026-02-01] - TeleOp Simplification & Auto-Aim Disable

### Disabled (Commented Out)
- **Chassis Auto-Aim** (`TeleOpDriveCommand.java`)
    - Absolute position update (Vision + Odometry fusion)
    - Soft Lock / Hard Lock aware auto-aim
    - D-Pad fine rotation
    
- **Turret Control** (`TeleOp.java`, `DriverControls.java`)
    - `ENABLE_TURRET = false` in `Robot.java`
    - Turret update logic in `run()` loop
    - Turret telemetry display
    - Right Stick Button lock mode toggle
    
- **Vision/Limelight** (`TeleOp.java`)
    - `ENABLE_VISION = false` in `Robot.java`
    - All Vision-related telemetry
    - Absolute position display
    
- **Adaptive Shooting** (`DriverControls.java`)
    - X button (Blue adaptive fire)
    - B button (Red adaptive fire)

### Removed
- **Brake Servo**: All brake-related code deleted from codebase
    - `ShooterConstants.java`: Removed brake constants
    - `Shooter.java`: Removed brake servo, methods, and auto-brake logic
    - `DriverControls.java`: Removed D-Pad Down brake binding

### Changed
- **Shooter Servo Positions** (`ShooterConstants.java`)
    - STOP/MID: 0.5 (center position)
    - SLOW (near): 0.04 (low angle)
    - FAST (far): 1.0 (high angle)
    
- **Transit/Limit Servo Positions** (`TransitConstants.java`)
    - `transitUpPos`: 0.36 (firing)
    - `transitDownPos`: 0.62 (loading)
    - `limitOpenPos`: 0.6
    - `limitClosedPos`: 0.3
    
- **Intake Full Power** (`IntakeConstants.java`)
    - LT trigger now runs intake at 1.0 power (was 0.65)
    
- **TeleOp Driving** (`TeleOpDriveCommand.java`)
    - Simplified to manual-only field-centric drive
    - Same logic as `DriveOnlyTeleOp.java`

### Note
All disabled code is preserved in comments and can be re-enabled when Vision/Turret hardware is ready.

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



