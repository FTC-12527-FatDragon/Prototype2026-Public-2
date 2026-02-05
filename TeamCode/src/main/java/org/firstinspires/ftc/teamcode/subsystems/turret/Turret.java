package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Note: DcMotor import kept for RunMode and ZeroPowerBehavior enums

/**
 * Subsystem for the Turret (Gimbal) mechanism.
 * Controls a single motor for turret rotation.
 * 
 * Encoder: REV Through Bore Encoder V2 mounted on motor shaft
 * - 8192 CPR (counts per revolution)
 * - Connected to motor encoder port, read via turretMotor.getCurrentPosition()
 * 
 * Gear ratio: Motor turns 116 times -> Turret turns 22 times
 * External gear ratio = 116/22 ≈ 5.2727
 * 
 * Coordinate system:
 * - 0° = turret facing forward
 * - Positive = clockwise (right)
 * - Negative = counterclockwise (left)
 */
public class Turret extends SubsystemBase {
    // Turret motor (also used for encoder reading)
    public final DcMotorEx turretMotor;
    
    // Position PIDF Controller (for both soft lock and hard lock)
    // F term used for static friction compensation
    private final PIDFController positionPIDF;
    
    // Current power being applied (for manual mode)
    private double targetPower = 0;
    
    // Lock mode state
    public enum LockMode {
        MANUAL,     // No lock, manual power control
        SOFT_LOCK,  // Lock to 0° forward, chassis handles aiming
        HARD_LOCK   // Track goal: tx when visible, odometry when not
    }
    private LockMode lockMode = LockMode.MANUAL;
    
    // Position hold state (for MANUAL mode brake)
    private boolean holdingPosition = false;
    private double holdAngleDeg = 0;
    private int holdTicks = 0;  // Hold position in encoder ticks (for PID)
    
    // Position control state
    private double targetAngleDeg = 0;
    
    // Hard lock state - target goal (set once at start based on alliance)
    private double goalX = 0;  // Target goal X coordinate
    private double goalY = 0;  // Target goal Y coordinate
    private boolean goalSet = false;  // Has goal been set?
    
    // Alliance enum for easy goal selection
    public enum Alliance {
        BLUE,
        RED
    }
    private Alliance currentAlliance = Alliance.BLUE;
    
    // Robot position (updated externally)
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;  // Radians
    
    // Flip/Unwind state: true when turret is flipping 180° because target is unreachable
    private boolean isUnwinding = false;
    // The target angle to flip to (calculated when flip starts)
    private double flipTargetAngle = 0;
    // Last valid target angle (before flip was triggered)
    private double lastValidTargetAngle = 0;
    
    // TX tracking for hard lock (when tag is visible)
    private double currentTx = 0;           // Current tx from vision
    private boolean hasValidTx = false;     // Is tx currently valid (tag visible)
    private int currentDetectedTagId = -1;  // Currently detected AprilTag ID
    
    // Target tag ID for TX tracking (based on alliance)
    // In SoloBlue: targetTagId = 20, only use TX tracking when seeing blue tag
    // In SoloRed: targetTagId = 24, only use TX tracking when seeing red tag
    // If we see the OTHER tag, we use inertial navigation to aim at OUR goal
    private int targetTagId = 20;  // Default to blue (ID 20)
    
    // TX low-pass filter for stability
    private double filteredTx = 0;
    private boolean txFilterInitialized = false;
    public static double txFilterAlpha = 0.3;  // Filter coefficient (0-1, lower = smoother)
    
    // Encoder offset for zeroing (relative to startup position)
    private int encoderOffset = 0;
    
    // Absolute home position (physical 0° forward, in raw encoder ticks)
    // This is the encoder value when turret is physically at 0°
    // Default 0 assumes first power-on with turret facing forward
    private int homeEncoderPosition = 0;
    
    // Startup encoder position (raw encoder value when program started)
    private int startupEncoderPosition = 0;
    
    // Calibration state
    private boolean isCalibrated = false;
    
    // Emergency disable flag (controlled by gamepad2)
    private boolean disabled = false;
    
    // Manual aim offset (controlled by D-pad, added to auto-aim target)
    private double manualAimOffset = 0.0;
    public static double manualOffsetSpeed = 1.5;  // Degrees per loop iteration

    /**
     * Constructor for Turret.
     * Initializes the turret motor (with built-in encoder).
     *
     * @param hardwareMap The hardware map from the OpMode.
     */
    public Turret(HardwareMap hardwareMap) {
        // Get motor as DcMotorEx for encoder access
        turretMotor = hardwareMap.get(DcMotorEx.class, TurretConstants.turretMotorName);
        
        // Configure motor
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set motor direction based on tuning (reverseMotor = true means REVERSE)
        turretMotor.setDirection(TurretConstants.reverseMotor ? 
            DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        
        // DON'T reset encoder - preserve absolute position across restarts
        // Only set to RUN_WITHOUT_ENCODER (we'll do our own PID)
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Record startup encoder position (for calculating relative movement)
        startupEncoderPosition = turretMotor.getCurrentPosition();
        // Set offset so getEncoderPosition() returns 0 at startup
        encoderOffset = startupEncoderPosition;
        
        // Mark as calibrated
        isCalibrated = true;
        
        // Initialize position PIDF controller (used for both soft and hard lock)
        // F term provides static friction compensation
        positionPIDF = new PIDFController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD,
                TurretConstants.kF
        );
    }

    /**
     * Sets the turret motor power with software limits.
     * NOTE: This is IGNORED during unwind - unwind has highest priority!
     * @param power Motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        // UNWIND HAS HIGHEST PRIORITY - ignore all manual control during unwind
        if (isUnwinding) {
            return;  // Silently ignore
        }
        
        // Apply software limits if calibrated
        // NOTE: positive power = angle decreases (left), negative power = angle increases (right)
        if (isCalibrated) {
            double currentAngle = getAngleDegrees();
            
            // Prevent moving past limits
            // Positive power → angle decreases → limit at minAngleDeg (left limit)
            if (currentAngle <= TurretConstants.minAngleDeg && power > 0) {
                power = 0;  // Don't go further left
            }
            // Negative power → angle increases → limit at maxAngleDeg (right limit)
            if (currentAngle >= TurretConstants.maxAngleDeg && power < 0) {
                power = 0;  // Don't go further right
            }
        }
        
        this.targetPower = Math.max(TurretConstants.minPower, 
                                    Math.min(TurretConstants.maxPower, power));
    }

    /**
     * Stops the turret motor.
     * NOTE: This is IGNORED during unwind - unwind has highest priority!
     */
    public void stop() {
        // UNWIND HAS HIGHEST PRIORITY
        if (isUnwinding) {
            return;
        }
        this.targetPower = 0;
    }
    
    /**
     * Enables position hold mode - uses PID to actively hold current position.
     * Call this when D-pad is released to maintain turret position.
     * Much stronger than passive BRAKE mode.
     */
    public void holdCurrentPosition() {
        if (isUnwinding) {
            return;
        }
        holdAngleDeg = getAngleDegrees();
        holdTicks = getEncoderPosition();  // Record ticks for PID (same as TurretMotorTuner)
        holdingPosition = true;
    }
    
    /**
     * Releases position hold and allows free movement.
     * Call this when D-pad is pressed to allow manual control.
     */
    public void releaseHold() {
        holdingPosition = false;
    }
    
    /**
     * Checks if turret is in position hold mode.
     * @return true if actively holding position
     */
    public boolean isHoldingPosition() {
        return holdingPosition;
    }

    /**
     * Rotates the turret left (negative power).
     * NOTE: This is IGNORED during unwind - unwind has highest priority!
     * @param speed Speed of rotation (0.0 to 1.0)
     */
    public void rotateLeft(double speed) {
        setPower(-Math.abs(speed));  // Will be ignored if unwinding
    }

    /**
     * Rotates the turret right (positive power).
     * @param speed Speed of rotation (0.0 to 1.0)
     */
    public void rotateRight(double speed) {
        setPower(Math.abs(speed));
    }

    // ==================== ENCODER METHODS ====================

    /**
     * Gets the raw encoder position (with offset applied).
     * Reads from motor's built-in encoder.
     * @return Encoder position in ticks.
     */
    public int getEncoderPosition() {
        return turretMotor.getCurrentPosition() - encoderOffset;
    }

    /**
     * Gets the current turret angle in degrees.
     * 0° = forward, positive = clockwise (right), negative = counterclockwise (left)
     * 
     * Calculation:
     * 1. Read motor encoder ticks
     * 2. Convert to motor rotations: ticks / ENCODER_CPR
     * 3. Convert to turret rotations: motor rotations / GEAR_RATIO (116/22)
     * 4. Convert to degrees and apply offset
     * 
     * Gear ratio: Motor turns 116 times -> Turret turns 22 times
     * So if motor turned 1 rotation, turret turned 22/116 ≈ 0.19 rotations
     * 
     * @return Turret angle in degrees.
     */
    public double getAngleDegrees() {
        int ticks = getEncoderPosition();
        // Motor shaft rotations (encoder counts / counts per motor revolution)
        double motorRotations = (double) ticks / TurretConstants.ENCODER_CPR;
        // Turret rotations (accounting for external gear ratio)
        // GEAR_RATIO = 116/22 means motor turns 5.27x for each turret turn
        double turretRotations = motorRotations / TurretConstants.GEAR_RATIO;
        // Convert to degrees and apply offset
        double rawAngle = turretRotations * 360.0;
        return TurretConstants.ANGLE_OFFSET - rawAngle;
    }

    /**
     * Gets the current turret angle in radians.
     * @return Turret angle in radians.
     */
    public double getAngleRadians() {
        return Math.toRadians(getAngleDegrees());
    }

    /**
     * Resets the encoder to zero (sets current position as 0°).
     * Call this when turret is facing forward.
     */
    public void resetEncoder() {
        encoderOffset = turretMotor.getCurrentPosition();
        isCalibrated = true;
    }
    
    /**
     * Sets the current physical position as the new "home" (0°) position.
     * Call this when turret is physically at forward position.
     * This updates homeEncoderPosition to the current raw encoder value.
     */
    public void setCurrentAsHome() {
        homeEncoderPosition = turretMotor.getCurrentPosition();
        // Also reset the working offset so current position = 0°
        encoderOffset = homeEncoderPosition;
        isCalibrated = true;
    }
    
    /**
     * Commands the turret to go to the home position (physical 0°).
     * Uses SOFT_LOCK mode to drive to home.
     */
    public void goToHome() {
        // Calculate target angle: home position relative to current offset
        // homeEncoderPosition is the raw encoder value at physical 0°
        // We need to convert this to degrees relative to current offset
        int ticksFromCurrentZero = homeEncoderPosition - encoderOffset;
        double homeDegrees = ticksToDegrees(ticksFromCurrentZero);
        enableSoftLock(homeDegrees);
    }
    
    /**
     * Gets the raw encoder position (without offset).
     * Useful for debugging absolute position.
     * @return Raw encoder ticks.
     */
    public int getRawEncoderPosition() {
        return turretMotor.getCurrentPosition();
    }
    
    /**
     * Gets the home encoder position.
     * @return Home position in raw encoder ticks.
     */
    public int getHomeEncoderPosition() {
        return homeEncoderPosition;
    }
    
    /**
     * Gets the startup encoder position.
     * @return Startup position in raw encoder ticks.
     */
    public int getStartupEncoderPosition() {
        return startupEncoderPosition;
    }
    
    /**
     * Converts encoder ticks to degrees.
     * @param ticks Encoder ticks.
     * @return Angle in degrees.
     */
    private double ticksToDegrees(int ticks) {
        // 1 turret rotation = ENCODER_CPR * GEAR_RATIO ticks
        double ticksPerTurretRotation = TurretConstants.ENCODER_CPR * TurretConstants.GEAR_RATIO;
        return (ticks / ticksPerTurretRotation) * 360.0;
    }
    
    /**
     * Converts degrees to encoder ticks.
     * Inverse of ticksToDegrees.
     * @param degrees Angle in degrees
     * @return Equivalent encoder ticks
     */
    private double degreesToTicks(double degrees) {
        // 1 turret rotation = ENCODER_CPR * GEAR_RATIO ticks
        double ticksPerTurretRotation = TurretConstants.ENCODER_CPR * TurretConstants.GEAR_RATIO;
        return (degrees / 360.0) * ticksPerTurretRotation;
    }

    /**
     * Checks if the turret has been calibrated (zero set).
     * @return True if calibrated.
     */
    public boolean isCalibrated() {
        return isCalibrated;
    }
    
    /**
     * Sets the emergency disable flag.
     * When disabled, the turret motor will be set to 0 power.
     * Controlled by gamepad2 (LB + RB to toggle).
     *
     * @param disabled True to disable turret.
     */
    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }
    
    /**
     * Checks if the turret is disabled.
     * @return True if disabled.
     */
    public boolean isDisabled() {
        return disabled;
    }
    
    /**
     * Toggles the disabled state.
     */
    public void toggleDisabled() {
        disabled = !disabled;
    }

    // ==================== GEOMETRY METHODS ====================

    /**
     * Calculate the distance from Limelight to chassis center based on current turret angle.
     * 
     * Geometry:
     * - Turret center is 47mm behind chassis center
     * - Limelight is 140.86521mm from turret center
     * - When turret angle = 0° (forward), Limelight is (140.86 - 47) = 93.86mm in front of chassis
     * 
     * @return Distance from Limelight to chassis center in mm.
     */
    public double getLimelightDistanceFromCenterMM() {
        double theta = getAngleRadians();
        
        double turretOffset = TurretConstants.turretOffsetMM;
        double limelightOffset = TurretConstants.limelightOffsetMM;
        
        // Limelight position in chassis coordinate system
        // +X = forward, +Y = right (clockwise positive)
        double Lx = -turretOffset + limelightOffset * Math.cos(theta);
        double Ly = limelightOffset * Math.sin(theta);
        
        return Math.sqrt(Lx * Lx + Ly * Ly);
    }

    /**
     * Calculate the distance from Limelight to chassis center in inches.
     * @return Distance in inches.
     */
    public double getLimelightDistanceFromCenterInches() {
        return getLimelightDistanceFromCenterMM() / 25.4;
    }

    /**
     * Gets the current target power.
     * @return Target power (-1.0 to 1.0)
     */
    public double getTargetPower() {
        return targetPower;
    }

    // ==================== LOCK MODE CONTROL ====================
    // NOTE: During UNWIND, mode changes are BLOCKED to ensure turret returns to 0° safely

    /**
     * Enables SOFT LOCK mode.
     * Turret will hold at the specified angle (or current angle if not specified).
     * Uses position PID to maintain angle.
     * 
     * NOTE: This is BLOCKED during unwind - unwind has highest priority!
     * 
     * @param angleDeg Target angle to lock at (0 = forward)
     */
    public void enableSoftLock(double angleDeg) {
        // UNWIND HAS HIGHEST PRIORITY - cannot change modes during unwind
        if (isUnwinding) {
            return;  // Silently ignore
        }
        
        // Clamp to limits
        angleDeg = Math.max(TurretConstants.minAngleDeg, 
                           Math.min(TurretConstants.maxAngleDeg, angleDeg));
        
        this.targetAngleDeg = angleDeg;
        this.lockMode = LockMode.SOFT_LOCK;
        
        // Update PID coefficients
        positionPIDF.setPIDF(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF);
        positionPIDF.reset();
    }

    /**
     * Enables SOFT LOCK at 0° (forward).
     * NOTE: This is BLOCKED during unwind!
     */
    public void enableSoftLock() {
        enableSoftLock(0);
    }

    /**
     * Sets the alliance for this match.
     * This determines which goal the turret will aim at in HARD_LOCK mode.
     * Also sets targetTagId for TX tracking:
     * - BLUE alliance: target tag ID 20, aim at blue basket
     * - RED alliance: target tag ID 24, aim at red basket
     * 
     * Call this once during initialization based on your TeleOp program.
     * 
     * @param alliance BLUE or RED
     */
    public void setAlliance(Alliance alliance) {
        this.currentAlliance = alliance;
        if (alliance == Alliance.BLUE) {
            this.goalX = TurretConstants.blueGoalX;
            this.goalY = TurretConstants.blueGoalY;
            this.targetTagId = 20;  // Only use TX tracking when seeing blue tag
        } else {
            this.goalX = TurretConstants.redGoalX;
            this.goalY = TurretConstants.redGoalY;
            this.targetTagId = 24;  // Only use TX tracking when seeing red tag
        }
        this.goalSet = true;
    }
    
    /**
     * Gets the current alliance setting.
     * @return Current alliance (BLUE or RED)
     */
    public Alliance getAlliance() {
        return currentAlliance;
    }
    
    /**
     * Enables HARD LOCK mode for Blue goal.
     * Also sets alliance to BLUE.
     * NOTE: This is BLOCKED during unwind!
     */
    public void enableHardLockBlue() {
        // UNWIND HAS HIGHEST PRIORITY - cannot change modes during unwind
        if (isUnwinding) {
            return;
        }
        setAlliance(Alliance.BLUE);
        this.lockMode = LockMode.HARD_LOCK;
        
        positionPIDF.setPIDF(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF);
        positionPIDF.reset();
    }

    /**
     * Enables HARD LOCK mode for Red goal.
     * Also sets alliance to RED.
     * NOTE: This is BLOCKED during unwind!
     */
    public void enableHardLockRed() {
        // UNWIND HAS HIGHEST PRIORITY
        if (isUnwinding) {
            return;
        }
        setAlliance(Alliance.RED);
        this.lockMode = LockMode.HARD_LOCK;
        
        positionPIDF.setPIDF(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF);
        positionPIDF.reset();
    }

    /**
     * Enables HARD LOCK mode using the pre-set alliance goal.
     * Make sure to call setAlliance() first!
     * NOTE: This is BLOCKED during unwind!
     */
    public void enableHardLock() {
        // UNWIND HAS HIGHEST PRIORITY
        if (isUnwinding) {
            return;
        }
        if (!goalSet) {
            // Default to blue if alliance not set
            setAlliance(Alliance.BLUE);
        }
        this.lockMode = LockMode.HARD_LOCK;
        
        positionPIDF.setPIDF(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF);
        positionPIDF.reset();
    }

    /**
     * Enables HARD LOCK mode for a custom goal position.
     * NOTE: This is BLOCKED during unwind!
     * 
     * @param targetX Goal X coordinate (inches)
     * @param targetY Goal Y coordinate (inches)
     */
    public void enableHardLock(double targetX, double targetY) {
        // UNWIND HAS HIGHEST PRIORITY
        if (isUnwinding) {
            return;
        }
        this.goalX = targetX;
        this.goalY = targetY;
        this.goalSet = true;
        this.lockMode = LockMode.HARD_LOCK;
        
        positionPIDF.setPIDF(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF);
        positionPIDF.reset();
    }

    /**
     * Disables all lock modes, returns to manual control.
     * NOTE: This is BLOCKED during unwind - unwind MUST complete!
     */
    public void disableLock() {
        // UNWIND HAS HIGHEST PRIORITY - cannot switch to manual during unwind
        if (isUnwinding) {
            return;
        }
        this.lockMode = LockMode.MANUAL;
        positionPIDF.reset();
    }

    /**
     * Gets the current lock mode.
     * @return Current LockMode.
     */
    public LockMode getLockMode() {
        return lockMode;
    }

    /**
     * Checks if turret is in any lock mode.
     * @return True if soft or hard locked.
     */
    public boolean isLocked() {
        return lockMode != LockMode.MANUAL;
    }
    
    // ==================== MANUAL AIM OFFSET ====================
    
    /**
     * Adjusts the manual aim offset (for D-pad control).
     * @param delta Amount to add to current offset (degrees).
     */
    public void adjustManualOffset(double delta) {
        manualAimOffset += delta;
        // Clamp to reasonable range
        manualAimOffset = Math.max(-90, Math.min(90, manualAimOffset));
    }
    
    /**
     * Resets the manual aim offset to zero.
     */
    public void resetManualOffset() {
        manualAimOffset = 0.0;
    }
    
    /**
     * Gets the current manual aim offset.
     * @return Manual offset in degrees.
     */
    public double getManualOffset() {
        return manualAimOffset;
    }
    
    /**
     * Directly sets motor power (for manual scan mode).
     * Bypasses PID and lock modes but respects software limits.
     * @param power Motor power (-1.0 to 1.0)
     */
    public void setMotorPower(double power) {
        if (disabled) {
            turretMotor.setPower(0);
            return;
        }
        
        // Apply software limits if calibrated
        if (isCalibrated) {
            double currentAngle = getAngleDegrees();
            // Positive power → angle decreases → limit at minAngleDeg (left limit)
            if (currentAngle <= TurretConstants.minAngleDeg && power > 0) {
                power = 0;
            }
            // Negative power → angle increases → limit at maxAngleDeg (right limit)
            if (currentAngle >= TurretConstants.maxAngleDeg && power < 0) {
                power = 0;
            }
        }
        
        turretMotor.setPower(power);
    }

    // ==================== ROBOT POSITION UPDATE ====================

    /**
     * Updates the robot's absolute position.
     * Call this every frame from TeleOp/Auto.
     * 
     * @param x Robot X position (inches)
     * @param y Robot Y position (inches)
     * @param heading Robot heading (radians, 0 = forward on field)
     */
    public void updateRobotPosition(double x, double y, double heading) {
        this.robotX = x;
        this.robotY = y;
        this.robotHeading = heading;
    }
    
    /**
     * Updates the tx value and detected tag ID for hard lock tracking.
     * Call this every frame when in HARD_LOCK mode.
     * 
     * TX tracking is ONLY used when:
     * 1. A tag is visible (valid = true)
     * 2. The detected tag ID matches our target tag (based on alliance)
     * 3. We are NOT in the middle of unwinding
     * 
     * If we see the OTHER alliance's tag, we use inertial navigation instead.
     * 
     * @param tx Horizontal offset from Limelight (degrees, positive = target is right)
     * @param valid True if tag is visible, false otherwise
     * @param tagId The detected AprilTag ID (-1 if none)
     */
    public void updateTx(double tx, boolean valid, int tagId) {
        this.currentDetectedTagId = tagId;
        this.hasValidTx = valid;
        
        if (valid) {
            // Apply low-pass filter to reduce jitter
            if (!txFilterInitialized) {
                filteredTx = tx;
                txFilterInitialized = true;
            } else {
                filteredTx = txFilterAlpha * tx + (1 - txFilterAlpha) * filteredTx;
            }
            this.currentTx = filteredTx;
        } else {
            // Reset filter when tag is lost
            txFilterInitialized = false;
            this.currentDetectedTagId = -1;
        }
    }
    
    /**
     * @deprecated Use updateTx(tx, valid, tagId) instead
     */
    @Deprecated
    public void updateTx(double tx, boolean valid) {
        updateTx(tx, valid, -1);
    }
    
    /**
     * Checks if turret currently has valid tx data (tag visible).
     * @return True if tag is visible and tx is valid.
     */
    public boolean hasValidTx() {
        return hasValidTx;
    }
    
    /**
     * Gets the current filtered tx value.
     * @return Filtered tx in degrees.
     */
    public double getFilteredTx() {
        return filteredTx;
    }
    
    /**
     * Gets the currently detected AprilTag ID.
     * @return Tag ID, or -1 if no tag detected.
     */
    public int getCurrentDetectedTagId() {
        return currentDetectedTagId;
    }
    
    /**
     * Gets the target tag ID for TX tracking (based on alliance).
     * @return Target tag ID (20 for blue, 24 for red).
     */
    public int getTargetTagId() {
        return targetTagId;
    }
    
    /**
     * Checks if TX tracking is currently active.
     * TX tracking is only active when:
     * 1. hasValidTx (tag visible)
     * 2. currentDetectedTagId == targetTagId (seeing our alliance's tag)
     * 3. NOT in unwind mode
     * 
     * @return True if TX tracking is active.
     */
    public boolean isTxTrackingActive() {
        // TX tracking only when seeing tag 24 (red goal)
        return hasValidTx && (currentDetectedTagId == 24) && !isUnwinding;
    }
    
    /**
     * Gets the current tracking mode as a string (for telemetry).
     * @return "TX_TRACKING", "INERTIAL", "FLIPPING", or "MANUAL/SOFT"
     */
    public String getTrackingModeString() {
        if (lockMode != LockMode.HARD_LOCK) {
            return lockMode.toString();
        }
        if (isUnwinding) {
            return String.format("FLIPPING→%.0f°", flipTargetAngle);
        }
        if (isTxTrackingActive()) {
            return "TX_TRACKING";
        }
        return "INERTIAL";
    }

    /**
     * Calculates the TX offset needed to aim at the basket instead of the AprilTag.
     * 
     * When using TX tracking, tx = 0 means the turret is pointing at the AprilTag.
     * But we want to aim at the basket center, which is offset from the tag.
     * This method calculates the angle difference so that:
     *   targetTx = txOffset  (instead of 0)
     * 
     * Pedro Pathing Coordinate System:
     * - 0° = +X (right), 90° = +Y (up), 180° = -X (left), 270° = -Y (down)
     * - Counter-clockwise is positive
     * 
     * @return TX offset in degrees (positive = basket is to the right of tag)
     */
    public double calculateTxOffsetToBasket() {
        // Get tag and goal positions based on alliance
        double tagX, tagY, goalX, goalY;
        if (currentAlliance == Alliance.BLUE) {
            tagX = TurretConstants.blueTagX;
            tagY = TurretConstants.blueTagY;
            goalX = TurretConstants.blueGoalX;
            goalY = TurretConstants.blueGoalY;
        } else {
            tagX = TurretConstants.redTagX;
            tagY = TurretConstants.redTagY;
            goalX = TurretConstants.redGoalX;
            goalY = TurretConstants.redGoalY;
        }
        
        // Calculate angles from robot position to tag and to goal
        double dxTag = tagX - robotX;
        double dyTag = tagY - robotY;
        double dxGoal = goalX - robotX;
        double dyGoal = goalY - robotY;
        
        // Angle to tag and goal in field coordinates (Pedro Pathing: 0° = +X)
        // atan2(dy, dx) gives angle from +X axis, CCW positive
        double angleToTag = Math.toDegrees(Math.atan2(dyTag, dxTag));
        double angleToGoal = Math.toDegrees(Math.atan2(dyGoal, dxGoal));
        
        // TX offset = how much we need to rotate from tag to basket
        // For turret (CW positive): negative of the field angle difference
        double txOffset = -(angleToGoal - angleToTag);
        
        // Normalize to [-180, 180]
        while (txOffset > 180) txOffset -= 360;
        while (txOffset < -180) txOffset += 360;
        
        return txOffset;
    }
    
    /**
     * Calculates the turret angle needed to aim at the goal.
     * 
     * Pedro Pathing Coordinate System:
     * - Field: 0° = +X (right), 90° = +Y (up), 180° = -X (left), 270° = -Y (down)
     * - Robot heading follows same convention (CCW positive)
     * - Turret: 0° = forward (robot heading), + = right (CW), - = left (CCW)
     * 
     * Math:
     * 1. Direction to goal (field frame) = atan2(dy, dx)
     *    - Standard math: angle from +X axis, CCW positive
     * 2. Turret angle (robot frame) = robotHeading - directionToGoal
     *    - Converts CCW positive (field) to CW positive (turret)
     * 
     * @return Required turret angle in degrees (0 = forward, + = right, - = left)
     */
    public double calculateAngleToGoal() {
        // Vector from robot to goal
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        
        // Direction to goal in field coordinates (radians)
        // Pedro Pathing: atan2(dy, dx) gives angle from +X axis, CCW positive
        double directionToGoal = Math.atan2(dy, dx);
        
        // Convert to robot-relative angle
        // Field (CCW +) to Turret (CW +): turretAngle = robotHeading - directionToGoal
        double turretAngleRad = robotHeading - directionToGoal;
        
        // Normalize to [-π, π]
        while (turretAngleRad > Math.PI) turretAngleRad -= 2 * Math.PI;
        while (turretAngleRad < -Math.PI) turretAngleRad += 2 * Math.PI;
        
        double targetAngle = Math.toDegrees(turretAngleRad);
        // targetAngle is now in [-180, +180]
        
        // ===== CHECK IF TARGET IS WITHIN PHYSICAL LIMITS =====
        // Physical limits: minAngleDeg to maxAngleDeg (e.g., -180 to +190)
        if (targetAngle >= TurretConstants.minAngleDeg && 
            targetAngle <= TurretConstants.maxAngleDeg) {
            // Target is directly reachable
            return targetAngle;
        }
        
        // Target is outside limits - need to clamp
        // This shouldn't happen often since limits are close to ±180
        if (targetAngle < TurretConstants.minAngleDeg) {
            return TurretConstants.minAngleDeg;
        } else {
            return TurretConstants.maxAngleDeg;
        }
    }

    /**
     * Gets the distance from robot to goal.
     * @return Distance in inches.
     */
    public double getDistanceToGoal() {
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Checks if turret is on target (hard lock mode).
     * @return True if turret angle is within tolerance of calculated angle.
     */
    public boolean isOnTarget() {
        if (lockMode != LockMode.HARD_LOCK) return false;
        double calculatedAngle = calculateAngleToGoal();
        double error = Math.abs(getAngleDegrees() - calculatedAngle);
        return error <= TurretConstants.positionTolerance;
    }

    /**
     * Checks if turret is at the soft lock position.
     * @return True if at target angle.
     */
    public boolean isAtLockPosition() {
        if (lockMode != LockMode.SOFT_LOCK) return false;
        double error = Math.abs(getAngleDegrees() - targetAngleDeg);
        return error <= TurretConstants.positionTolerance;
    }
    
    /**
     * Checks if the target angle is reachable by the turret.
     * @param targetAngle Target angle in degrees
     * @return True if within turret limits
     */
    public boolean isTargetReachable(double targetAngle) {
        return targetAngle >= TurretConstants.minAngleDeg && 
               targetAngle <= TurretConstants.maxAngleDeg;
    }
    
    /**
     * Checks if the turret needs to flip 180° (target is behind robot, unreachable directly).
     * When |target angle| >= 185°, turret flips to the opposite side instead of hitting limits.
     * 
     * @param rawTargetAngle The raw calculated target angle (before clamping)
     * @return True if turret should flip 180°
     */
    public boolean shouldUnwind(double rawTargetAngle) {
        return Math.abs(rawTargetAngle) >= TurretConstants.unwindThreshold;
    }
    
    /**
     * Calculates the flip target angle (180° in opposite direction).
     * E.g., +185° → -175° (185 - 360 = -175)
     * E.g., -190° → +170° (-190 + 360 = +170)
     * @param rawTargetAngle The original target angle
     * @return The flipped target angle
     */
    public double calculateFlipTarget(double rawTargetAngle) {
        if (rawTargetAngle >= TurretConstants.unwindThreshold) {
            // Target is on the right side (positive), flip to left
            return rawTargetAngle - 360.0;
        } else if (rawTargetAngle <= -TurretConstants.unwindThreshold) {
            // Target is on the left side (negative), flip to right
            return rawTargetAngle + 360.0;
        }
        return rawTargetAngle;  // No flip needed
    }
    
    /**
     * Checks if the turret is currently flipping/unwinding (cannot be interrupted).
     * @return True if flipping
     */
    public boolean isUnwinding() {
        return isUnwinding;
    }
    
    /**
     * EMERGENCY: Force cancel unwind and return to normal operation.
     * WARNING: Only use this in emergencies! Normally unwind should complete naturally.
     * This bypasses all safety checks.
     */
    public void forceStopUnwind() {
        isUnwinding = false;
        lastValidTargetAngle = 0;
    }

    /**
     * Gets the current target angle.
     * For soft lock: returns the set target.
     * For hard lock: returns the calculated angle to goal.
     * 
     * @return Target angle in degrees.
     */
    public double getTargetAngle() {
        if (lockMode == LockMode.HARD_LOCK) {
            return calculateAngleToGoal();
        }
        return targetAngleDeg;
    }

    /**
     * Gets the current position error.
     * @return Error in degrees (target - current).
     */
    public double getPositionError() {
        return getTargetAngle() - getAngleDegrees();
    }

    // ==================== LEGACY METHODS (for compatibility) ====================

    /**
     * Sets the target angle for position control.
     * @deprecated Use enableSoftLock(angleDeg) instead.
     */
    public void setTargetAngle(double angleDeg) {
        enableSoftLock(angleDeg);
    }

    /**
     * Disables position control.
     * @deprecated Use disableLock() instead.
     */
    public void disablePositionControl() {
        disableLock();
    }

    /**
     * Checks if position control is enabled.
     * @deprecated Use getLockMode() instead.
     */
    public boolean isPositionControlEnabled() {
        return lockMode == LockMode.SOFT_LOCK;
    }

    /**
     * Periodic update method.
     * Executes the appropriate control logic based on lock mode.
     */
    @Override
    public void periodic() {
        // Emergency disable check - highest priority
        if (disabled) {
            turretMotor.setPower(0);
            return;
        }
        
        double outputPower = 0;
        double currentAngle = getAngleDegrees();
        
        switch (lockMode) {
            case SOFT_LOCK:
                // Position hold mode - maintain fixed target angle
                if (isCalibrated) {
                    double error = targetAngleDeg - currentAngle;
                    
                    // Check if current position is out of bounds
                    boolean softLockOutOfBounds = currentAngle < TurretConstants.minAngleDeg || 
                                                   currentAngle > TurretConstants.maxAngleDeg;
                    
                    // Normalize error for shortest VALID path (unless out of bounds)
                    if (!softLockOutOfBounds) {
                        double normalizedError = error;
                        while (normalizedError > 180) normalizedError -= 360;
                        while (normalizedError < -180) normalizedError += 360;
                        
                        // Check if the shorter path stays within physical limits
                        double shortestPathTarget = currentAngle + normalizedError;
                        if (shortestPathTarget >= TurretConstants.minAngleDeg && 
                            shortestPathTarget <= TurretConstants.maxAngleDeg) {
                            error = normalizedError;
                        }
                        // Otherwise use original longer path
                    }
                    
                    // Recalculate effective target based on (possibly normalized) error
                    double effectiveTarget = currentAngle + error;
                    
                    if (Math.abs(error) <= TurretConstants.positionTolerance) {
                        outputPower = 0;
                    } else {
                        // PID controller using DEGREE parameters (disable built-in F, we handle it manually)
                        positionPIDF.setPIDF(TurretConstants.kP_deg, TurretConstants.kI_deg, TurretConstants.kD_deg, 0);
                        double pidPower = positionPIDF.calculate(currentAngle, effectiveTarget);
                        
                        // IMPORTANT: Negate PID output!
                        // PID gives positive when target > current (error > 0, target on right)
                        // But positive motor power DECREASES angle (turns left)
                        // So we negate: error > 0 → need negative power to go right
                        pidPower = -pidPower;
                        
                        // Add F manually with direction awareness (static friction compensation)
                        // error > 0 means target is to the right, need negative power
                        // error < 0 means target is to the left, need positive power
                        double feedforward = (error > 0) ? -TurretConstants.kF_deg : TurretConstants.kF_deg;
                        outputPower = pidPower + feedforward;
                        
                        // Clamp to max output
                        outputPower = Math.max(-TurretConstants.maxOutputPower, 
                                               Math.min(TurretConstants.maxOutputPower, outputPower));
                    }
                }
                break;
                
            case HARD_LOCK:
                // Goal tracking mode with FLIP protection and alliance-specific TX tracking:
                //
                // TX Tracking Rules:
                // - ONLY use TX tracking when we see OUR alliance's tag (targetTagId)
                // - If we see the OTHER alliance's tag, use inertial navigation
                // - Example: SoloBlue (targetTagId=20) sees red tag (24) → use inertial to aim at blue basket
                //
                // Flip Rules (when |target| >= 185°):
                // - Turret flips 180° to the opposite side (e.g., +185° → -175°)
                // - During flip: CANNOT be interrupted by chassis movement or TX tracking
                // - Flip completes when turret reaches the flip target (within tolerance)
                //
                if (isCalibrated) {
                    double rawDesiredAngle;
                    double desiredAngle;
                    
                    // ===== CHECK IF WE'RE IN FLIP MODE (CANNOT be interrupted!) =====
                    if (isUnwinding) {
                        // During flip, turret MUST complete the 180° rotation
                        // This cannot be interrupted by TX tracking or chassis movement
                        
                        // Check if flip is complete (turret near flip target)
                        double flipCompleteTolerance = TurretConstants.positionTolerance * 2;
                        double errorToFlipTarget = flipTargetAngle - currentAngle;
                        
                        if (Math.abs(errorToFlipTarget) <= flipCompleteTolerance) {
                            // Flip complete, can resume normal operation
                            isUnwinding = false;
                        } else {
                            // Still flipping - drive to flip target
                            desiredAngle = flipTargetAngle;
                            
                            // Clamp to physical limits (safety)
                            desiredAngle = Math.max(TurretConstants.minAngleDeg, 
                                                   Math.min(TurretConstants.maxAngleDeg, desiredAngle));
                            double error = desiredAngle - currentAngle;
                            
                            if (Math.abs(error) <= TurretConstants.positionTolerance) {
                                outputPower = 0;
                            } else {
                                // PID + direction-aware F (using DEGREE parameters)
                                positionPIDF.setPIDF(TurretConstants.kP_deg, TurretConstants.kI_deg, TurretConstants.kD_deg, 0);
                                double pidPower = positionPIDF.calculate(currentAngle, desiredAngle);
                                
                                // IMPORTANT: Negate PID output!
                                pidPower = -pidPower;
                                
                                // error > 0 means target is to the right, need negative power
                                double feedforward = (error > 0) ? -TurretConstants.kF_deg : TurretConstants.kF_deg;
                                outputPower = pidPower + feedforward;
                                outputPower = Math.max(-TurretConstants.maxOutputPower, 
                                                       Math.min(TurretConstants.maxOutputPower, outputPower));
                            }
                            break;  // Exit HARD_LOCK case, don't run normal logic
                        }
                    }
                    
                    // ===== NORMAL MODE: Choose between TX tracking and inertial =====
                    // TX tracking ONLY when seeing tag 24 (red goal)
                    boolean canUseTxTracking = hasValidTx && (currentDetectedTagId == 24);
                    
                    if (canUseTxTracking) {
                        // ===== TX TRACKING MODE =====
                        // Use current angle + tx correction to center the BASKET (not the tag!)
                        // txOffset compensates for the difference between tag and basket positions
                        // When currentTx = txOffset, the turret is pointing at the basket center
                        double txOffset = calculateTxOffsetToBasket();
                        rawDesiredAngle = currentAngle + (currentTx - txOffset);
                    } else {
                        // ===== INERTIAL NAVIGATION MODE =====
                        // Calculate desired turret angle to aim at goal using robot position
                        // Used when:
                        // - No tag visible
                        // - Seeing the OTHER alliance's tag (we still know our position!)
                        rawDesiredAngle = calculateAngleToGoal();
                    }
                    
                    // ===== APPLY MANUAL AIM OFFSET (D-pad control) =====
                    rawDesiredAngle += manualAimOffset;
                    
                    // ===== CHECK IF CURRENT POSITION IS OUT OF BOUNDS =====
                    // If turret is already beyond physical limits, flip to safe side
                    boolean currentOutOfBounds = currentAngle < TurretConstants.minAngleDeg || 
                                                  currentAngle > TurretConstants.maxAngleDeg;
                    
                    // ===== FLIP CHECK (when |target| >= 185°) =====
                    // If target is beyond the threshold (behind robot),
                    // flip 180° to the opposite side instead of hitting limits
                    if (shouldUnwind(rawDesiredAngle) || currentOutOfBounds) {
                        // Target is unreachable - need to flip 180°
                        if (!isUnwinding) {
                            isUnwinding = true;
                            lastValidTargetAngle = rawDesiredAngle;
                            // Calculate flip target: e.g., +185° → -175°
                            flipTargetAngle = calculateFlipTarget(rawDesiredAngle);
                        }
                        desiredAngle = flipTargetAngle;  // Flip to opposite side
                    } else {
                        // Target is reachable and current position is valid
                        desiredAngle = rawDesiredAngle;
                    }
                    
                    // Clamp to turret limits (safety)
                    desiredAngle = Math.max(TurretConstants.minAngleDeg, 
                                           Math.min(TurretConstants.maxAngleDeg, desiredAngle));
                    
                    double error = desiredAngle - currentAngle;
                    
                    // ===== NORMALIZE ERROR FOR SHORTEST VALID PATH =====
                    // Only normalize if NOT out of bounds - if out of bounds,
                    // we want to take the DIRECT path back, not the "shortest" one
                    // which might go further out of bounds.
                    if (!currentOutOfBounds) {
                        // Normal case: wrap error to find shorter path
                        double normalizedError = error;
                        while (normalizedError > 180) normalizedError -= 360;
                        while (normalizedError < -180) normalizedError += 360;
                        
                        // Check if the shorter path stays within physical limits
                        double shortestPathTarget = currentAngle + normalizedError;
                        if (shortestPathTarget >= TurretConstants.minAngleDeg && 
                            shortestPathTarget <= TurretConstants.maxAngleDeg) {
                            // Shortest path is valid, use it
                            error = normalizedError;
                        }
                        // Otherwise, use original (longer) path which should stay in bounds
                    }
                    // If out of bounds, use direct error to get back to safe zone
                    
                    // IMPORTANT: Recalculate desiredAngle based on (possibly normalized) error
                    // This ensures PIDF.calculate() uses the correct error internally
                    double effectiveDesiredAngle = currentAngle + error;
                    
                    if (Math.abs(error) <= TurretConstants.positionTolerance) {
                        outputPower = 0;  // On target
                    } else {
                        // PID + direction-aware F (using DEGREE parameters)
                        positionPIDF.setPIDF(TurretConstants.kP_deg, TurretConstants.kI_deg, TurretConstants.kD_deg, 0);
                        double pidPower = positionPIDF.calculate(currentAngle, effectiveDesiredAngle);
                        
                        // IMPORTANT: Negate PID output!
                        // PID gives positive when target > current (error > 0, target on right)
                        // But positive motor power DECREASES angle (turns left)
                        // So we negate: error > 0 → need negative power to go right
                        pidPower = -pidPower;
                        
                        // error > 0 means target is to the right, need negative power
                        // error < 0 means target is to the left, need positive power
                        double feedforward = (error > 0) ? -TurretConstants.kF_deg : TurretConstants.kF_deg;
                        outputPower = pidPower + feedforward;
                        
                        // Clamp to max output
                        outputPower = Math.max(-TurretConstants.maxOutputPower, 
                                               Math.min(TurretConstants.maxOutputPower, outputPower));
                    }
                }
                break;
                
            case MANUAL:
            default:
                // Manual power mode with optional position hold
                if (holdingPosition) {
                    // Use TICKS for PID calculation (SAME AS TurretMotorTuner!)
                    // kP/kI/kD/kF are tuned for ticks
                    int currentTicks = getEncoderPosition();
                    double errorTicks = holdTicks - currentTicks;
                    
                    // Tolerance check (100 ticks ≈ 0.83°, same as TurretMotorTuner)
                    double toleranceTicks = 100;
                    if (Math.abs(errorTicks) <= toleranceTicks) {
                        outputPower = 0;  // Within tolerance, no correction needed
                    } else {
                        // PID controller using TICKS (exactly like TurretMotorTuner)
                        positionPIDF.setPIDF(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, 0);
                        double pidPower = positionPIDF.calculate(currentTicks, holdTicks);
                        
                        // Add F manually with direction awareness (static friction compensation)
                        // F pushes in the direction of error (same as TurretMotorTuner)
                        double feedforward = (errorTicks > 0) ? TurretConstants.kF : -TurretConstants.kF;
                        outputPower = pidPower + feedforward;
                        
                        // Clamp to max output
                        outputPower = Math.max(-TurretConstants.maxOutputPower, 
                                               Math.min(TurretConstants.maxOutputPower, outputPower));
                    }
                } else {
                    // Direct power control
                    outputPower = targetPower;
                }
                break;
        }
        
        // Apply software limits
        // NOTE: positive power = angle decreases (left), negative power = angle increases (right)
        if (isCalibrated) {
            // Positive power → angle decreases → limit at minAngleDeg (left limit)
            if (currentAngle <= TurretConstants.minAngleDeg && outputPower > 0) {
                outputPower = 0;
            }
            // Negative power → angle increases → limit at maxAngleDeg (right limit)
            if (currentAngle >= TurretConstants.maxAngleDeg && outputPower < 0) {
                outputPower = 0;
            }
        }
        
        turretMotor.setPower(outputPower);
    }
}



// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
