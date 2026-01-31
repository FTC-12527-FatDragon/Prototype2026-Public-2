package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for the Turret (Gimbal) mechanism.
 * Controls a single motor for turret rotation.
 * Uses REV Through Bore Encoder V2 for precise angle measurement.
 * 
 * Encoder specs:
 * - REV Through Bore Encoder V2 (REV-11-3174)
 * - 8192 counts per revolution (incremental mode)
 * - Accuracy: ±0.5°
 * 
 * Coordinate system:
 * - 0° = turret facing forward
 * - Positive = clockwise (right)
 * - Negative = counterclockwise (left)
 */
public class Turret extends SubsystemBase {
    public final DcMotor turretMotor;
    
    // External encoder (REV Through Bore Encoder V2)
    // Connected to a motor encoder port on the hub
    private final DcMotorEx encoderPort;
    
    // Position PIDF Controller (for both soft lock and hard lock)
    // F term used for static friction compensation
    private final PIDFController positionPIDF;
    
    // Current power being applied (for manual mode)
    private double targetPower = 0;
    
    // Lock mode state
    public enum LockMode {
        MANUAL,     // No lock, manual power control
        SOFT_LOCK,  // Lock to encoder position (0° forward)
        HARD_LOCK   // Lock to goal position (calculated from absolute position)
    }
    private LockMode lockMode = LockMode.MANUAL;
    
    // Position control state
    private double targetAngleDeg = 0;
    
    // Hard lock state
    private double goalX = 0;  // Target goal X coordinate
    private double goalY = 0;  // Target goal Y coordinate
    
    // Robot position (updated externally)
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;  // Radians
    
    // Encoder offset for zeroing
    private int encoderOffset = 0;
    
    // Calibration state
    private boolean isCalibrated = false;

    /**
     * Constructor for Turret.
     * Initializes the turret motor and encoder.
     *
     * @param hardwareMap The hardware map from the OpMode.
     */
    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, TurretConstants.turretMotorName);
        
        // REV Through Bore Encoder connected to an encoder port
        // Note: In FTC, external encoders are read through motor encoder ports
        encoderPort = hardwareMap.get(DcMotorEx.class, TurretConstants.turretEncoderName);
        
        // Configure motor
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Configure encoder port (just for reading, not driving)
        encoderPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
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
     * @param power Motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        // Apply software limits if calibrated
        if (isCalibrated) {
            double currentAngle = getAngleDegrees();
            
            // Prevent moving past limits
            if (currentAngle <= TurretConstants.minAngleDeg && power < 0) {
                power = 0;  // Don't go further left
            }
            if (currentAngle >= TurretConstants.maxAngleDeg && power > 0) {
                power = 0;  // Don't go further right
            }
        }
        
        this.targetPower = Math.max(TurretConstants.minPower, 
                                    Math.min(TurretConstants.maxPower, power));
    }

    /**
     * Stops the turret motor.
     */
    public void stop() {
        this.targetPower = 0;
    }

    /**
     * Rotates the turret left (negative power).
     * @param speed Speed of rotation (0.0 to 1.0)
     */
    public void rotateLeft(double speed) {
        setPower(-Math.abs(speed));
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
     * @return Encoder position in ticks.
     */
    public int getEncoderPosition() {
        return encoderPort.getCurrentPosition() - encoderOffset;
    }

    /**
     * Gets the current turret angle in degrees.
     * 0° = forward, positive = clockwise (right), negative = counterclockwise (left)
     * 
     * Calculation accounts for gear ratio and angle offset:
     * - Encoder measures motor/input shaft rotations
     * - Turret rotations = Encoder rotations / GEAR_RATIO
     * - ANGLE_OFFSET corrects for encoder zero position
     * 
     * Example: If encoder reads 90° when turret faces forward (0°):
     * - Set ANGLE_OFFSET = 90
     * - Turret angle = OFFSET - encoder = 90 - 90 = 0° (forward) ✓
     * - When encoder = 0°: turret = 90 - 0 = 90° (right) ✓
     * 
     * @return Turret angle in degrees.
     */
    public double getAngleDegrees() {
        int ticks = getEncoderPosition();
        // Encoder rotations (how many times the encoder has rotated)
        double encoderRotations = (double) ticks / TurretConstants.ENCODER_CPR;
        // Turret rotations (accounting for gear ratio)
        double turretRotations = encoderRotations / TurretConstants.GEAR_RATIO;
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
        encoderOffset = encoderPort.getCurrentPosition();
        isCalibrated = true;
    }

    /**
     * Checks if the turret has been calibrated (zero set).
     * @return True if calibrated.
     */
    public boolean isCalibrated() {
        return isCalibrated;
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

    /**
     * Enables SOFT LOCK mode.
     * Turret will hold at the specified angle (or current angle if not specified).
     * Uses position PID to maintain angle.
     * 
     * @param angleDeg Target angle to lock at (0 = forward)
     */
    public void enableSoftLock(double angleDeg) {
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
     */
    public void enableSoftLock() {
        enableSoftLock(0);
    }

    /**
     * Enables HARD LOCK mode for Blue goal.
     * Turret will continuously calculate the angle to aim at the Blue goal
     * based on robot's absolute position.
     */
    public void enableHardLockBlue() {
        this.goalX = TurretConstants.blueGoalX;
        this.goalY = TurretConstants.blueGoalY;
        this.lockMode = LockMode.HARD_LOCK;
        
        positionPIDF.setPIDF(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF);
        positionPIDF.reset();
    }

    /**
     * Enables HARD LOCK mode for Red goal.
     * Turret will continuously calculate the angle to aim at the Red goal
     * based on robot's absolute position.
     */
    public void enableHardLockRed() {
        this.goalX = TurretConstants.redGoalX;
        this.goalY = TurretConstants.redGoalY;
        this.lockMode = LockMode.HARD_LOCK;
        
        positionPIDF.setPIDF(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF);
        positionPIDF.reset();
    }

    /**
     * Enables HARD LOCK mode for a custom goal position.
     * 
     * @param targetX Goal X coordinate (inches)
     * @param targetY Goal Y coordinate (inches)
     */
    public void enableHardLock(double targetX, double targetY) {
        this.goalX = targetX;
        this.goalY = targetY;
        this.lockMode = LockMode.HARD_LOCK;
        
        positionPIDF.setPIDF(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF);
        positionPIDF.reset();
    }

    /**
     * Disables all lock modes, returns to manual control.
     */
    public void disableLock() {
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
     * Calculates the turret angle needed to aim at the goal.
     * 
     * Coordinate System:
     * - Field: X = right, Y = forward (standard FTC)
     * - Robot heading = 0 means facing Y+ direction (forward on field)
     * - Turret: 0° = forward, + = clockwise (right), - = counterclockwise (left)
     * 
     * Math:
     * 1. Direction to goal (field frame) = atan2(dx, dy)  // Note: atan2(x,y) not atan2(y,x)!
     *    - This gives angle from Y+ axis (forward), where +angle = clockwise (right)
     * 2. Turret angle (robot frame) = direction - robotHeading
     * 
     * @return Required turret angle in degrees (0 = forward, + = right, - = left)
     */
    public double calculateAngleToGoal() {
        // Vector from robot to goal
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        
        // Direction to goal in field coordinates (radians)
        // Using atan2(dx, dy) because:
        // - heading = 0 is Y+ direction (forward)
        // - atan2(dx, dy) gives angle from Y+ axis, with + = clockwise (right)
        double directionToGoal = Math.atan2(dx, dy);
        
        // Convert to robot-relative angle
        // Turret angle = field direction - robot heading
        double turretAngleRad = directionToGoal - robotHeading;
        
        // Normalize to [-π, π]
        while (turretAngleRad > Math.PI) turretAngleRad -= 2 * Math.PI;
        while (turretAngleRad < -Math.PI) turretAngleRad += 2 * Math.PI;
        
        return Math.toDegrees(turretAngleRad);
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
        double outputPower = 0;
        double currentAngle = getAngleDegrees();
        
        switch (lockMode) {
            case SOFT_LOCK:
                // Position hold mode - maintain fixed target angle
                if (isCalibrated) {
                    double error = targetAngleDeg - currentAngle;
                    
                    if (Math.abs(error) <= TurretConstants.positionTolerance) {
                        outputPower = 0;
                    } else {
                        // PIDF controller: F term provides feedforward
                        outputPower = positionPIDF.calculate(currentAngle, targetAngleDeg);
                        
                        // Min power to overcome static friction
                        if (Math.abs(outputPower) < TurretConstants.minOutputPower && Math.abs(error) > 0) {
                            outputPower = Math.signum(error) * TurretConstants.minOutputPower;
                        }
                        
                        // Clamp to max output
                        outputPower = Math.max(-TurretConstants.maxOutputPower, 
                                               Math.min(TurretConstants.maxOutputPower, outputPower));
                    }
                }
                break;
                
            case HARD_LOCK:
                // Goal tracking mode - calculate angle to goal based on robot position
                if (isCalibrated) {
                    // Calculate desired turret angle to aim at goal
                    double desiredAngle = calculateAngleToGoal();
                    
                    // Clamp to turret limits
                    desiredAngle = Math.max(TurretConstants.minAngleDeg, 
                                           Math.min(TurretConstants.maxAngleDeg, desiredAngle));
                    
                    double error = desiredAngle - currentAngle;
                    
                    if (Math.abs(error) <= TurretConstants.positionTolerance) {
                        outputPower = 0;  // On target
                    } else {
                        // PIDF controller: F term provides feedforward
                        outputPower = positionPIDF.calculate(currentAngle, desiredAngle);
                        
                        // Min power to overcome static friction
                        if (Math.abs(outputPower) < TurretConstants.minOutputPower && Math.abs(error) > 0) {
                            outputPower = Math.signum(error) * TurretConstants.minOutputPower;
                        }
                        
                        // Clamp to max output
                        outputPower = Math.max(-TurretConstants.maxOutputPower, 
                                               Math.min(TurretConstants.maxOutputPower, outputPower));
                    }
                }
                break;
                
            case MANUAL:
            default:
                // Manual power mode
                outputPower = targetPower;
                break;
        }
        
        // Apply software limits
        if (isCalibrated) {
            if (currentAngle <= TurretConstants.minAngleDeg && outputPower < 0) {
                outputPower = 0;
            }
            if (currentAngle >= TurretConstants.maxAngleDeg && outputPower > 0) {
                outputPower = 0;
            }
        }
        
        turretMotor.setPower(outputPower);
    }
}


