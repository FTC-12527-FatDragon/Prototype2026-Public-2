package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.strafingBalance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.utils.Util;

import com.arcrobotics.ftclib.controller.PIDController;

/**
 * Subsystem for the Mecanum Drive train using GoBilda Pinpoint Odometry for localization.
 * This class handles motor control, odometry updates, and movement logic.
 */
@Config
public class MecanumDrivePinpoint extends SubsystemBase {
    // Hardware devices
    public final DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private final GoBildaPinpointDriver pinpoint;
    
    // Offset for heading to allow resetting heading without resetting full odometry (for field-centric drive)
    private double yawOffset;

    // Flag to indicate if gamepad control is active (used for braking logic)
    public boolean isGamepadOn;

    // Last recorded pose
    Pose2D lastPose;
    
    // Flag to track if vision calibration has been performed
    private boolean hasVisionCalibrated = false;
    
    // ==================== ABSOLUTE FIELD COORDINATES ====================
    // Fused localization: Vision when available, odometry dead-reckoning otherwise
    private double absoluteX = 0;           // Absolute X in field (inches)
    private double absoluteY = 0;           // Absolute Y in field (inches)
    private double absoluteHeading = 0;     // Absolute heading in field (radians)
    
    // Last odometry pose for dead-reckoning delta calculation
    private Pose2D lastOdoPose = null;
    
    // Flag to track if we have a valid absolute position
    private boolean hasAbsolutePosition = false;
    
    // ==================== AUTO-AIM ====================
    private final PIDController alignPID;
    
    // Auto-aim PID constants (tunable via Dashboard)
    // Distance-adaptive: actual kP = kP_alignH * (1 - distanceFactor * kP_distanceScale)
    public static double kP_alignH = 0.025;      // Base P gain for auto-aim
    public static double kI_alignH = 0;          // I gain for auto-aim
    public static double kD_alignH = 0.008;      // D gain for auto-aim (damping)
    public static double kP_near = 0.012;        // P gain at close range (weaker to prevent overshoot)
    public static double kP_far = 0.03;          // P gain at far range (stronger for precision)
    
    // Deadband with hysteresis to prevent oscillation at boundary
    public static double alignDeadbandNear = 3.0;      // Entry deadband for close shots (degrees)
    public static double alignDeadbandFar = 0.5;       // Entry deadband for far shots (degrees)
    public static double alignHysteresis = 1.5;        // Exit deadband = entry + hysteresis
    
    // Auto-aim offset constants
    public static double farDistanceThreshold = 94;    // Distance threshold for offset (inches)
    public static double farOffsetDegrees = 2.0;       // Offset in degrees for far shots
    
    // Low-pass filter for tx smoothing (reduces jitter from Limelight noise)
    public static double txFilterAlpha = 0.3;          // Filter coefficient (0-1, lower = smoother)
    private double filteredTx = 0;
    private boolean txFilterInitialized = false;
    
    // Current deadband (set based on distance)
    private double currentDeadband = alignDeadbandNear;
    
    // Hysteresis state: true = inside deadband (aligned), false = outside (aligning)
    private boolean isAligned = false;
    
    // Cached offset to prevent jitter (Mode 1: tx-based)
    private double currentOffset = 0;
    private boolean offsetLocked = false;
    
    // Locked target goal for heading-based alignment (Mode 2: no tag visible)
    private boolean headingTargetLocked = false;
    private double lockedTargetGoalX = 0;
    private double lockedTargetGoalY = 0;
    
    // Last aligned tag for search mode
    private int lastAlignedTagId = -1;
    private boolean hasLastAlignedTag = false;

    /**
     * Constructor for MecanumDrivePinpoint.
     * Initializes motors and the Pinpoint driver.
     *
     * @param hardwareMap The hardware map from the OpMode.
     */
    public MecanumDrivePinpoint(final HardwareMap hardwareMap) {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        
        // Initialize Pinpoint Driver (device name "od")
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "od");
        isGamepadOn = false;

        // Set Zero Power Behavior to BRAKE for all motors to resist movement when 0 power is applied
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pinpoint Configuration
        // Set offsets for the odometry pods relative to the robot center.
        // Constants are stored in inches in DriveConstants, but Pinpoint expects millimeters.
        // We pass values in inches and specify DistanceUnit.INCH.
        pinpoint.setOffsets(DriveConstants.xPoseDW, DriveConstants.yPoseDW, DistanceUnit.INCH);
        
        // Set the encoder resolution for the pods (GoBilda 4-Bar Pods)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        
        // Set encoder directions based on installation
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.REVERSED, 
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        
        // Reset Position and IMU on initialization
        pinpoint.resetPosAndIMU();

        // Motor Directions
        // Updated based on user feedback: Forward/Strafe were reversed, Turn was correct.
        // Inverting all motors corrects Forward/Strafe while maintaining relative Turn direction.
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        lastPose = new Pose2D(DriveConstants.distanceUnit, 0, 0, DriveConstants.angleUnit, 0);
        
        // Initialize auto-aim PID controller
        alignPID = new PIDController(kP_alignH, kI_alignH, kD_alignH);
        
        // Initialize absolute field coordinates to (0, 0, 0)
        absoluteX = 0;
        absoluteY = 0;
        absoluteHeading = 0;
        hasAbsolutePosition = false;
        lastOdoPose = null;
    }

    /**
     * Stops the robot by setting all motor powers to 0.
     */
    public void stop() {
        moveRobot(0, 0, 0);
    }

    /**
     * Resets the robot's heading to a specified value.
     * This updates the yawOffset rather than resetting the Pinpoint hardware to preserve X/Y coordinates.
     *
     * @param heading The new heading in Radians.
     */
    public void reset(double heading) {
        // Update Pinpoint data before reading
        pinpoint.update();
        // Calculate offset so that (current_hardware_heading - offset) = new_desired_heading
        // offset = current_hardware_heading - new_desired_heading
        // Actually, logic in moveRobotFieldRelative is: botHeading = pinpoint.getHeading() - yawOffset;
        // So if we want botHeading = heading:
        // heading = pinpoint.getHeading() - yawOffset
        // yawOffset = pinpoint.getHeading() - heading
        
        // However, standard "reset" usually means "set current heading to 0".
        // If parameter 'heading' is the desired new heading:
        yawOffset = pinpoint.getHeading(DriveConstants.angleUnit) - heading;
    }
    
    /**
     * Resets the robot's heading to 0 (convenience method).
     */
    public void resetHeading() {
        reset(0);
    }

    /**
     * Sets the gamepad active flag.
     * Used to determine if the robot should actively brake (stop) when no input is detected.
     *
     * @param on True if gamepad input is active, false otherwise.
     */
    public void setGamepad(boolean on) {
        isGamepadOn = on;
    }

    /**
     * Moves the robot relative to the field (Field Centric Drive).
     *
     * @param forward Forward movement (Y axis in field coordinates).
     * @param fun Strafe movement (X axis in field coordinates).
     * @param turn Rotation speed.
     */
    public void moveRobotFieldRelative(double forward, double fun, double turn) {
        // Apply deadband only to joystick inputs (forward/strafe), not turn
        // This allows auto-aim small turn values to work for fine adjustments
        if (Math.abs(forward) < DriveConstants.deadband) forward = 0;
        if (Math.abs(fun) < DriveConstants.deadband) fun = 0;
        // Note: No deadband on turn to allow auto-aim fine adjustments
        
        // If all inputs are zero, stop
        if (forward == 0 && fun == 0 && turn == 0) {
            stop();
            return;
        }

        pinpoint.update(); // Ensure data is fresh
        
        // Get current robot heading from Pinpoint, adjusting for offset
        double botHeading = pinpoint.getHeading(DriveConstants.angleUnit) - yawOffset;
        
        // Rotate the movement vector by the negative of the robot's heading to transform field-relative input to robot-relative
        // If the robot moves with the heading (Robot Centric behavior in Field Centric mode), 
        // it usually means the heading compensation is inverted. 
        // Trying Positive botHeading here to reverse the compensation direction.
        double rotX = fun * Math.cos(botHeading) - forward * Math.sin(botHeading);
        double rotY = fun * Math.sin(botHeading) + forward * Math.cos(botHeading);

        // Apply strafing balance correction to counteract mechanical inefficiencies in mecanum strafing
        rotX = -rotX * strafingBalance;

        // Normalize motor powers so that the maximum power is 1.0 if any calculated power exceeds 1.0
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        // Set motor powers
        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    /**
     * Moves the robot relative to itself (Robot Centric Drive).
     *
     * @param forward Forward speed (positive is forward).
     * @param fun Strafe speed (positive is right, but notice the sign flip logic below).
     * @param turn Rotation speed (positive is turn right).
     */
    public void moveRobot(double forward, double fun, double turn) {
        // Deadband check to stop drift
        if (Math.abs(forward) < DriveConstants.deadband &&
            Math.abs(fun) < DriveConstants.deadband &&
            Math.abs(turn) < DriveConstants.deadband) {
            // Manually set motors to 0 to ensure braking
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            return;
        }

        // Apply strafing balance and sign correction
        // Note: 'fun' input usually assumes positive is right, but standard mecanum math might expect different sign convention
        // Here we invert 'fun' and apply balance
        double rotX = -fun * strafingBalance; 
        double rotY = forward;

        // Normalize motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        // Set motor powers
        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    /**
     * Gets the current pose of the robot from the Pinpoint driver.
     *
     * @return The current Pose2D (x, y, heading).
     */
    public Pose2D getPose() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        return new Pose2D(
            DistanceUnit.INCH, 
            pose.getX(DistanceUnit.INCH), 
            pose.getY(DistanceUnit.INCH), 
            DriveConstants.angleUnit, 
            pose.getHeading(DriveConstants.angleUnit)
        );
    }

    /**
     * Gets the current yaw offset.
     * @return Yaw offset in radians.
     */
    public double getYawOffset() {
        return yawOffset;
    }

    /**
     * Checks if the robot's heading is within a small tolerance of a setpoint.
     *
     * @param headingSetPoint The target heading.
     * @return True if at setpoint, false otherwise.
     */
    public boolean isHeadingAtSetPoint(double headingSetPoint) {
        return Util.epsilonEqual(pinpoint.getHeading(DriveConstants.angleUnit), headingSetPoint,
                DriveConstants.headingEpsilon);
    }

    /**
     * Applies motor brake by setting power to 0.
     * Relies on ZeroPowerBehavior.BRAKE to hold position.
     */
    private void applyBreak() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    
    /**
     * Calculates the angle from the robot's current position to the target position.
     * 
     * @param targetX Target X coordinate in inches.
     * @param targetY Target Y coordinate in inches.
     * @return Angle in radians from current position to target (range: -π to π).
     */
    public double getAngleToTarget(double targetX, double targetY) {
        Pose2D currentPose = getPose();
        double currentX = currentPose.getX(DistanceUnit.INCH);
        double currentY = currentPose.getY(DistanceUnit.INCH);
        
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        
        // atan2 returns angle in radians from -π to π
        // This angle points FROM current position TO target
        return Math.atan2(deltaY, deltaX);
    }
    
    /**
     * Calibrates the odometry position using vision data from AprilTag detection.
     * DIRECTLY RESETS the Pinpoint odometry position (like Prototype2026-Public).
     * Only calibrates when detecting RED goal (ID 24) or BLUE goal (ID 20) tags.
     * 
     * @param vision The Vision subsystem to get robot pose from.
     * @param alliance The alliance color for yaw offset.
     * @return True if calibration was successful (valid goal tag detected), false otherwise.
     */
    public boolean visionCalibrate(Vision vision, Vision.Alliance alliance) {
        // Only calibrate if we see a goal tag (red or blue), not obelisk tags
        int detectedTagId = vision.getDetectedTagId();
        if (detectedTagId != Vision.BLUE_GOAL_TAG_ID && detectedTagId != Vision.RED_GOAL_TAG_ID) {
            return false;  // Not a goal tag, don't calibrate
        }
        
        Pose3D visionPose = vision.getRobotPose();
        
        if (visionPose == null) {
            return false;  // No valid vision data
        }
        
        // Convert Limelight Pose3D to field coordinates
        Pose2D calibratedPose = Util.visionPoseToPinpointPose(visionPose);
        
        // DIRECTLY SET the Pinpoint position (like Prototype2026-Public)
        pinpoint.setPosition(calibratedPose);
        
        // Set yaw offset based on alliance (for field-centric driving)
        yawOffset = (alliance == Vision.Alliance.BLUE) ? Math.PI : 0;
        
        hasVisionCalibrated = true;
        
        return true;  // Calibration successful
    }
    
    /**
     * Gets the current pose (same as getPose, for compatibility).
     * After vision calibration, this returns the calibrated position.
     */
    public Pose2D getAbsolutePose() {
        return getPose();
    }
    
    /**
     * Checks if vision calibration has been performed.
     * @return True if vision has calibrated at least once.
     */
    public boolean hasVisionCalibrated() {
        return hasVisionCalibrated;
    }

    // ==================== ABSOLUTE POSITION UPDATE ====================
    
    // Turret geometry constants (from TurretConstants, in mm then converted to inches)
    // Turret center is 47mm behind chassis center
    private static final double TURRET_OFFSET_INCHES = 47.0 / 25.4;  // ~1.85"
    // Limelight is 140.86521mm from turret center
    private static final double LIMELIGHT_OFFSET_INCHES = 140.86521 / 25.4;  // ~5.55"
    
    /**
     * Updates absolute field position from Vision with turret angle compensation.
     * 
     * Limelight is mounted on the turret, so its position relative to chassis center
     * changes with turret angle. This method calculates:
     * 1. Limelight position in field (from vision)
     * 2. Limelight offset from chassis center (based on turret angle)
     * 3. Chassis center = Limelight position - offset (transformed to field coordinates)
     * 
     * Coordinate system:
     * - Chassis: +X = forward, +Y = right
     * - Field: standard FTC field coordinates
     * - Turret: 0° = forward, + = clockwise (right)
     * 
     * @param vision The Vision subsystem
     * @param turretAngleRad Current turret angle in radians (0 = forward, + = right)
     * @return true if vision update was successful
     */
    public boolean updateAbsolutePositionFromVisionWithTurret(Vision vision, double turretAngleRad) {
        int tagId = vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        if (!isGoalTag) {
            return false;
        }
        
        Pose3D visionPose = vision.getRobotPose();
        if (visionPose == null) {
            return false;
        }
        
        // Step 1: Get Limelight's position in field (from vision)
        // Note: visionPoseToPinpointPose returns what Limelight thinks is "robot" position
        // But since Limelight is on turret, this is actually Limelight's position
        Pose2D limelightPose = Util.visionPoseToPinpointPose(visionPose);
        double limelightX = limelightPose.getX(DistanceUnit.INCH);
        double limelightY = limelightPose.getY(DistanceUnit.INCH);
        double limelightHeading = limelightPose.getHeading(AngleUnit.RADIANS);  // This is LIMELIGHT's heading!
        
        // Step 2: Calculate ROBOT heading from Limelight heading
        // Limelight heading = Robot heading + Turret angle (in field coordinates)
        // Therefore: Robot heading = Limelight heading - Turret angle
        double robotHeading = Util.normalizeAngleRadians(limelightHeading - turretAngleRad);
        
        // Step 3: Calculate Limelight offset from chassis center (in chassis coordinates)
        // Chassis coordinate system: +X = forward, +Y = right
        // Turret center is TURRET_OFFSET behind chassis center (negative X)
        // Limelight is LIMELIGHT_OFFSET from turret center at angle turretAngleRad
        double limelightOffsetX_chassis = -TURRET_OFFSET_INCHES + LIMELIGHT_OFFSET_INCHES * Math.cos(turretAngleRad);
        double limelightOffsetY_chassis = LIMELIGHT_OFFSET_INCHES * Math.sin(turretAngleRad);
        
        // Step 4: Transform offset from chassis coordinates to field coordinates
        // Rotation by ROBOT heading (not Limelight heading!)
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);
        double limelightOffsetX_field = limelightOffsetX_chassis * cosH - limelightOffsetY_chassis * sinH;
        double limelightOffsetY_field = limelightOffsetX_chassis * sinH + limelightOffsetY_chassis * cosH;
        
        // Step 5: Chassis center = Limelight position - offset
        absoluteX = limelightX - limelightOffsetX_field;
        absoluteY = limelightY - limelightOffsetY_field;
        absoluteHeading = robotHeading;  // Now this is the correct ROBOT heading
        
        hasAbsolutePosition = true;
        
        // Update lastOdoPose for delta calculation when vision is lost
        lastOdoPose = getPose();
        
        return true;
    }
    
    /**
     * Updates absolute field position from Vision (LEGACY - no turret compensation).
     * @deprecated Use updateAbsolutePositionFromVisionWithTurret() when turret is available.
     * 
     * @param vision The Vision subsystem
     * @return true if vision update was successful
     */
    @Deprecated
    public boolean updateAbsolutePositionFromVision(Vision vision) {
        int tagId = vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        if (!isGoalTag) {
            return false;
        }
        
        Pose3D visionPose = vision.getRobotPose();
        if (visionPose == null) {
            return false;
        }
        
        // Convert Limelight Pose3D to field coordinates using shared utility
        // WARNING: This assumes Limelight is at chassis center (no turret compensation)
        Pose2D convertedPose = Util.visionPoseToPinpointPose(visionPose);
        absoluteX = convertedPose.getX(DistanceUnit.INCH);
        absoluteY = convertedPose.getY(DistanceUnit.INCH);
        absoluteHeading = convertedPose.getHeading(AngleUnit.RADIANS);
        
        hasAbsolutePosition = true;
        
        // Update lastOdoPose for delta calculation when vision is lost
        lastOdoPose = getPose();
        
        return true;
    }
    
    /**
     * Updates absolute field position using odometry dead-reckoning.
     * Call this when no goal tag is visible.
     * Uses delta from last odometry reading to update absolute position.
     */
    public void updateAbsolutePositionFromOdometry() {
        Pose2D currentOdoPose = getPose();
        
        if (lastOdoPose == null) {
            // First time, just record current pose
            lastOdoPose = currentOdoPose;
            return;
        }
        
        if (!hasAbsolutePosition) {
            // No valid absolute position yet, just update lastOdoPose
            lastOdoPose = currentOdoPose;
            return;
        }
        
        // Calculate delta from odometry
        double deltaX = currentOdoPose.getX(DistanceUnit.INCH) - lastOdoPose.getX(DistanceUnit.INCH);
        double deltaY = currentOdoPose.getY(DistanceUnit.INCH) - lastOdoPose.getY(DistanceUnit.INCH);
        double deltaHeading = currentOdoPose.getHeading(AngleUnit.RADIANS) - lastOdoPose.getHeading(AngleUnit.RADIANS);
        
        // Apply delta to absolute position
        absoluteX += deltaX;
        absoluteY += deltaY;
        absoluteHeading += deltaHeading;
        
        // Normalize heading to [-π, π]
        absoluteHeading = Util.normalizeAngleRadians(absoluteHeading);
        
        // Update lastOdoPose
        lastOdoPose = currentOdoPose;
    }
    
    /**
     * Gets the absolute X coordinate in field (inches).
     */
    public double getAbsoluteX() {
        return absoluteX;
    }
    
    /**
     * Gets the absolute Y coordinate in field (inches).
     */
    public double getAbsoluteY() {
        return absoluteY;
    }
    
    /**
     * Gets the absolute heading in field (radians).
     */
    public double getAbsoluteHeading() {
        return absoluteHeading;
    }
    
    /**
     * Checks if we have a valid absolute position.
     */
    public boolean hasAbsolutePosition() {
        return hasAbsolutePosition;
    }
    
    /**
     * Resets absolute position to (0, 0, 0).
     */
    public void resetAbsolutePosition() {
        absoluteX = 0;
        absoluteY = 0;
        absoluteHeading = 0;
        hasAbsolutePosition = false;
        lastOdoPose = null;
    }
    
    /**
     * Calculates distance from absolute position to a target point.
     * 
     * @param targetX Target X in inches
     * @param targetY Target Y in inches
     * @return Distance in inches, or -1 if no valid absolute position
     */
    public double distanceToPoint(double targetX, double targetY) {
        if (!hasAbsolutePosition) {
            return -1;
        }
        double dx = targetX - absoluteX;
        double dy = targetY - absoluteY;
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    // ==================== ADAPTIVE SHOOTING ====================
    
    /**
     * Calculates distance to goal based on detected tag.
     * - Blue goal (tag 20): (4, 140)
     * - Red goal (tag 24): (140, 140)
     * 
     * @param tagId The detected tag ID
     * @return Distance in inches, or -1 if invalid
     */
    public double distanceToGoal(int tagId) {
        if (!hasAbsolutePosition) {
            return -1;
        }
        
        double targetX, targetY;
        if (tagId == Vision.BLUE_GOAL_TAG_ID) {
            targetX = ShooterConstants.blueGoalX;
            targetY = ShooterConstants.blueGoalY;
        } else if (tagId == Vision.RED_GOAL_TAG_ID) {
            targetX = ShooterConstants.redGoalX;
            targetY = ShooterConstants.redGoalY;
        } else {
            return -1;  // Not a goal tag
        }
        
        return distanceToPoint(targetX, targetY);
    }
    
    /**
     * Calculates theoretical shooter velocity based on distance to goal.
     * Uses linear interpolation between slowVelocity and fastVelocity.
     * 
     * @param tagId The detected tag ID
     * @return Target velocity in TPS (negative), defaults to midVelocity if no valid distance
     */
    public double calculateAdaptiveVelocity(int tagId) {
        double distance = distanceToGoal(tagId);
        
        if (distance < 0) {
            // No valid distance, use mid velocity as default
            return ShooterConstants.midVelocity;
        }
        
        // Piecewise linear interpolation based on calibrated data:
        // Near: 24.4" -> 700 TPS, Mid: 77.4" -> 950 TPS, Far: 128.4" -> 1420 TPS
        
        if (distance <= ShooterConstants.nearDistance) {
            // Closer than near distance, use slow velocity
            return ShooterConstants.slowVelocity;
        }
        if (distance >= ShooterConstants.farDistance) {
            // Further than far distance, use fast velocity
            return ShooterConstants.fastVelocity;
        }
        
        if (distance < ShooterConstants.midDistance) {
            // Between near and mid: interpolate slow -> mid
            double ratio = (distance - ShooterConstants.nearDistance) 
                         / (ShooterConstants.midDistance - ShooterConstants.nearDistance);
            return ShooterConstants.slowVelocity 
                 + (ShooterConstants.midVelocity - ShooterConstants.slowVelocity) * ratio;
        } else {
            // Between mid and far: interpolate mid -> fast
            double ratio = (distance - ShooterConstants.midDistance) 
                         / (ShooterConstants.farDistance - ShooterConstants.midDistance);
            return ShooterConstants.midVelocity 
                 + (ShooterConstants.fastVelocity - ShooterConstants.midVelocity) * ratio;
        }
    }
    
    /**
     * Gets the current adaptive velocity segment name based on distance.
     * @param tagId The detected tag ID
     * @return Segment name: "NEAR", "NEAR→MID", "MID→FAR", "FAR", or "UNKNOWN"
     */
    public String getAdaptiveSegment(int tagId) {
        double distance = distanceToGoal(tagId);
        
        if (distance < 0) {
            return "UNKNOWN";
        }
        
        if (distance <= ShooterConstants.nearDistance) {
            return "NEAR (≤24.4\")";
        }
        if (distance >= ShooterConstants.farDistance) {
            return "FAR (≥128.4\")";
        }
        
        if (distance < ShooterConstants.midDistance) {
            return "NEAR→MID (24.4\"~77.4\")";
        } else {
            return "MID→FAR (77.4\"~128.4\")";
        }
    }
    
    /**
     * Calculates adaptive servo position based on distance to goal.
     * Uses non-linear (square root) interpolation:
     * - Near distance changes servo angle more rapidly
     * - Far distance changes servo angle more slowly
     * 
     * @param tagId The detected tag ID
     * @return Servo position (0.85 for near, 0.29 for far), defaults to mid position if invalid
     */
    public double calculateAdaptiveServoPosition(int tagId) {
        double distance = distanceToGoal(tagId);
        
        if (distance < 0) {
            // No valid distance, use mid servo position as default
            return ShooterConstants.shooterServoMidPos;
        }
        
        // Clamp distance to range
        if (distance <= ShooterConstants.servoNearDistance) {
            return ShooterConstants.shooterServoDownPos;  // 0.85 (near/low angle)
        }
        if (distance >= ShooterConstants.servoFarDistance) {
            return ShooterConstants.shooterServoUpPos;    // 0.29 (far/high angle)
        }
        
        // Non-linear interpolation using square root
        // sqrt makes the curve steep at the beginning (near) and flat at the end (far)
        // ratio = sqrt((distance - near) / (far - near))
        double normalizedDistance = (distance - ShooterConstants.servoNearDistance) 
                                  / (ShooterConstants.servoFarDistance - ShooterConstants.servoNearDistance);
        double ratio = Math.sqrt(normalizedDistance);
        
        // Interpolate: down (0.85) -> up (0.29)
        // servoPos = down + (up - down) * ratio = 0.85 + (0.29 - 0.85) * ratio = 0.85 - 0.56 * ratio
        return ShooterConstants.shooterServoDownPos 
             + (ShooterConstants.shooterServoUpPos - ShooterConstants.shooterServoDownPos) * ratio;
    }
    
    /**
     * Checks if auto-fire is allowed (aligned within threshold).
     * 
     * @param tx The horizontal offset in degrees from Vision
     * @return true if aligned within autoFireTxThreshold (tx close to 0)
     */
    public boolean isAutoFireAllowed(double tx) {
        // Target is tx = 0 (centered)
        return Math.abs(tx) < ShooterConstants.autoFireTxThreshold;
    }
    
    // ==================== AUTO-AIM METHODS ====================
    
    /**
     * Calculates turn power for auto-aim using vision feedback.
     * Features:
     * - Low-pass filtered tx for stability (reduces Limelight noise)
     * - Hysteresis deadband to prevent oscillation at boundary
     * - Distance-adaptive PID (weaker at close range, stronger at far range)
     * - Distance-based offset compensation for far shots
     * 
     * @param vision The Vision subsystem.
     * @return Turn power (-1 to 1). Positive = turn right, Negative = turn left.
     */
    public double getAlignTurnPower(Vision vision) {
        int tagId = vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        if (isGoalTag) {
            // ==================== MODE 1: TX-BASED ALIGNMENT ====================
            // Record which tag we're aligning to
            lastAlignedTagId = tagId;
            hasLastAlignedTag = true;
            
            // Get raw tx value (horizontal offset in degrees)
            double rawTx = vision.getTx();
            
            // ========== LOW-PASS FILTER ==========
            // Smooths out Limelight noise, especially noticeable at close range
            if (!txFilterInitialized) {
                filteredTx = rawTx;
                txFilterInitialized = true;
            } else {
                // filteredTx = alpha * rawTx + (1-alpha) * filteredTx
                filteredTx = txFilterAlpha * rawTx + (1 - txFilterAlpha) * filteredTx;
            }
            double tx = filteredTx;
            
            // ========== DISTANCE-BASED PARAMETERS ==========
            // Lock offset and deadband once determined
            double distance = vision.getDistanceToTag();
            
            if (!offsetLocked) {
                if (distance > 0 && distance > farDistanceThreshold) {
                    // Far shot: apply offset and use smaller deadband
                    currentOffset = (tagId == Vision.BLUE_GOAL_TAG_ID) ? -farOffsetDegrees : farOffsetDegrees;
                    currentDeadband = alignDeadbandFar;
                } else {
                    // Close shot: no offset, larger deadband
                    currentOffset = 0;
                    currentDeadband = alignDeadbandNear;
                }
                offsetLocked = true;
            }
            
            // ========== DISTANCE-ADAPTIVE PID ==========
            // Interpolate kP between near and far values based on distance
            double effectiveKp;
            if (distance <= 0) {
                effectiveKp = kP_alignH;  // Fallback
            } else if (distance < 40) {
                effectiveKp = kP_near;  // Close range: weak P to prevent overshoot
            } else if (distance > 100) {
                effectiveKp = kP_far;   // Far range: strong P for precision
            } else {
                // Linear interpolation between 40" and 100"
                double ratio = (distance - 40) / 60.0;
                effectiveKp = kP_near + (kP_far - kP_near) * ratio;
            }
            alignPID.setPID(effectiveKp, kI_alignH, kD_alignH);
            
            // ========== HYSTERESIS DEADBAND ==========
            // Entry threshold: currentDeadband
            // Exit threshold: currentDeadband + hysteresis
            // This prevents oscillation at the deadband boundary
            double targetTx = -currentOffset;
            double error = Math.abs(tx - targetTx);
            
            double entryThreshold = currentDeadband;
            double exitThreshold = currentDeadband + alignHysteresis;
            
            if (isAligned) {
                // Currently aligned: stay aligned until error exceeds exit threshold
                if (error > exitThreshold) {
                    isAligned = false;  // Exit aligned state, start correcting
                } else {
                    // Still aligned, but DON'T reset PID (keep D term history)
                    return 0;
                }
            } else {
                // Currently aligning: become aligned when error drops below entry threshold
                if (error < entryThreshold) {
                    isAligned = true;  // Enter aligned state
                    return 0;
                }
            }
            
            // ========== PID CONTROL ==========
            double turn = -alignPID.calculate(tx, targetTx);
            return Math.max(-1, Math.min(1, turn));
            
        } else {
            // No goal tag visible, reset state
            offsetLocked = false;
            currentOffset = 0;
            headingTargetLocked = false;
            isAligned = false;
            txFilterInitialized = false;
            alignPID.reset();
            return 0;
        }
    }
    
    /**
     * Calculates the heading (in radians) from robot's absolute position to a goal.
     * Returns the heading where robot's BACK faces the goal (for shooting).
     * 
     * @param goalX Goal X coordinate
     * @param goalY Goal Y coordinate
     * @return Target heading in radians
     */
    private double calculateHeadingToGoal(double goalX, double goalY) {
        double dx = goalX - absoluteX;
        double dy = goalY - absoluteY;
        // atan2 gives angle from robot to goal
        // Subtract PI because we want robot's BACK to face the goal
        return Math.atan2(dy, dx) - Math.PI;
    }
    
    /**
     * Resets the auto-aim offset lock, heading target lock, filter, and PID.
     * Call this when releasing the aim button.
     */
    public void resetAutoAimOffset() {
        // Reset tx-based offset lock (Mode 1)
        offsetLocked = false;
        currentOffset = 0;
        
        // Reset hysteresis state
        isAligned = false;
        
        // Reset low-pass filter
        txFilterInitialized = false;
        filteredTx = 0;
        
        // Reset heading-based target lock (Mode 2)
        headingTargetLocked = false;
        lockedTargetGoalX = 0;
        lockedTargetGoalY = 0;
        
        alignPID.reset();
    }
    
    @Override
    public void periodic() {
        // Update Pinpoint driver to ensure latest data is available
        pinpoint.update();

        // If no gamepad input is detected, apply brakes to hold position
        if (!isGamepadOn) {
            applyBreak();
        }

        // Update last recorded pose
        lastPose = getPose();
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
