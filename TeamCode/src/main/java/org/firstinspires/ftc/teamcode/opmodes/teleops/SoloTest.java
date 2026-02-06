package org.firstinspires.ftc.teamcode.opmodes.teleops;

/**
 * SoloTest - Turret Auto-Aim with Inertial Navigation
 * 
 * === AUTO-AIM LOGIC ===
 * Step 1: See any tag (20 or 24) → Calculate Limelight's field position
 * Step 2: Use geometry (LL offset, turret offset, turret angle) → Calculate robot's absolute position & heading
 * Step 3: Use robot position + goal position → Calculate turret target angle
 * 
 * === CONTROLS ===
 * - Start: Turret locked at 0°
 * - See tag + Press A: Start inertial navigation (auto-aim)
 * - During auto-aim + Press A: Return to 0° and lock (back to initial state)
 * 
 * Target: RED goal (tag 24 basket at 140, 140)
 */

import com.acmerobotics.dashboard.FtcDashboard; 
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.utils.Util;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SoloTest", group = "TeleOp")
public class SoloTest extends CommandOpMode {
    private Robot robot;
    private GamepadEx gamepadEx1;
    private boolean[] isAuto = {false};
    
    // Edge detection
    private boolean lastAButton = false;
    
    // ========== AUTO-AIM STATE ==========
    private enum AimState {
        LOCKED_AT_ZERO,      // Turret locked at 0°, waiting for tag + A
        INERTIAL_NAVIGATION  // Auto-aiming using inertial navigation
    }
    private AimState aimState = AimState.LOCKED_AT_ZERO;
    private boolean hasValidPosition = false;
    
    // Goal coordinates (RED goal)
    private static final double GOAL_X = TurretConstants.redGoalX;  // 140
    private static final double GOAL_Y = TurretConstants.redGoalY;  // 140
    
    // Software limits
    private static final double MIN_TURRET_ANGLE = -145.0;
    private static final double MAX_TURRET_ANGLE = 226.2;
    private static final double FLIP_THRESHOLD = 185.0;
    
    // Flip state
    private boolean isFlipping = false;
    private double flipTargetAngle = 0;
    
    // Cached position values (used by turret algorithm, telemetry, and dashboard)
    private double cachedRobotX = 0;
    private double cachedRobotY = 0;
    private double cachedHeadingDeg = 0;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        // Register subsystems
        CommandScheduler.getInstance().registerSubsystem(robot.shooter);
        CommandScheduler.getInstance().registerSubsystem(robot.transit);
        CommandScheduler.getInstance().registerSubsystem(robot.intake);
        if (robot.turret != null) {
            CommandScheduler.getInstance().registerSubsystem(robot.turret);
            robot.turret.setAlliance(Turret.Alliance.RED);
            // Start with turret locked at 0° (same as Solo)
            robot.turret.enableSoftLock(0);
            aimState = AimState.LOCKED_AT_ZERO;
        }

        // Left stick button: Reset heading
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> robot.drive.resetHeading())
        );

        DriverControls.bind(gamepadEx1, robot, isAuto);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        telemetry.addLine("=== SOLO TEST (Inertial Auto-Aim) ===");
        telemetry.addLine("See tag + Press A → Start auto-aim");
        telemetry.addLine("During auto-aim + Press A → Back to 0°");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        
        // ========== MANUAL DRIVE ==========
        double leftX = -gamepadEx1.getLeftX();
        double leftY = gamepadEx1.getLeftY();
        double rightX = gamepadEx1.getRightX();
        robot.drive.setGamepad(true);
        robot.drive.moveRobotFieldRelative(leftY, leftX, rightX);
        
        // ========== UPDATE ABSOLUTE POSITION ==========
        updateAbsolutePosition();
        
        // ========== CACHE POSITION VALUES (used everywhere) ==========
        if (hasValidPosition) {
            cachedRobotX = robot.drive.getAbsoluteX();
            cachedRobotY = robot.drive.getAbsoluteY();
            cachedHeadingDeg = Math.toDegrees(robot.drive.getAbsoluteHeading());
        }
        
        // ========== A BUTTON STATE MACHINE ==========
        boolean aButton = gamepadEx1.getButton(GamepadKeys.Button.A);
        boolean aPressed = aButton && !lastAButton;  // Rising edge
        
        if (robot.turret != null && robot.vision != null) {
            int tagId = robot.vision.getDetectedTagId();
            boolean canSeeGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
            
            switch (aimState) {
                case LOCKED_AT_ZERO:
                    // Turret ALWAYS locked at 0° in this state (enforce every loop)
                    // This prevents any drift or unintended tracking
                    if (robot.turret.getLockMode() != Turret.LockMode.SOFT_LOCK || 
                        Math.abs(robot.turret.getTargetAngle()) > 0.1) {
                        robot.turret.enableSoftLock(0);
                    }
                    
                    // If see tag AND press A → start inertial navigation
                    if (aPressed && canSeeGoalTag && hasValidPosition) {
                        aimState = AimState.INERTIAL_NAVIGATION;
                        isFlipping = false;
                    }
                    break;
                    
                case INERTIAL_NAVIGATION:
                    // Auto-aiming with inertial navigation
                    // Press A → return to 0° and lock (back to initial state)
                    if (aPressed) {
                        aimState = AimState.LOCKED_AT_ZERO;
                        robot.turret.enableSoftLock(0);
                        isFlipping = false;
                    } else {
                        // Calculate and apply turret angle
                        calculateAndApplyTurretAngle();
                    }
                    break;
            }
        }
        lastAButton = aButton;
        
        // ========== INTAKE/SHOOTER CONTROLS ==========
        handleIntakeShooter();
        
        // ========== TELEMETRY ==========
        updateTelemetry();

        // Dashboard field drawing
        TelemetryPacket packet = new TelemetryPacket();
        
        // Draw robot at odometry position (blue)
        org.firstinspires.ftc.teamcode.utils.DashboardUtil.drawRobot(packet, robot.drive.getPose());
        
        // Draw absolute position if available (green circle + line for heading)
        // Uses cached values (same as turret algorithm)
        if (hasValidPosition) {
            double absX = cachedRobotX;
            double absY = cachedRobotY;
            double absHeadingRad = Math.toRadians(cachedHeadingDeg);
            
            // Add to packet telemetry
            packet.put("Abs X", String.format("%.1f", absX));
            packet.put("Abs Y", String.format("%.1f", absY));
            packet.put("Abs Heading", String.format("%.1f°", cachedHeadingDeg));
            
            // Draw green circle at absolute position
            packet.fieldOverlay()
                    .setStroke("#00FF00")
                    .setStrokeWidth(2)
                    .strokeCircle(absX, absY, 4);
            
            // Draw heading line
            double lineLen = 8;
            double endX = absX + lineLen * Math.cos(absHeadingRad);
            double endY = absY + lineLen * Math.sin(absHeadingRad);
            packet.fieldOverlay()
                    .setStroke("#00FF00")
                    .strokeLine(absX, absY, endX, endY);
            
            // Draw line to goal (red dashed)
            packet.fieldOverlay()
                    .setStroke("#FF0000")
                    .setStrokeWidth(1)
                    .strokeLine(absX, absY, GOAL_X, GOAL_Y);
        }
        
        // Draw goal position (red circle)
        packet.fieldOverlay()
                .setStroke("#FF0000")
                .setFill("#FF000044")
                .fillCircle(GOAL_X, GOAL_Y, 3);
        
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    
    /**
     * Update robot's absolute position from vision or odometry
     */
    private void updateAbsolutePosition() {
        if (robot.vision == null || robot.turret == null) return;
        
        int tagId = robot.vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        if (isGoalTag) {
            // Pass turret angle in DEGREES (method only updates when turret is near 0°)
            double turretAngleDeg = robot.turret.isCalibrated() ? robot.turret.getAngleDegrees() : 0;
            boolean success = robot.drive.updateAbsolutePositionFromVisionWithTurret(robot.vision, turretAngleDeg);
            if (success) {
                hasValidPosition = true;
            }
        } else {
            robot.drive.updateAbsolutePositionFromOdometry();
        }
    }
    
    // Debug values for telemetry
    private double debugGoalFieldAngle = 0;  // x from atan equation
    private double debugTurnRight = 0;       // heading - x
    private double debugTurnLeft = 0;        // -((360 - heading) + x)
    private double debugChosenAngle = 0;     // final choice
    
    /**
     * Calculate turret target angle using simplified geometry:
     * 
     * STEP 1: Get chassis heading (0-360°) and absolute position (x, y)
     * STEP 2: Solve tan(angle) = (140 - y) / (140 - x) → angle = atan2(dy, dx)
     * STEP 3: Calculate two options:
     *         - Turn RIGHT: heading - angle
     *         - Turn LEFT:  -((360 - heading) + angle)
     * STEP 4: Choose the option within limits; if both valid, pick smaller |angle|
     * 
     * Turret coord: positive = RIGHT, negative = LEFT
     * Limits: -145° to +240°
     */
    private void calculateAndApplyTurretAngle() {
        if (!hasValidPosition || robot.turret == null) return;
        
        // === STEP 1: Use cached position (already updated this loop) ===
        double robotX = cachedRobotX;
        double robotY = cachedRobotY;
        
        // Heading in [0, 360)
        double heading = Util.normalizeAngleDegrees0To360(cachedHeadingDeg);
        
        // === STEP 2: Solve for goal field angle ===
        // tan(x) = (140 - robotY) / (140 - robotX)
        // x = atan2(dy, dx)
        double dx = GOAL_X - robotX;  // 140 - robotX
        double dy = GOAL_Y - robotY;  // 140 - robotY
        double x = Math.toDegrees(Math.atan2(dy, dx));
        // Normalize x to [0, 360)
        x = Util.normalizeAngleDegrees0To360(x);
        
        // === STEP 3: Calculate two options ===
        // Turn RIGHT (positive turret angle): heading - x
        double turnRight = heading - x;
        
        // Turn LEFT (negative turret angle): -((360 - heading) + x)
        double turnLeft = -((360.0 - heading) + x);
        
        // Save for telemetry
        debugGoalFieldAngle = x;
        debugTurnRight = turnRight;
        debugTurnLeft = turnLeft;
        
        // === STEP 4: Choose the best option ===
        boolean rightInLimits = (turnRight >= MIN_TURRET_ANGLE && turnRight <= MAX_TURRET_ANGLE);
        boolean leftInLimits = (turnLeft >= MIN_TURRET_ANGLE && turnLeft <= MAX_TURRET_ANGLE);
        
        double targetAngle;
        if (rightInLimits && leftInLimits) {
            // Both valid - pick the one with smaller absolute value (shorter turn)
            targetAngle = (Math.abs(turnRight) <= Math.abs(turnLeft)) ? turnRight : turnLeft;
        } else if (rightInLimits) {
            targetAngle = turnRight;
        } else if (leftInLimits) {
            targetAngle = turnLeft;
        } else {
            // Neither in limits - clamp turnRight to nearest limit
            targetAngle = turnRight;
            if (targetAngle < MIN_TURRET_ANGLE) targetAngle = MIN_TURRET_ANGLE;
            if (targetAngle > MAX_TURRET_ANGLE) targetAngle = MAX_TURRET_ANGLE;
        }
        
        debugChosenAngle = targetAngle;
        
        // Apply to turret
        robot.turret.enableSoftLock(targetAngle);
    }
    
    /**
     * Handle intake/shooter controls
     */
    private void handleIntakeShooter() {
        boolean shooterAccelerationPressed =
                gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER) ||
                gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER) ||
                gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.3;
        boolean feedPressed =
                shooterAccelerationPressed &&
                gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.3;
        robot.intake.setShooting(feedPressed);
        
        boolean intakeAccelerationPressed =
                gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.3;
        robot.intake.setFastIntaking(intakeAccelerationPressed);
    }
    
    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        Pose2D pose = robot.drive.getPose();
        
        // === STATE ===
        telemetry.addLine("========== STATE ==========");
        telemetry.addData("Aim State", aimState.toString());
        telemetry.addData("Has Position", hasValidPosition ? "YES" : "NO");
        telemetry.addData("Is Flipping", isFlipping ? "YES" : "NO");
        
        // === VISION ===
        telemetry.addLine("========== VISION ==========");
        if (robot.vision != null) {
            int tagId = robot.vision.getDetectedTagId();
            boolean canSee = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
            telemetry.addData("CAN SEE TAG", canSee ? "YES ✓" : "NO ✗");
            telemetry.addData("Tag ID", tagId == -1 ? "NONE" : tagId);
            telemetry.addData("TX", String.format("%.1f°", robot.vision.getTx()));
        }
        
        // === POSITION (cached - same as turret algorithm) ===
        telemetry.addLine("========== POSITION ==========");
        if (hasValidPosition) {
            telemetry.addData("Abs X", String.format("%.1f in", cachedRobotX));
            telemetry.addData("Abs Y", String.format("%.1f in", cachedRobotY));
            telemetry.addData("Abs Heading", String.format("%.1f°", cachedHeadingDeg));
        } else {
            telemetry.addLine("Waiting for tag...");
        }
        
        // === TURRET ===
        if (robot.turret != null) {
            telemetry.addLine("========== TURRET ==========");
            telemetry.addData("Current Angle", String.format("%.1f°", robot.turret.getAngleDegrees()));
            telemetry.addData("Target Angle", String.format("%.1f°", robot.turret.getTargetAngle()));
            telemetry.addData("Goal Pos", String.format("X=%.0f, Y=%.0f", GOAL_X, GOAL_Y));
            
            // === AIM DEBUG (Simplified geometry) ===
            if (hasValidPosition && aimState == AimState.INERTIAL_NAVIGATION) {
                telemetry.addLine("---------- AIM DEBUG ----------");
                telemetry.addData("Goal Angle (x)", String.format("%.1f°", debugGoalFieldAngle));
                telemetry.addData("Turn RIGHT", String.format("%.1f° %s", 
                        debugTurnRight,
                        (debugTurnRight >= MIN_TURRET_ANGLE && debugTurnRight <= MAX_TURRET_ANGLE) ? "✓" : "✗"));
                telemetry.addData("Turn LEFT", String.format("%.1f° %s", 
                        debugTurnLeft,
                        (debugTurnLeft >= MIN_TURRET_ANGLE && debugTurnLeft <= MAX_TURRET_ANGLE) ? "✓" : "✗"));
                telemetry.addData("CHOSEN", String.format("%.1f°", debugChosenAngle));
            }
        }
        
        // === ODOMETRY ===
        telemetry.addLine("========== ODOMETRY ==========");
        telemetry.addData("X", String.format("%.1f in", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Y", String.format("%.1f in", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS))));
        
        // === SHOOTER ===
        telemetry.addLine("========== SHOOTER ==========");
        telemetry.addData("READY", robot.shooter.isShooterAtSetPoint());
        telemetry.addData("State", robot.shooter.shooterState);
        
        // === CONTROLS ===
        telemetry.addLine("========== CONTROLS ==========");
        if (aimState == AimState.LOCKED_AT_ZERO) {
            telemetry.addLine("See tag + A → Start auto-aim");
        } else {
            telemetry.addLine("A → Return to 0° (stop auto-aim)");
        }
        
        telemetry.update();
    }
}
