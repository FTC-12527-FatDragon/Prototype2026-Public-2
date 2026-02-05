package org.firstinspires.ftc.teamcode.opmodes.teleops;

/**
 * Test TeleOp - Turret Auto-Aim to RED Goal (Tag 24)
 * 
 * Features:
 * - Absolute position calibration when seeing ANY goal tag (20 or 24)
 * - Turret ONLY aims at RED goal (tag 24 basket)
 * - Uses HARD_LOCK mode: TX tracking when seeing tag 24, inertial navigation otherwise
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
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Config
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SoloTest", group = "TeleOp")
public class SoloTest extends CommandOpMode {
    private Robot robot;
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;
    private boolean[] isAuto = {false};
    
    // Edge detection for gamepad2 emergency disable combos
    private boolean lastIntakeDisableCombo = false;
    private boolean lastShooterDisableCombo = false;
    private boolean lastTurretDisableCombo = false;
    
    // Edge detection for GP2 right stick (set turret home)
    private boolean lastGP2RightStickButton = false;
    
    // ========== TURRET AIM STATE ==========
    private enum AimState {
        WAITING_FOR_TAG,   // Turret locked at 0°, waiting to see tag
        TX_LOCKING,        // Doing TX lock after seeing tag + pressing A
        INERTIAL_TRACKING  // TX lock done, using inertial navigation
    }
    private AimState aimState = AimState.WAITING_FOR_TAG;
    private boolean lastAButton = false;
    
    // TX lock parameters
    public static long TX_LOCK_DURATION_MS = 500;   // How long to TX-lock when triggered
    private long txLockStartTime = 0;
    
    // ========== INERTIAL NAVIGATION (Delta-based) ==========
    // Recorded at TX lock completion
    private double lockedTurretAngle = 0;        // Turret angle when locked
    private double lockedOdoX = 0;               // Odometry X when locked
    private double lockedOdoY = 0;               // Odometry Y when locked  
    private double lockedOdoHeading = 0;         // Odometry heading when locked
    private double lockedAbsX = 0;               // Absolute X when locked (for distance calc)
    private double lockedAbsY = 0;               // Absolute Y when locked
    private boolean hasLockedPosition = false;   // Whether we have a valid locked position

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // Register all subsystems including turret
        CommandScheduler.getInstance().registerSubsystem(robot.shooter);
        CommandScheduler.getInstance().registerSubsystem(robot.transit);
        CommandScheduler.getInstance().registerSubsystem(robot.intake);
        if (robot.turret != null) {
            CommandScheduler.getInstance().registerSubsystem(robot.turret);
            // Set alliance to RED - turret ONLY aims at tag 24 basket
            robot.turret.setAlliance(Turret.Alliance.RED);
            // Start with turret locked at 0° (waiting for tag)
            robot.turret.enableSoftLock(0);
            aimState = AimState.WAITING_FOR_TAG;
        }

        // Left stick button: Reset heading to 0
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> robot.drive.resetHeading())
        );

        DriverControls.bind(gamepadEx1, robot, isAuto);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        telemetry.addLine("=== SOLO TEST (RED GOAL AIM) ===");
        telemetry.addLine("Turret aims at RED goal (tag 24)");
        telemetry.addLine("Position calibrates on ANY goal tag");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        
        // ========== GAMEPAD2 RIGHT STICK: SET TURRET HOME ==========
        boolean gp2RightStickButton = gamepadEx2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        if (gp2RightStickButton && !lastGP2RightStickButton && robot.turret != null) {
            robot.turret.setCurrentAsHome();
        }
        lastGP2RightStickButton = gp2RightStickButton;
        
        // ========== GAMEPAD2 EMERGENCY DISABLE CONTROLS ==========
        boolean intakeDisableCombo = gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3 
                && gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean shooterDisableCombo = gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3 
                && gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean turretDisableCombo = gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER) 
                && gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                && !intakeDisableCombo && !shooterDisableCombo;
        
        if (intakeDisableCombo && !lastIntakeDisableCombo) {
            robot.intake.toggleDisabled();
        }
        if (shooterDisableCombo && !lastShooterDisableCombo) {
            robot.shooter.toggleDisabled();
        }
        if (turretDisableCombo && !lastTurretDisableCombo && robot.turret != null) {
            robot.turret.toggleDisabled();
        }
        
        lastIntakeDisableCombo = intakeDisableCombo;
        lastShooterDisableCombo = shooterDisableCombo;
        lastTurretDisableCombo = turretDisableCombo;
        
        // ========== MANUAL DRIVE ==========
        double leftX = -gamepadEx1.getLeftX();
        double leftY = gamepadEx1.getLeftY();
        double rightX = gamepadEx1.getRightX();
        robot.drive.setGamepad(true);
        robot.drive.moveRobotFieldRelative(leftY, leftX, rightX);
        
        // ========== ABSOLUTE POSITION CALIBRATION ==========
        // Calibrate position when seeing ANY goal tag (20 or 24)
        // But turret only aims at tag 24 (RED goal)
        if (robot.vision != null) {
            int currentTagId = robot.vision.getDetectedTagId();
            boolean isGoalTag = (currentTagId == Vision.BLUE_GOAL_TAG_ID || currentTagId == Vision.RED_GOAL_TAG_ID);
            
            if (isGoalTag) {
                // Get turret angle for position calculation
                double turretAngle = (robot.turret != null && robot.turret.isCalibrated()) 
                    ? robot.turret.getAngleRadians() : 0;
                // Update absolute position from vision (works with ANY goal tag)
                robot.drive.updateAbsolutePositionFromVisionWithTurret(robot.vision, turretAngle);
            } else {
                // No tag visible - update from odometry delta
                robot.drive.updateAbsolutePositionFromOdometry();
            }
        }
        
        // ========== GAMEPAD2 D-PAD: MANUAL TURRET AIM OFFSET ==========
        if (robot.turret != null) {
            boolean dpadLeft = gamepadEx2.getButton(GamepadKeys.Button.DPAD_LEFT);
            boolean dpadRight = gamepadEx2.getButton(GamepadKeys.Button.DPAD_RIGHT);
            
            if (dpadLeft) {
                // D-pad left: decrease aim offset (turn turret left/CCW)
                robot.turret.adjustManualOffset(-robot.turret.manualOffsetSpeed);
            } else if (dpadRight) {
                // D-pad right: increase aim offset (turn turret right/CW)
                robot.turret.adjustManualOffset(robot.turret.manualOffsetSpeed);
            }
            
            // Reset offset on D-pad down
            if (gamepadEx2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                robot.turret.resetManualOffset();
            }
        }
        
        // ========== TURRET AIM STATE MACHINE ==========
        boolean aButton = gamepadEx1.getButton(GamepadKeys.Button.A);
        Pose2D currentOdoPose = robot.drive.getPose();
        double currentOdoX = currentOdoPose.getX(DistanceUnit.INCH);
        double currentOdoY = currentOdoPose.getY(DistanceUnit.INCH);
        double currentOdoHeading = currentOdoPose.getHeading(AngleUnit.RADIANS);
        
        if (robot.turret != null && robot.vision != null) {
            int currentTagId = robot.vision.getDetectedTagId();
            boolean canSeeTargetTag = (currentTagId == Vision.RED_GOAL_TAG_ID);
            boolean canSeeAnyGoalTag = (currentTagId == Vision.RED_GOAL_TAG_ID || currentTagId == Vision.BLUE_GOAL_TAG_ID);
            
            switch (aimState) {
                case WAITING_FOR_TAG:
                    // Turret locked at 0°, waiting to see tag
                    // Press A while seeing target tag → start TX lock
                    if (aButton && !lastAButton && canSeeTargetTag) {
                        aimState = AimState.TX_LOCKING;
                        txLockStartTime = System.currentTimeMillis();
                        robot.turret.disableLock(); // Switch to manual for TX tracking
                    }
                    break;
                    
                case TX_LOCKING:
                    // TX lock: use current TX to fine-tune angle
                    if (canSeeTargetTag) {
                        double tx = robot.vision.getTx();
                        // Apply correction based on TX (P control)
                        double correction = tx * 0.03; // Gain for TX tracking
                        robot.turret.setMotorPower(-correction); // Negative: positive TX = target is right
                    } else {
                        robot.turret.setMotorPower(0); // Lost tag, stop
                    }
                    
                    // Check if lock duration expired
                    if (System.currentTimeMillis() - txLockStartTime >= TX_LOCK_DURATION_MS) {
                        // TX lock complete → record state for inertial tracking
                        robot.turret.setMotorPower(0);
                        
                        // Record turret angle at lock
                        lockedTurretAngle = robot.turret.getAngleDegrees();
                        
                        // Record odometry at lock (for delta calculation)
                        lockedOdoX = currentOdoX;
                        lockedOdoY = currentOdoY;
                        lockedOdoHeading = currentOdoHeading;
                        
                        // Record absolute position at lock (for distance calculation)
                        if (robot.drive.hasAbsolutePosition()) {
                            lockedAbsX = robot.drive.getAbsoluteX();
                            lockedAbsY = robot.drive.getAbsoluteY();
                            hasLockedPosition = true;
                        }
                        
                        // Switch to inertial tracking with SOFT_LOCK
                        aimState = AimState.INERTIAL_TRACKING;
                        robot.turret.enableSoftLock(lockedTurretAngle);
                    }
                    break;
                    
                case INERTIAL_TRACKING:
                    // Delta-based inertial navigation
                    // Odometry returns field coordinates, so delta is already in field frame
                    double deltaX = currentOdoX - lockedOdoX;
                    double deltaY = currentOdoY - lockedOdoY;
                    double deltaHeading = currentOdoHeading - lockedOdoHeading;
                    
                    // Calculate new turret angle based on delta
                    // Full calculation: account for both heading and position change
                    // Estimate current absolute position (odometry delta is already in field coords)
                    double estAbsX = lockedAbsX + deltaX;  // No rotation needed - already field coords
                    double estAbsY = lockedAbsY + deltaY;
                    
                    // Calculate angle to goal from estimated position
                    double goalX = TurretConstants.redGoalX;
                    double goalY = TurretConstants.redGoalY;
                    double dx = goalX - estAbsX;
                    double dy = goalY - estAbsY;
                    double fieldAngleToGoal = Math.atan2(dy, dx);  // Pedro: 0°=+X, 90°=+Y
                    
                    // Turret angle = robot heading - field angle (turret positive = CW from robot front)
                    // When robot turns left (heading+), turret should turn right (angle+) to compensate
                    double newTurretAngle = Math.toDegrees(currentOdoHeading - fieldAngleToGoal);
                    // Normalize to [-180, 180]
                    while (newTurretAngle > 180) newTurretAngle -= 360;
                    while (newTurretAngle < -180) newTurretAngle += 360;
                    
                    // === SIMPLIFIED VERSION (commented out - using full version only) ===
                    // if (!hasLockedPosition) {
                    //     // Simplified: only compensate for heading change
                    //     // If chassis turns left (heading+), turret should turn right (angle+) to compensate
                    //     newTurretAngle = lockedTurretAngle + Math.toDegrees(deltaHeading);
                    // }
                    
                    // Update turret target
                    robot.turret.enableSoftLock(newTurretAngle);
                    
                    // Press A while seeing target tag → re-lock with TX
                    if (aButton && !lastAButton && canSeeTargetTag) {
                        aimState = AimState.TX_LOCKING;
                        txLockStartTime = System.currentTimeMillis();
                        robot.turret.disableLock();
                    }
                    break;
            }
            
            // ========== ABSOLUTE POSITION UPDATE (any goal tag) ==========
            // Update absolute position when seeing ANY goal tag (20 or 24)
            // This is used for distance estimation in inertial navigation
            if (canSeeAnyGoalTag) {
                double turretAngle = robot.turret.isCalibrated() ? robot.turret.getAngleRadians() : 0;
                robot.drive.updateAbsolutePositionFromVisionWithTurret(robot.vision, turretAngle);
            } else {
                robot.drive.updateAbsolutePositionFromOdometry();
            }
        }
        lastAButton = aButton;
        
        // --- Telemetry ---
        Pose2D pose = robot.drive.getPose();
        
        // === TAG DETECTION ===
        telemetry.addLine("========== VISION ==========");
        if (robot.vision != null) {
            // Debug info - check why tag detection fails
            telemetry.addData("LL Connected", robot.vision.isConnected() ? "YES" : "NO");
            telemetry.addData("LL FPS", String.format("%.0f", robot.vision.getFps()));
            telemetry.addData("Pipeline", robot.vision.getPipelineIndex());
            telemetry.addData("Result Valid", robot.vision.isResultValid() ? "YES" : "NO");
            telemetry.addData("Num Tags", robot.vision.getNumTagsDetected());
            
            int tagId = robot.vision.getDetectedTagId();
            boolean canSeeTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
            telemetry.addData("CAN SEE TAG?", canSeeTag ? "YES ✓" : "NO ✗");
            telemetry.addData("Current Tag", tagId == -1 ? "NONE" : tagId);
            telemetry.addData("TX", String.format("%.1f°", robot.vision.getTx()));
            telemetry.addData("Has Abs Pos", robot.drive.hasAbsolutePosition() ? "YES" : "NO");
            
            if (robot.drive.hasAbsolutePosition()) {
                telemetry.addData("Abs X", String.format("%.1f in", robot.drive.getAbsoluteX()));
                telemetry.addData("Abs Y", String.format("%.1f in", robot.drive.getAbsoluteY()));
                telemetry.addData("Abs Heading", String.format("%.1f°", Math.toDegrees(robot.drive.getAbsoluteHeading())));
            }
        } else {
            telemetry.addLine("Vision NOT AVAILABLE!");
        }
        
        // === TURRET STATUS ===
        if (robot.turret != null) {
            telemetry.addLine("========== TURRET ==========");
            telemetry.addData("Aim State", aimState.toString());
            telemetry.addData("Has Lock Pos", hasLockedPosition ? "YES" : "NO");
            if (aimState == AimState.INERTIAL_TRACKING) {
                double dH = Math.toDegrees(currentOdoHeading - lockedOdoHeading);
                telemetry.addData("Delta Heading", String.format("%.1f°", dH));
            }
            telemetry.addData("Calibrated", robot.turret.isCalibrated() ? "YES" : "NO");
            telemetry.addData("Angle", String.format("%.1f°", robot.turret.getAngleDegrees()));
            telemetry.addData("Target", String.format("%.1f°", robot.turret.getTargetAngle()));
            telemetry.addData("Manual Offset", String.format("%.1f°", robot.turret.getManualOffset()));
            telemetry.addData("Goal", String.format("(%.0f, %.0f)", TurretConstants.redGoalX, TurretConstants.redGoalY));
            telemetry.addData("Disabled", robot.turret.isDisabled() ? "YES!" : "NO");
            telemetry.addLine("GP1 A = Scan | GP2 ←→ = Offset");
        }
        
        telemetry.addLine("========== ODOMETRY ==========");
        telemetry.addData("Odo X", String.format("%.2f in", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Odo Y", String.format("%.2f in", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Odo Heading", String.format("%.1f°", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS))));
        
        // --- Intake/Shooter ---
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
        
        // --- Shooter Status ---
        telemetry.addLine("========== SHOOTER ==========");
        telemetry.addData("READY", robot.shooter.isShooterAtSetPoint());
        telemetry.addData("STATE", robot.shooter.shooterState);
        telemetry.addData("Velocity", String.format("%.0f TPS", robot.shooter.getVelocity()));
        
        // --- Emergency Disable ---
        telemetry.addLine("========== EMERGENCY (GP2) ==========");
        telemetry.addData("Intake", robot.intake.isDisabled() ? "DISABLED" : "OK");
        telemetry.addData("Shooter", robot.shooter.isDisabled() ? "DISABLED" : "OK");
        if (robot.turret != null) {
            telemetry.addData("Turret", robot.turret.isDisabled() ? "DISABLED" : "OK");
        }
        
        telemetry.update();

        // Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        org.firstinspires.ftc.teamcode.utils.DashboardUtil.drawRobot(packet, pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
