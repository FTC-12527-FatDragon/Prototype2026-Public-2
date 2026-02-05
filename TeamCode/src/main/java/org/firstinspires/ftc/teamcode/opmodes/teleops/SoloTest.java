package org.firstinspires.ftc.teamcode.opmodes.teleops;

/**
 * Test TeleOp - HARD_LOCK TX Tracking Mode Test
 * Turret automatically tracks target using Limelight TX value
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
    
    // Edge detection for GP2 right stick (set home)
    private boolean lastGP2RightStickButton = false;
    
    // Lock to first seen tag (20 or 24)
    private int lockedTagId = -1;  // -1 means not locked yet
    
    // HARD_LOCK mode: Turret auto-tracks goal using TX + inertial navigation

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // Register subsystems
        CommandScheduler.getInstance().registerSubsystem(robot.shooter);
        CommandScheduler.getInstance().registerSubsystem(robot.transit);
        CommandScheduler.getInstance().registerSubsystem(robot.intake);
        if (robot.turret != null) {
            CommandScheduler.getInstance().registerSubsystem(robot.turret);
            // Set alliance to RED (only tag 24 supported)
            robot.turret.setAlliance(Turret.Alliance.RED);
        }

        // NO default drive command - we manually control chassis with auto TX tracking

        // Left stick button: Reset heading to 0
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> robot.drive.resetHeading())
        );

        DriverControls.bind(gamepadEx1, robot, isAuto);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // ========== HARD_LOCK MODE ==========
        // Turret automatically tracks goal using TX + inertial navigation
        if (robot.turret != null) {
            robot.turret.enableHardLock();  // Enable automatic goal tracking
        }
        
        telemetry.addLine("=== SOLO TEST (HARD_LOCK) ===");
        telemetry.addLine("Auto: TX + Inertial Navigation");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        
        // ========== GAMEPAD2 RIGHT STICK: SET CURRENT AS HOME ==========
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
        
        // ========== TURRET HARD_LOCK MODE ==========
        // Update robot position and TX for Turret's automatic tracking
        if (robot.turret != null && robot.vision != null) {
            // Get ABSOLUTE position (calibrated, not raw odometry)
            // This is essential for inertial navigation to work!
            double robotX, robotY, robotHeading;
            if (robot.drive.hasAbsolutePosition()) {
                robotX = robot.drive.getAbsoluteX();
                robotY = robot.drive.getAbsoluteY();
                robotHeading = robot.drive.getAbsoluteHeading();
            } else {
                // Fallback to raw odometry if not calibrated yet
                Pose2D pose = robot.drive.getPose();
                robotX = pose.getX(DistanceUnit.INCH);
                robotY = pose.getY(DistanceUnit.INCH);
                robotHeading = pose.getHeading(AngleUnit.RADIANS);
            }
            
            // Update Turret with robot position (for inertial navigation)
            robot.turret.updateRobotPosition(robotX, robotY, robotHeading);
            
            // Update Turret with TX (for visual tracking when tag visible)
            int currentTagId = robot.vision.getDetectedTagId();
            double tx = robot.vision.getTx();
            boolean hasValidTx = (currentTagId != -1);
            robot.turret.updateTx(tx, hasValidTx, currentTagId);
            
            // Lock to first seen goal tag (for telemetry display)
            boolean isGoalTag = (currentTagId == Vision.BLUE_GOAL_TAG_ID || currentTagId == Vision.RED_GOAL_TAG_ID);
            if (lockedTagId == -1 && isGoalTag) {
                lockedTagId = currentTagId;
            }
            
            // DEBUG: Show tracking info
            telemetry.addLine("=== TURRET DEBUG ===");
            telemetry.addData("DISABLED?", robot.turret.isDisabled() ? "YES!!!" : "NO");
            telemetry.addData("Lock Mode", robot.turret.getLockMode());
            telemetry.addData("Is Calibrated", robot.turret.isCalibrated() ? "YES" : "NO");
            telemetry.addData("Has Abs Pos", robot.drive.hasAbsolutePosition() ? "YES" : "NO");
            telemetry.addData("Tracking", robot.turret.getTrackingModeString());
            telemetry.addData("TX Active", robot.turret.isTxTrackingActive() ? "YES" : "NO");
            telemetry.addData("TX", String.format("%.1f°", tx));
            telemetry.addData("Tag ID", currentTagId);
            telemetry.addData("Target Angle", String.format("%.1f°", robot.turret.getTargetAngle()));
            telemetry.addData("Current Angle", String.format("%.1f°", robot.turret.getAngleDegrees()));
            telemetry.addData("Calc Angle", String.format("%.1f°", robot.turret.calculateAngleToGoal()));
            telemetry.addData("Robot Pos", String.format("(%.1f, %.1f)", robotX, robotY));
            telemetry.addData("Goal Pos", String.format("(%.1f, %.1f)", 
                TurretConstants.redGoalX, TurretConstants.redGoalY));
        }
        
        // ========== ABSOLUTE POSITION UPDATE ==========
        if (robot.vision != null) {
            int currentTagId = robot.vision.getDetectedTagId();
            boolean isGoalTag = (currentTagId == Vision.BLUE_GOAL_TAG_ID || currentTagId == Vision.RED_GOAL_TAG_ID);
            
            if (isGoalTag) {
                double turretAngle = (robot.turret != null && robot.turret.isCalibrated()) 
                    ? robot.turret.getAngleRadians() : 0;
                robot.drive.updateAbsolutePositionFromVisionWithTurret(robot.vision, turretAngle);
            } else {
                robot.drive.updateAbsolutePositionFromOdometry();
            }
        }
        
        // --- Telemetry ---
        Pose2D pose = robot.drive.getPose();
        
        // === TAG DETECTION (FIRST!) ===
        telemetry.addLine("========== TAG ==========");
        if (robot.vision != null) {
            int tagId = robot.vision.getDetectedTagId();
            boolean canSeeTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
            telemetry.addData("CAN SEE TAG?", canSeeTag ? "YES ✓" : "NO ✗");
            telemetry.addData("Current Tag", tagId == -1 ? "NONE" : tagId);
            telemetry.addData("LOCKED Tag", lockedTagId == -1 ? "WAITING..." : lockedTagId);
            telemetry.addData("TX", String.format("%.1f°", robot.vision.getTx()));
            
            // Show absolute position when seeing tag
            if (canSeeTag && robot.drive.hasAbsolutePosition()) {
                telemetry.addLine("--- ABSOLUTE POS ---");
                telemetry.addData("Abs X", String.format("%.2f in", robot.drive.getAbsoluteX()));
                telemetry.addData("Abs Y", String.format("%.2f in", robot.drive.getAbsoluteY()));
                telemetry.addData("Abs Heading", String.format("%.1f°", Math.toDegrees(robot.drive.getAbsoluteHeading())));
            }
        } else {
            telemetry.addLine("Vision NOT AVAILABLE!");
        }
        telemetry.addLine("");
        
        telemetry.addLine("=== SOLO TEST (HARD_LOCK TX) ===");
        telemetry.addData("Odo X", String.format("%.2f in", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Odo Y", String.format("%.2f in", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Odo Heading", String.format("%.1f°", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS))));
        
        // --- Turret Status ---
        if (robot.turret != null) {
            telemetry.addLine("=== TURRET (HARD_LOCK) ===");
            telemetry.addData("Angle", String.format("%.1f°", robot.turret.getAngleDegrees()));
            telemetry.addData("Mode", robot.turret.getLockMode());
            telemetry.addData("Tracking", robot.turret.getTrackingModeString());
            telemetry.addData("TX Active", robot.turret.isTxTrackingActive() ? "YES" : "NO");
            telemetry.addData("Unwinding", robot.turret.isUnwinding() ? "YES" : "NO");
            telemetry.addLine("GP2 RS = Set Home");
        }
        
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
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("READY", robot.shooter.isShooterAtSetPoint());
        telemetry.addData("STATE", robot.shooter.shooterState);
        telemetry.addData("Velocity", String.format("%.0f TPS", robot.shooter.getVelocity()));
        
        // --- Emergency Disable ---
        telemetry.addLine("=== EMERGENCY (GP2) ===");
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
