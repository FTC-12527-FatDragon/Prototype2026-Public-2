package org.firstinspires.ftc.teamcode.opmodes.teleops;

/**
 * Main TeleOp OpMode - D-Pad Turret Control.
 * Field-centric Mecanum drive with manual turret control via D-Pad.
 * Turret starts at 0° (forward), D-Pad Left/Right adjusts target position.
 * 
 * For TURRET auto-aim (hard lock), use SoloBlue or SoloRed.
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Main TeleOp - Field Centric Driving with D-Pad Turret Control.
 * 
 * Controls:
 * - Left Stick: Move (field-centric)
 * - Right Stick: Rotate
 * - Left Stick Click: Reset heading
 * - D-Pad Left: Turret counter-clockwise (+200 ticks)
 * - D-Pad Right: Turret clockwise (-200 ticks)
 */
@Config
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Solo", group = "TeleOp")
public class Solo extends CommandOpMode {
    private Robot robot;
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;  // Secondary gamepad for emergency controls
    private boolean[] isAuto = {false};
    
    // Edge detection for gamepad2 emergency disable combos
    private boolean lastIntakeDisableCombo = false;   // LT + LB
    private boolean lastShooterDisableCombo = false;  // RT + RB
    private boolean lastTurretDisableCombo = false;   // LB + RB
    
    // Turret home control buttons
    private boolean lastYButton = false;  // Y = go to home
    private boolean lastBButton = false;  // B = set current as home

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);  // Secondary gamepad for emergency controls

        // Register subsystems so their periodic() methods are called
        CommandScheduler.getInstance().registerSubsystem(robot.shooter);
        CommandScheduler.getInstance().registerSubsystem(robot.transit);
        CommandScheduler.getInstance().registerSubsystem(robot.intake);
        if (robot.turret != null) {
            CommandScheduler.getInstance().registerSubsystem(robot.turret);
        }

        // Default drive command (with turret for soft/hard lock aware auto-aim)
        robot.drive.setDefaultCommand(new TeleOpDriveCommand(
                robot.drive,
                robot.vision,
                robot.turret,
                gamepadEx1, 
                isAuto
        ));

        // Left stick button: Reset heading to 0
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> robot.drive.resetHeading())
        );

        DriverControls.bind(gamepadEx1, robot, isAuto);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Initialize turret to SOFT LOCK (0° forward, chassis handles aiming)
        if (robot.turret != null) {
            robot.turret.enableSoftLock();
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        
        // ========== GAMEPAD2 EMERGENCY DISABLE CONTROLS ==========
        // Read current combo states
        boolean intakeDisableCombo = gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3 
                && gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean shooterDisableCombo = gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3 
                && gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean turretDisableCombo = gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER) 
                && gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                && !intakeDisableCombo && !shooterDisableCombo;  // Avoid conflict with other combos
        
        // Edge detection: toggle on rising edge (just pressed)
        if (intakeDisableCombo && !lastIntakeDisableCombo) {
            robot.intake.toggleDisabled();
        }
        if (shooterDisableCombo && !lastShooterDisableCombo) {
            robot.shooter.toggleDisabled();
        }
        if (turretDisableCombo && !lastTurretDisableCombo && robot.turret != null) {
            robot.turret.toggleDisabled();
        }
        
        // Update last combo states for next frame
        lastIntakeDisableCombo = intakeDisableCombo;
        lastShooterDisableCombo = shooterDisableCombo;
        lastTurretDisableCombo = turretDisableCombo;
        
        // ========== D-PAD TURRET MANUAL CONTROL (OPEN LOOP) ==========
        // D-Pad Left: Counter-clockwise at 0.5 power
        // D-Pad Right: Clockwise at 0.5 power
        // Release: Stop (0 power)
        boolean dpadLeft = gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT);
        boolean dpadRight = gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT);
        
        if (robot.turret != null) {
            if (dpadLeft) {
                robot.turret.setPower(0.5);  // Counter-clockwise
            } else if (dpadRight) {
                robot.turret.setPower(-0.5); // Clockwise
            } else {
                robot.turret.setPower(0);    // Stop
            }
        }
        
        // ========== TURRET HOME CONTROL ==========
        // Y button: Go to home position (physical 0°)
        // B button: Set current position as new home
        boolean yButton = gamepadEx1.getButton(GamepadKeys.Button.Y);
        boolean bButton = gamepadEx1.getButton(GamepadKeys.Button.B);
        
        if (yButton && !lastYButton && robot.turret != null) {
            robot.turret.goToHome();
        }
        if (bButton && !lastBButton && robot.turret != null) {
            robot.turret.setCurrentAsHome();
        }
        
        lastYButton = yButton;
        lastBButton = bButton;
        
        // Solo: Turret controlled by D-Pad (Left=CCW +200 ticks, Right=CW -200 ticks)
        // Initial target is 0 ticks (0°), PID drives turret to target
        // Turret.periodic() handles SOFT_LOCK PID control automatically.
        
        // --- Update Absolute Position for Chassis Auto-Aim ---
        // CHASSIS AUTO-AIM DISABLED - Uncomment when ready
        /*
        // ===== ABSOLUTE POSITION UPDATE (for chassis auto-aim) =====
        // In SOFT_LOCK mode, turret is at 0°, so no turret compensation needed
        if (robot.vision != null) {
            int currentTagId = robot.vision.getDetectedTagId();
            boolean isGoalTag = (currentTagId == Vision.BLUE_GOAL_TAG_ID || currentTagId == Vision.RED_GOAL_TAG_ID);
            
            if (isGoalTag) {
                // Turret is locked at 0°, so turretAngle = 0
                robot.drive.updateAbsolutePositionFromVisionWithTurret(robot.vision, 0);
            } else {
                robot.drive.updateAbsolutePositionFromOdometry();
            }
        }
        */
        
        // --- Odometry Pose Telemetry ---
        Pose2D pose = robot.drive.getPose();
        telemetry.addData("Odo X", String.format("%.2f in", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Odo Y", String.format("%.2f in", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Odo Heading", String.format("%.1f deg", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS))));
        
        // CHASSIS AUTO-AIM DISABLED - Uncomment when ready
        /*
        // --- Absolute Field Position (for chassis auto-aim) ---
        telemetry.addLine("=== ABSOLUTE POSITION ===");
        if (robot.drive.hasAbsolutePosition()) {
            telemetry.addData("Abs X", String.format("%.2f in", robot.drive.getAbsoluteX()));
            telemetry.addData("Abs Y", String.format("%.2f in", robot.drive.getAbsoluteY()));
            telemetry.addData("Abs Heading", String.format("%.1f deg", Math.toDegrees(robot.drive.getAbsoluteHeading())));
        } else {
            telemetry.addData("Abs Position", "NOT INITIALIZED (need to see tag 20/24)");
        }
        
        // --- Vision Status ---
        if (robot.vision != null) {
            int currentTagId = robot.vision.getDetectedTagId();
            boolean isGoalTag = (currentTagId == Vision.BLUE_GOAL_TAG_ID || currentTagId == Vision.RED_GOAL_TAG_ID);
            
            telemetry.addLine("=== VISION ===");
            telemetry.addData("Current Tag", currentTagId != -1 ? currentTagId : "None");
            if (isGoalTag) {
                telemetry.addData("tx", String.format("%.2f°", robot.vision.getTx()));
                telemetry.addData("Distance", String.format("%.2f in", robot.vision.getDistanceToTag()));
            }
        }
        
        // --- Chassis Auto-Aim Status ---
        // Note: Actual auto-aim is in TeleOpDriveCommand, triggered by A button or shoot buttons
        telemetry.addLine("=== CHASSIS AUTO-AIM ===");
        telemetry.addData("Mode", "SOFT LOCK (chassis aims, turret fixed 0°)");
        telemetry.addData("Aligned", robot.drive.isAligned() ? "YES" : "NO");
        */
        
        // --- Intake/Shooter Status ---
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
        
        // --- Turret Status (Open Loop Control) ---
        if (robot.turret != null) {
            telemetry.addLine("=== TURRET (OPEN LOOP) ===");
            telemetry.addData("Current Deg", String.format("%.1f°", robot.turret.getAngleDegrees()));
            telemetry.addData("Raw Encoder", robot.turret.getRawEncoderPosition());
            telemetry.addData("Home Encoder", robot.turret.getHomeEncoderPosition());
            telemetry.addLine("D-Pad: L=CCW(0.5) R=CW(-0.5) | Y=Home | B=SetHome");
        }
        
        // --- Emergency Disable Status (Gamepad2) ---
        telemetry.addLine("=== EMERGENCY DISABLE (GP2) ===");
        telemetry.addData("Intake", robot.intake.isDisabled() ? "DISABLED" : "OK");
        telemetry.addData("Shooter", robot.shooter.isDisabled() ? "DISABLED" : "OK");
        if (robot.turret != null) {
            telemetry.addData("Turret", robot.turret.isDisabled() ? "DISABLED" : "OK");
        }
        telemetry.addLine("GP2: LT+LB=Intake | RT+RB=Shooter | LB+RB=Turret");
        
        // --- Shooter Status ---
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("READY TO SHOOT", robot.shooter.isShooterAtSetPoint());
        telemetry.addData("SHOOTER STATE", robot.shooter.shooterState);
        telemetry.addData("Current Velocity", String.format("%.0f TPS", robot.shooter.getVelocity()));
        
        telemetry.update();

        // Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        org.firstinspires.ftc.teamcode.utils.DashboardUtil.drawRobot(packet, pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
