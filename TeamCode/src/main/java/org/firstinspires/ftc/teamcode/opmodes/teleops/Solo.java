package org.firstinspires.ftc.teamcode.opmodes.teleops;

/**
 * Main TeleOp OpMode - D-Pad Turret Open-Loop Control.
 * Field-centric Mecanum drive with manual turret control via D-Pad.
 * Turret is in MANUAL mode (open-loop), D-Pad controls motor power directly.
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
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Main TeleOp - Field Centric Driving with D-Pad Turret Open-Loop Control.
 * 
 * Controls:
 * - Left Stick: Move (field-centric)
 * - Right Stick: Rotate
 * - Left Stick Click: Reset heading
 * - D-Pad Left: Turret left (CCW) at 0.5 power
 * - D-Pad Right: Turret right (CW) at 0.5 power
 * - Release D-Pad: Turret stops (BRAKE mode holds position)
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
        
        // Initialize turret to MANUAL mode (open-loop control)
        // Motor is already in BRAKE mode, so it holds position when power = 0
        if (robot.turret != null) {
            robot.turret.disableLock();  // MANUAL mode
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
        
        // ========== D-PAD TURRET OPEN-LOOP CONTROL ==========
        // D-Pad Left: Turn left (CCW) at 0.5 power
        // D-Pad Right: Turn right (CW) at 0.5 power
        // Release: Active position hold using PID (stronger than passive BRAKE)
        if (robot.turret != null) {
            boolean dpadLeft = gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT);
            boolean dpadRight = gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT);
            
            if (dpadLeft) {
                robot.turret.releaseHold();   // Allow manual control
                robot.turret.setPower(1.0);   // Left (angle decreases)
            } else if (dpadRight) {
                robot.turret.releaseHold();   // Allow manual control
                robot.turret.setPower(-1.0);  // Right (angle increases)
            } else {
                // No D-pad input: actively hold current position with PID
                if (!robot.turret.isHoldingPosition()) {
                    robot.turret.holdCurrentPosition();  // Lock to current angle
                }
            }
        }
        
        // --- Odometry Pose Telemetry ---
        Pose2D pose = robot.drive.getPose();
        telemetry.addData("Odo X", String.format("%.2f in", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Odo Y", String.format("%.2f in", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Odo Heading", String.format("%.1f deg", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS))));
        
        // --- Vision / Auto-Aim Status ---
        telemetry.addLine("=== VISION ===");
        if (robot.vision != null) {
            int tagId = robot.vision.getDetectedTagId();
            telemetry.addData("Tag ID", tagId == -1 ? "NONE" : tagId);
            telemetry.addData("TX", String.format("%.1f°", robot.vision.getTx()));
            telemetry.addData("Distance", String.format("%.1f in", robot.vision.getDistanceToTag()));
            telemetry.addData("Aligned", robot.drive.isAligned() ? "YES ✓" : "NO");
        } else {
            telemetry.addLine("Vision not available");
        }
        telemetry.addLine("A = Chassis Auto-Aim");
        
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
        
        // --- Turret Status (Open Loop) ---
        if (robot.turret != null) {
            telemetry.addLine("=== TURRET (MANUAL) ===");
            telemetry.addData("Angle", String.format("%.1f°", robot.turret.getAngleDegrees()));
            telemetry.addData("Mode", robot.turret.getLockMode());
            telemetry.addLine("D-Pad L=Left(0.5) | R=Right(-0.5)");
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
