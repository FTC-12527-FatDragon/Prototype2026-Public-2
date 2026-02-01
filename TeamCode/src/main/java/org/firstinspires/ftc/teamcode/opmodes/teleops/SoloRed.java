package org.firstinspires.ftc.teamcode.opmodes.teleops;

/**
 * TeleOp for RED Alliance.
 * Turret will aim at RED basket (140, 140) in HARD_LOCK mode.
 * Can see either tag 20 (blue) or 24 (red) for positioning, but always aims at red basket.
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
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Solo Red", group = "TeleOp")
public class SoloRed extends CommandOpMode {
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

        // Register subsystems
        CommandScheduler.getInstance().registerSubsystem(robot.shooter);
        CommandScheduler.getInstance().registerSubsystem(robot.transit);
        CommandScheduler.getInstance().registerSubsystem(robot.intake);
        if (robot.turret != null) {
            CommandScheduler.getInstance().registerSubsystem(robot.turret);
            // ========== SET ALLIANCE TO RED ==========
            robot.turret.setAlliance(Turret.Alliance.RED);
        }

        // Default drive command
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
        
        // Initialize turret to SOFT LOCK
        if (robot.turret != null) {
            robot.turret.enableSoftLock();
        }
        
        telemetry.addLine("=== SOLO RED ===");
        telemetry.addLine("Aiming at RED basket (140, 140)");
        telemetry.update();
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
        
        // --- Update Absolute Position & Turret ---
        // TURRET/VISION DISABLED - Uncomment when ready
        /*
        // ===== ABSOLUTE POSITION UPDATE (with Turret Compensation) =====
        if (robot.vision != null) {
            int currentTagId = robot.vision.getDetectedTagId();
            // Can use EITHER tag 20 or 24 for positioning!
            boolean isGoalTag = (currentTagId == Vision.BLUE_GOAL_TAG_ID || currentTagId == Vision.RED_GOAL_TAG_ID);
            
            if (isGoalTag) {
                double turretAngle = (robot.turret != null && robot.turret.isCalibrated()) 
                    ? robot.turret.getAngleRadians() : 0;
                robot.drive.updateAbsolutePositionFromVisionWithTurret(robot.vision, turretAngle);
            } else {
                robot.drive.updateAbsolutePositionFromOdometry();
            }
        }
        
        // ===== TURRET UPDATE =====
        if (robot.turret != null && robot.vision != null) {
            int currentTagId = robot.vision.getDetectedTagId();
            boolean isGoalTag = (currentTagId == Vision.BLUE_GOAL_TAG_ID || currentTagId == Vision.RED_GOAL_TAG_ID);
            
            // Update turret with tx AND tagId (for HARD_LOCK tx tracking)
            // TX tracking only activates when seeing RED tag (our target)
            // If seeing BLUE tag, we still get position but use inertial to aim at red
            if (isGoalTag) {
                robot.turret.updateTx(robot.vision.getTx(), true, currentTagId);
            } else {
                robot.turret.updateTx(0, false, -1);
            }
            
            // Update turret with robot position (for HARD_LOCK inertial mode)
            if (robot.drive.hasAbsolutePosition()) {
                robot.turret.updateRobotPosition(
                        robot.drive.getAbsoluteX(),
                        robot.drive.getAbsoluteY(),
                        robot.drive.getAbsoluteHeading()
                );
            }
        }
        */
        
        // --- Telemetry ---
        Pose2D pose = robot.drive.getPose();
        telemetry.addLine("=== SOLO RED ===");
        telemetry.addData("Target", "RED basket (140, 140)");
        telemetry.addData("Odo X", String.format("%.2f in", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Odo Y", String.format("%.2f in", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Odo Heading", String.format("%.1f deg", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS))));
        
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
        
        // --- Shooter Status ---
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("READY TO SHOOT", robot.shooter.isShooterAtSetPoint());
        telemetry.addData("SHOOTER STATE", robot.shooter.shooterState);
        telemetry.addData("Current Velocity", String.format("%.0f TPS", robot.shooter.getVelocity()));
        
        // --- Turret Status ---
        if (robot.turret != null) {
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Alliance", robot.turret.getAlliance());
            telemetry.addData("Lock Mode", robot.turret.getLockMode());
            telemetry.addData("Unwinding", robot.turret.isUnwinding() ? "YES" : "NO");
        }
        
        // --- Emergency Disable Status (Gamepad2) ---
        telemetry.addLine("=== EMERGENCY DISABLE (GP2) ===");
        telemetry.addData("Intake", robot.intake.isDisabled() ? "DISABLED" : "OK");
        telemetry.addData("Shooter", robot.shooter.isDisabled() ? "DISABLED" : "OK");
        if (robot.turret != null) {
            telemetry.addData("Turret", robot.turret.isDisabled() ? "DISABLED" : "OK");
        }
        telemetry.addLine("GP2: LT+LB=Intake | RT+RB=Shooter | LB+RB=Turret");
        
        telemetry.update();

        // Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        org.firstinspires.ftc.teamcode.utils.DashboardUtil.drawRobot(packet, pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
