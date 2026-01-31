package org.firstinspires.ftc.teamcode.opmodes.teleops;

/**
 * Main TeleOp OpMode.
 * Field-centric Mecanum drive with adaptive shooting and comprehensive telemetry.
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
 * Main TeleOp - Field Centric Driving.
 * 
 * Controls:
 * - Left Stick: Move (field-centric)
 * - Right Stick: Rotate
 * - Left Stick Click: Reset heading
 */
@Config
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Solo", group = "TeleOp")
public class TeleOp extends CommandOpMode {
    private Robot robot;
    private GamepadEx gamepadEx1;
    private boolean[] isAuto = {false};

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        // Default drive command
        robot.drive.setDefaultCommand(new TeleOpDriveCommand(
                robot.drive,
                robot.vision,
                gamepadEx1, 
                isAuto,
                () -> false  // unused parameter kept for compatibility
        ));

        // Left stick button: Reset heading to 0
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> robot.drive.resetHeading())
        );

        DriverControls.bind(gamepadEx1, robot, isAuto);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // TURRET DISABLED
        // robot.turret.enableSoftLock();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        
        // TURRET DISABLED
        /*
        // --- Update Turret with Robot Position (for Hard Lock) ---
        if (robot.drive.hasAbsolutePosition()) {
            robot.turret.updateRobotPosition(
                    robot.drive.getAbsoluteX(),
                    robot.drive.getAbsoluteY(),
                    robot.drive.getAbsoluteHeading()
            );
        }
        */
        
        // --- Odometry Pose Telemetry ---
        Pose2D pose = robot.drive.getPose();
        telemetry.addData("Odo X", String.format("%.2f in", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Odo Y", String.format("%.2f in", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Odo Heading", String.format("%.1f deg", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS))));
        
        // --- Absolute Field Position (Fused: Vision + Odometry) ---
        telemetry.addLine("=== ABSOLUTE POSITION ===");
        if (robot.drive.hasAbsolutePosition()) {
            telemetry.addData("Abs X", String.format("%.2f in", robot.drive.getAbsoluteX()));
            telemetry.addData("Abs Y", String.format("%.2f in", robot.drive.getAbsoluteY()));
            telemetry.addData("Abs Heading", String.format("%.1f deg", Math.toDegrees(robot.drive.getAbsoluteHeading())));
        } else {
            telemetry.addData("Abs Position", "NOT INITIALIZED (need to see tag 20/24)");
        }
        
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
        
        int currentTagId = robot.vision.getDetectedTagId();
        boolean isGoalTag = (currentTagId == Vision.BLUE_GOAL_TAG_ID || currentTagId == Vision.RED_GOAL_TAG_ID);
        
        telemetry.addLine("=== VISION ===");
        telemetry.addData("Current Tag", currentTagId != -1 ? currentTagId : "None");
        if (isGoalTag) {
            telemetry.addData("tx", String.format("%.2f°", robot.vision.getTx()));
        }
        
        // --- Vision Robot Pose (when seeing goal tag 20 or 24) ---
        if (isGoalTag) {
            Pose3D visionPose = robot.vision.getRobotPose();
            double distance = robot.vision.getDistanceToTag();
            
            telemetry.addLine("=== VISION POSE ===");
            if (visionPose != null) {
                // Pose3D position is in meters, convert to inches
                double visionX = visionPose.getPosition().x * 39.3701;
                double visionY = visionPose.getPosition().y * 39.3701;
                double visionHeading = visionPose.getOrientation().getYaw(AngleUnit.DEGREES);
                
                telemetry.addData("Vision X", String.format("%.2f in", visionX));
                telemetry.addData("Vision Y", String.format("%.2f in", visionY));
                telemetry.addData("Vision Heading", String.format("%.1f deg", visionHeading));
                telemetry.addData("Distance to Tag", String.format("%.2f in", distance));
            } else {
                telemetry.addData("Vision Pose", "NULL (tag detected but no pose)");
            }
        }
        
        // --- Adaptive Shooting ---
        if (isGoalTag && robot.drive.hasAbsolutePosition()) {
            telemetry.addLine("=== ADAPTIVE SHOOTING ===");
            
            double distToGoal = robot.drive.distanceToGoal(currentTagId);
            double adaptiveVelocity = robot.drive.calculateAdaptiveVelocity(currentTagId);
            String segment = robot.drive.getAdaptiveSegment(currentTagId);
            double tx = robot.vision.getTx();
            boolean canFire = robot.drive.isAutoFireAllowed(tx);
            
            telemetry.addData("Distance to Goal", String.format("%.2f in", distToGoal));
            telemetry.addData("Segment", segment);
            telemetry.addData("Adaptive Velocity", String.format("%.0f TPS", adaptiveVelocity));
            telemetry.addData("tx", String.format("%.2f°", tx));
            telemetry.addData("CAN FIRE", canFire ? "YES (|tx| < 0.3°)" : "NO (align first)");
        }
        
        // TURRET DISABLED
        /*
        // --- Turret Status ---
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Lock Mode", robot.turret.getLockMode());
        telemetry.addData("Angle", String.format("%.1f°", robot.turret.getAngleDegrees()));
        telemetry.addData("Target", String.format("%.1f°", robot.turret.getTargetAngle()));
        if (robot.turret.getLockMode() == org.firstinspires.ftc.teamcode.subsystems.turret.Turret.LockMode.HARD_LOCK) {
            telemetry.addData("On Target", robot.turret.isOnTarget() ? "YES" : "NO");
            telemetry.addData("Dist to Goal", String.format("%.1f in", robot.turret.getDistanceToGoal()));
        }
        */
        
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
