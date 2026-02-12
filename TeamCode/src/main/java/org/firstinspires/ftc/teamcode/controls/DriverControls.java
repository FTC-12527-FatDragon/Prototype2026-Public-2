package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

/**
 * Driver Controls Configuration.
 * Centralizes gamepad button bindings for shooter, transit, and intake.
 *
 * Turret controls are handled directly in each TeleOp (Solo/SoloRed/SoloBlue)
 * because different OpModes have different turret control schemes.
 */
public class DriverControls {

    /**
     * Binds gamepad controls to robot commands.
     *
     * @param gamepad The gamepad wrapper.
     * @param robot   The robot hardware container.
     * @param isAuto  Flag array to disable chassis when adaptive shooting is active.
     */
    public static void bind(GamepadEx gamepad, Robot robot, boolean[] isAuto) {

        // ── Heading Reset (Left Stick Button) ──
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> robot.drive.reset(0))
        );

        // ── Slow Shot (Left Bumper) ──
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.SLOW))
        ).whenReleased(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        // ── Mid Shot (Right Bumper) ──
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.MID))
        ).whenReleased(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        // ── Far Shot (Right Trigger) ──
        new FunctionalButton(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= TeleOpConstants.slowShootTriggerThreshold
        ).whenHeld(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.FAST))
        ).whenReleased(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        // ── Transit Fire (Left Trigger + any Shoot Button) ──
        new FunctionalButton(
                () -> {
                    boolean leftTrigger = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                            >= TeleOpConstants.transitFireTriggerThreshold;
                    boolean shootButton = gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)
                            || gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                            || gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                               >= TeleOpConstants.slowShootTriggerThreshold;
                    return leftTrigger && shootButton;
                }
        ).whenHeld(
                new TransitCommand(robot.transit, robot.shooter)
        );

        // ── Reverse Intake (D-Pad Up) ──
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_UP)
        ).whenHeld(
                new InstantCommand(() -> robot.intake.setReversed(true))
        ).whenReleased(
                new InstantCommand(() -> robot.intake.setReversed(false))
        );

        // ── Full-Power Intake (Left Trigger) ──
        new FunctionalButton(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                        >= TeleOpConstants.intakeFullPowerTriggerThreshold
        ).whenHeld(
                new InstantCommand(() -> robot.intake.setFullPower(true))
        ).whenReleased(
                new InstantCommand(() -> robot.intake.setFullPower(false))
        );
    }
}
