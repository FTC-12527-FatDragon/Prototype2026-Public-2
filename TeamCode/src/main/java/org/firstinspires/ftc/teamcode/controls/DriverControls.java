package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

/**
 * Driver Controls Configuration.
 * Centralizes the gamepad button bindings for the robot.
 */
public class DriverControls {

    /**
     * Binds the gamepad controls to the robot commands.
     *
     * @param gamepad The gamepad wrapper (GamepadEx).
     * @param robot The robot hardware container.
     * @param isAuto Flag array to disable chassis when adaptive shooting is active.
     */
    public static void bind(GamepadEx gamepad, Robot robot, boolean[] isAuto) {
        // Reset Field Centric Heading (Left Stick Button)
        // Note: For RobotCentric drive, this just resets odometry, which is harmless.
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> robot.drive.reset(0))
        );

        // Slow Shoot (Left Bumper - Close Shot)
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.SLOW))
        ).whenReleased(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        // Mid Shoot (Right Bumper - Mid Shot)
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.MID))
        ).whenReleased(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        // Fast Shoot (Right Trigger - Far Shot)
        new FunctionalButton(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= TeleOpConstants.slowShootTriggerThreshold
        ).whenHeld(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.FAST))
        ).whenReleased(
                new InstantCommand(() -> robot.shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        // Transit Fire (Left Trigger + Shoot Button)
        // Only fires when BOTH left trigger AND a shoot button (LB/RB/RT) are pressed
        new FunctionalButton(
                () -> {
                    boolean leftTriggerPressed = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= TeleOpConstants.transitFireTriggerThreshold;
                    boolean shootButtonPressed = gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) ||
                                                  gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) ||
                                                  gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= TeleOpConstants.slowShootTriggerThreshold;
                    return leftTriggerPressed && shootButtonPressed;
                }
        ).whenHeld(
                new TransitCommand(robot.transit, robot.shooter)
        );

        // Reverse Intake (D-Pad Up)
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_UP)
        ).whenHeld(
                new InstantCommand(() -> robot.intake.setReversed(true))
        ).whenReleased(
                new InstantCommand(() -> robot.intake.setReversed(false))
        );

        // Intake Full Power (Left Trigger > Threshold)
        new FunctionalButton(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= TeleOpConstants.intakeFullPowerTriggerThreshold
        ).whenHeld(
                new InstantCommand(() -> robot.intake.setFullPower(true))
        ).whenReleased(
                new InstantCommand(() -> robot.intake.setFullPower(false))
        );
        
        // ==================== TURRET LOCK MODE (Right Stick Button) ====================
        // TURRET DISABLED
        /*
        // Toggle between Soft Lock and Hard Lock
        // - Soft Lock: Hold turret at 0° (forward), chassis handles aiming
        // - Hard Lock: Turret tracks goal (TX when seeing own tag, inertial otherwise)
        // Initial state: Soft Lock (set in TeleOp.initialize())
        // 
        // NOTE: Alliance is set by the OpMode (SoloBlue/SoloRed), NOT by which tag is seen!
        // The turret will always aim at the alliance goal set by setAlliance().
        // TX tracking only activates when seeing YOUR alliance's tag.
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> {
                    // Cannot change modes during unwind
                    if (robot.turret.isUnwinding()) {
                        return;
                    }
                    
                    Turret.LockMode currentMode = robot.turret.getLockMode();
                    
                    if (currentMode == Turret.LockMode.SOFT_LOCK) {
                        // Switch to Hard Lock (uses alliance set by OpMode)
                        // No need to check tag - turret uses inertial if no tag visible
                        robot.turret.enableHardLock();
                        
                    } else if (currentMode == Turret.LockMode.HARD_LOCK) {
                        // Switch back to Soft Lock
                        robot.turret.enableSoftLock();
                    } else {
                        // If in MANUAL mode, enable Soft Lock
                        robot.turret.enableSoftLock();
                    }
                })
        );
        */
    }
}


// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
