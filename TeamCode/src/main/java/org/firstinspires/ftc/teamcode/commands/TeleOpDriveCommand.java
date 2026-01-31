package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivePinpoint;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/**
 * Command for TeleOp field-centric driving.
 * Manual control only - auto-aim moved to turret.
 */
public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDrivePinpoint drive;
    private final Vision vision;
    private final GamepadEx gamepadEx;
    private final boolean[] isAuto;

    public TeleOpDriveCommand(MecanumDrivePinpoint drive, Vision vision, GamepadEx gamepadEx, 
                              boolean[] isAuto, java.util.function.BooleanSupplier unused) {
        this.drive = drive;
        this.vision = vision;
        this.gamepadEx = gamepadEx;
        this.isAuto = isAuto;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // ==================== ABSOLUTE POSITION UPDATE ====================
        // Update absolute field coordinates every frame
        int tagId = vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        if (isGoalTag) {
            drive.updateAbsolutePositionFromVision(vision);
        } else {
            drive.updateAbsolutePositionFromOdometry();
        }
        
        // ==================== MANUAL DRIVING ====================
        if (!isAuto[0]) {
            // Get raw inputs
            double rawLeftX = gamepadEx.getLeftX();
            double rawLeftY = gamepadEx.getLeftY();
            double rawRightX = gamepadEx.getRightX();
            
            // D-Pad rotation input
            double dpadTurn = 0;
            if (gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                dpadTurn = -DriveConstants.dpadTurnSpeed;
            } else if (gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                dpadTurn = DriveConstants.dpadTurnSpeed;
            }
            
            // Check for input
            boolean hasInput = Math.abs(rawLeftX) > DriveConstants.deadband || 
                               Math.abs(rawLeftY) > DriveConstants.deadband || 
                               Math.abs(rawRightX) > DriveConstants.deadband ||
                               dpadTurn != 0;
            
            if (hasInput) {
                drive.setGamepad(true);
                
                // Apply squared input curve
                double forward = rawLeftY * Math.abs(rawLeftY);
                double strafe = -rawLeftX * Math.abs(rawLeftX);
                double turn = rawRightX * Math.abs(rawRightX) + dpadTurn;
                
                // Clamp turn to [-1, 1]
                turn = Math.max(-1, Math.min(1, turn));
                
                // Drive Field Relative
                drive.moveRobotFieldRelative(forward, strafe, turn);
            } else {
                drive.setGamepad(false);
            }
        }
    }
}
