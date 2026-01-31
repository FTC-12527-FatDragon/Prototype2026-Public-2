package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

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
        // VISION/LIMELIGHT DISABLED
        /*
        // ==================== ABSOLUTE POSITION UPDATE ====================
        // Update absolute field coordinates every frame
        int tagId = vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        if (isGoalTag) {
            drive.updateAbsolutePositionFromVision(vision);
        } else {
            drive.updateAbsolutePositionFromOdometry();
        }
        */
        
        // ==================== MANUAL DRIVING ====================
        if (!isAuto[0]) {
            // Get raw inputs (same as DriveOnlyTeleOp)
            double rawLeftX = -gamepadEx.getLeftX();  // Negate X
            double rawLeftY = gamepadEx.getLeftY();
            double rawRightX = gamepadEx.getRightX();
            
            // Check for input
            boolean hasInput = Math.abs(rawLeftX) > DriveConstants.deadband || 
                               Math.abs(rawLeftY) > DriveConstants.deadband || 
                               Math.abs(rawRightX) > DriveConstants.deadband;
            
            if (hasInput) {
                drive.setGamepad(true);
                
                // Apply squared input curve (same as DriveOnlyTeleOp)
                double forward = rawLeftY * Math.abs(rawLeftY);
                double strafe = rawLeftX * Math.abs(rawLeftX);  // No negation here
                double turn = rawRightX * Math.abs(rawRightX);
                
                // Drive Field Relative
                drive.moveRobotFieldRelative(forward, strafe, turn);
            } else {
                drive.setGamepad(false);
            }
        }
    }
}
