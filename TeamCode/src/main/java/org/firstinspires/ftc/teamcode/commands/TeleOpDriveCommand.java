package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivePinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/**
 * Command for TeleOp driving with integrated turret-aware auto-aim.
 * 
 * SOFT LOCK mode: Chassis auto-aim enabled (turret fixed at 0°)
 * HARD LOCK mode: Chassis manual only (turret handles tracking)
 * 
 * Auto-aim trigger: A button ONLY (not shoot buttons)
 */
public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDrivePinpoint drive;
    private final Vision vision;
    private final Turret turret;
    private final GamepadEx gamepadEx;
    private final boolean[] isAuto;
    
    // Trigger threshold for shoot buttons
    private static final double TRIGGER_THRESHOLD = 0.3;

    public TeleOpDriveCommand(MecanumDrivePinpoint drive, Vision vision, Turret turret,
                              GamepadEx gamepadEx, boolean[] isAuto) {
        this.drive = drive;
        this.vision = vision;
        this.turret = turret;
        this.gamepadEx = gamepadEx;
        this.isAuto = isAuto;
        addRequirements(drive);
    }
    
    // Legacy constructor for compatibility
    public TeleOpDriveCommand(MecanumDrivePinpoint drive, Vision vision, GamepadEx gamepadEx, 
                              boolean[] isAuto, java.util.function.BooleanSupplier unused) {
        this(drive, vision, null, gamepadEx, isAuto);
    }
    
    /**
     * Checks if any shoot button (with auto-aim) is pressed.
     * LB = Slow, RB = Mid, RT = Fast
     */
    private boolean isShootButtonPressed() {
        return gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER) ||
               gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER) ||
               gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= TRIGGER_THRESHOLD;
    }

    @Override
    public void execute() {
        // ==================== ABSOLUTE POSITION UPDATE (with Turret Compensation) ====================
        // Limelight is on the turret, so we need to compensate for turret angle
        // to get the correct chassis center position
        if (vision != null) {
            int tagId = vision.getDetectedTagId();
            boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
            
            if (isGoalTag) {
                // Vision update - with turret angle compensation
                // Get turret angle (default to 0 if turret not available)
                double turretAngle = (turret != null && turret.isCalibrated()) 
                    ? turret.getAngleRadians() 
                    : 0;
                drive.updateAbsolutePositionFromVisionWithTurret(vision, turretAngle);
            } else {
                // Odometry dead-reckoning when no goal tag
                drive.updateAbsolutePositionFromOdometry();
            }
        }
        
        // ==================== DRIVING WITH AUTO-AIM ====================
        if (!isAuto[0]) {
            // Get raw inputs
            double rawLeftX = -gamepadEx.getLeftX();  // Negate X for correct strafing direction
            double rawLeftY = gamepadEx.getLeftY();
            double rawRightX = gamepadEx.getRightX();
            
            // Check button states
            boolean aPressed = gamepadEx.getButton(GamepadKeys.Button.A);
            
            // Auto-aim trigger: A button ONLY
            // Shoot buttons do NOT trigger chassis auto-aim
            boolean shouldAlign = aPressed;
            
            // Check for input
            boolean hasInput = Math.abs(rawLeftX) > DriveConstants.deadband || 
                               Math.abs(rawLeftY) > DriveConstants.deadband || 
                               Math.abs(rawRightX) > DriveConstants.deadband ||
                               shouldAlign;
            
            if (hasInput) {
                drive.setGamepad(true);
                
                // Apply squared input curve
                double forward = rawLeftY * Math.abs(rawLeftY);
                double strafe = rawLeftX * Math.abs(rawLeftX);
                
                // Determine turn input based on turret lock mode
                double turn;
                
                // Check if chassis auto-aim should be used
                // SOFT LOCK or no turret: chassis auto-aim
                // HARD LOCK: turret handles aiming, chassis manual only
                boolean useChassisAutoAim = shouldAlign && 
                    (turret == null || turret.getLockMode() == Turret.LockMode.SOFT_LOCK);
                
                if (useChassisAutoAim && vision != null) {
                    // SOFT LOCK MODE: Chassis auto-aim using tx from Limelight
                    turn = drive.getAlignTurnPower(vision);
                } else {
                    // HARD LOCK or MANUAL MODE: Manual turn control
                    turn = rawRightX * Math.abs(rawRightX);
                }
                
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

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
