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
    
    // Auto-aim state (one-shot heading control)
    private boolean autoAimActive = false;      // Is auto-aim currently running?
    private boolean lastAPressed = false;       // A button state last frame (for edge detection)
    private long autoAimStartTime = 0;          // When auto-aim started (for timeout)
    private static final long AUTO_AIM_TIMEOUT_MS = 2000;  // Auto-aim timeout: 2 seconds
    
    // One-shot heading target (read TX once, then turn to that heading)
    private double targetHeadingRad = 0;        // Target heading to turn to
    private boolean headingCaptured = false;    // Has target heading been captured?

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
            
            // Auto-aim: ONE-SHOT heading control
            // Press A: Read TX once, calculate target heading, then turn to it
            if (aPressed && !lastAPressed) {
                if (autoAimActive) {
                    // Already aiming, cancel it
                    autoAimActive = false;
                    headingCaptured = false;
                } else {
                    // Start aiming - capture target heading from TX
                    autoAimActive = true;
                    headingCaptured = false;
                    autoAimStartTime = System.currentTimeMillis();
                    
                    // Capture TX and calculate target heading
                    if (vision != null) {
                        int tagId = vision.getDetectedTagId();
                        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
                        if (isGoalTag) {
                            double tx = vision.getTx();  // Degrees
                            double currentHeading = drive.getHeading();  // Radians
                            // Target = current heading - TX (TX positive = target is to the right)
                            targetHeadingRad = currentHeading - Math.toRadians(tx);
                            headingCaptured = true;
                        }
                    }
                }
            }
            lastAPressed = aPressed;
            
            // Stop auto-aim when:
            // 1. Reached target heading
            // 2. User has ANY right stick input (manual override)
            // 3. Timeout
            // 4. No heading captured (no tag visible when pressed)
            boolean manualTurnOverride = Math.abs(rawRightX) > 0.1;
            boolean timeout = (System.currentTimeMillis() - autoAimStartTime) > AUTO_AIM_TIMEOUT_MS;
            boolean reachedTarget = headingCaptured && drive.isAtTargetHeading(targetHeadingRad);
            if (autoAimActive && (reachedTarget || manualTurnOverride || timeout || !headingCaptured)) {
                autoAimActive = false;
                headingCaptured = false;
            }
            
            // Use auto-aim state
            boolean shouldAlign = autoAimActive && headingCaptured;
            
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
                // SOFT LOCK or MANUAL or no turret: chassis auto-aim
                // HARD LOCK: turret handles aiming, chassis manual only
                boolean useChassisAutoAim = shouldAlign && 
                    (turret == null || turret.getLockMode() != Turret.LockMode.HARD_LOCK);
                
                if (useChassisAutoAim && headingCaptured) {
                    // ONE-SHOT MODE: Turn to captured target heading (not tracking TX)
                    turn = drive.getTurnPowerToHeading(targetHeadingRad);
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

