package org.firstinspires.ftc.teamcode.opmodes.teleops;

/**
 * SoloBlue - Combined Solo + SoloTest (Inertial Auto-Aim to BLUE Goal)
 * 
 * === MODES ===
 * - MANUAL: D-Pad turret control (same as Solo)
 * - AUTO_AIM: Inertial navigation auto-aim to BLUE basket (4, 140)
 * 
 * === CONTROLS ===
 * - Right Stick Click: Toggle between MANUAL and AUTO_AIM modes
 * - A (in AUTO_AIM): Toggle auto-aim on/off
 * 
 * Target: BLUE goal (tag 20 basket at 4, 140)
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
import org.firstinspires.ftc.teamcode.utils.Util;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@Config
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Solo Blue", group = "TeleOp")
public class SoloBlue extends CommandOpMode {
    private Robot robot;
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;
    private boolean[] isAuto = {false};
    
    // ========== MODE CONTROL ==========
    private enum ControlMode {
        MANUAL,     // Solo mode: D-Pad turret control
        AUTO_AIM    // SoloTest mode: Inertial auto-aim
    }
    private ControlMode controlMode = ControlMode.MANUAL;
    private boolean lastRightStickButton = false;
    
    // ========== SOLO MODE VARIABLES ==========
    // Emergency disable edge detection
    private boolean lastIntakeDisableCombo = false;
    private boolean lastShooterDisableCombo = false;
    private boolean lastTurretDisableCombo = false;
    
    // Turret PID hold delay
    private boolean lastDpadPressed = false;
    private long dpadReleaseTime = 0;
    private static final long TURRET_HOLD_DELAY_MS = 300;
    
    // Turret preset buttons
    private boolean lastDpadDown = false;
    private boolean lastXButton = false;
    private boolean lastBButton = false;
    private boolean turretAt180 = false;
    
    // ========== AUTO-AIM MODE VARIABLES ==========
    private enum AimState {
        LOCKED_AT_ZERO,
        INERTIAL_NAVIGATION
    }
    private AimState aimState = AimState.LOCKED_AT_ZERO;
    private boolean lastAButton = false;
    private boolean hasValidPosition = false;
    
    // Goal coordinates (BLUE goal)
    private static final double GOAL_X = TurretConstants.blueGoalX;  // 4
    private static final double GOAL_Y = TurretConstants.blueGoalY;  // 140
    private static final int TARGET_TAG_ID = Vision.BLUE_GOAL_TAG_ID;  // 20
    
    // Software limits
    private static final double MIN_TURRET_ANGLE = -145.0;
    private static final double MAX_TURRET_ANGLE = 226.2;
    
    // Flip state
    private boolean isFlipping = false;
    
    // Cached position values
    private double cachedRobotX = 0;
    private double cachedRobotY = 0;
    private double cachedHeadingDeg = 0;
    
    // Debug values
    private double debugGoalFieldAngle = 0;
    private double debugTurnRight = 0;
    private double debugTurnLeft = 0;
    private double debugChosenAngle = 0;

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // Register subsystems
        CommandScheduler.getInstance().registerSubsystem(robot.shooter);
        CommandScheduler.getInstance().registerSubsystem(robot.transit);
        CommandScheduler.getInstance().registerSubsystem(robot.intake);
        if (robot.turret != null) {
            CommandScheduler.getInstance().registerSubsystem(robot.turret);
            robot.turret.setAlliance(Turret.Alliance.BLUE);
            robot.turret.disableLock();  // Start in MANUAL mode
        }

        // Default drive command
        robot.drive.setDefaultCommand(new TeleOpDriveCommand(
                robot.drive,
                robot.vision,
                robot.turret,
                gamepadEx1,
                isAuto
        ));

        // Left stick button: Reset heading
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> robot.drive.resetHeading())
        );

        DriverControls.bind(gamepadEx1, robot, isAuto);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        dpadReleaseTime = System.currentTimeMillis();
        
        telemetry.addLine("=== SOLO BLUE ===");
        telemetry.addLine("Right Stick Click: Toggle MANUAL/AUTO_AIM");
        telemetry.addLine("Target: BLUE basket (4, 140)");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        
        // ========== MODE TOGGLE (Right Stick Click) ==========
        boolean rightStickButton = gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        if (rightStickButton && !lastRightStickButton) {
            if (controlMode == ControlMode.MANUAL) {
                controlMode = ControlMode.AUTO_AIM;
                aimState = AimState.LOCKED_AT_ZERO;
                if (robot.turret != null) {
                    robot.turret.enableSoftLock(0);
                }
            } else {
                controlMode = ControlMode.MANUAL;
                if (robot.turret != null) {
                    robot.turret.disableLock();
                }
            }
        }
        lastRightStickButton = rightStickButton;
        
        // ========== GAMEPAD2 EMERGENCY DISABLE ==========
        handleEmergencyDisable();
        
        // ========== UPDATE ABSOLUTE POSITION ==========
        updateAbsolutePosition();
        
        // ========== CACHE POSITION VALUES ==========
        if (hasValidPosition) {
            cachedRobotX = robot.drive.getAbsoluteX();
            cachedRobotY = robot.drive.getAbsoluteY();
            cachedHeadingDeg = Math.toDegrees(robot.drive.getAbsoluteHeading());
        }
        
        // ========== MODE-SPECIFIC LOGIC ==========
        if (controlMode == ControlMode.MANUAL) {
            runManualMode();
        } else {
            runAutoAimMode();
        }
        
        // ========== INTAKE/SHOOTER CONTROLS ==========
        handleIntakeShooter();
        
        // ========== TELEMETRY ==========
        updateTelemetry();
        
        // ========== DASHBOARD ==========
        updateDashboard();
    }
    
    // ==================== MANUAL MODE (Solo) ====================
    private void runManualMode() {
        if (robot.turret == null) return;
        
        boolean dpadLeft = gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT);
        boolean dpadRight = gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT);
        boolean dpadPressed = dpadLeft || dpadRight;
        
        if (dpadLeft) {
            robot.turret.releaseHold();
            robot.turret.setPower(1.0);
        } else if (dpadRight) {
            robot.turret.releaseHold();
            robot.turret.setPower(-1.0);
        } else {
            if (lastDpadPressed && !dpadPressed) {
                dpadReleaseTime = System.currentTimeMillis();
                robot.turret.releaseHold();
                robot.turret.setPower(0);
            }
            
            long timeSinceRelease = System.currentTimeMillis() - dpadReleaseTime;
            if (dpadReleaseTime > 0 && timeSinceRelease >= TURRET_HOLD_DELAY_MS) {
                if (!robot.turret.isHoldingPosition()) {
                    robot.turret.holdCurrentPosition();
                }
            }
        }
        lastDpadPressed = dpadPressed;
        
        // Turret presets
        boolean dpadDown = gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN);
        boolean xButton = gamepadEx1.getButton(GamepadKeys.Button.X);
        boolean bButton = gamepadEx1.getButton(GamepadKeys.Button.B);
        
        if (dpadDown && !lastDpadDown) {
            robot.turret.releaseHold();
            if (turretAt180) {
                robot.turret.enableSoftLock(0);
                turretAt180 = false;
            } else {
                robot.turret.enableSoftLock(180);
                turretAt180 = true;
            }
        }
        
        if (xButton && !lastXButton) {
            robot.turret.releaseHold();
            robot.turret.enableSoftLock(-90);
            turretAt180 = false;
        }
        
        if (bButton && !lastBButton) {
            robot.turret.releaseHold();
            robot.turret.enableSoftLock(90);
            turretAt180 = false;
        }
        
        lastDpadDown = dpadDown;
        lastXButton = xButton;
        lastBButton = bButton;
    }
    
    // ==================== AUTO-AIM MODE (SoloTest) ====================
    private void runAutoAimMode() {
        if (robot.turret == null || robot.vision == null) return;
        
        boolean aButton = gamepadEx1.getButton(GamepadKeys.Button.A);
        boolean aPressed = aButton && !lastAButton;
        
        int tagId = robot.vision.getDetectedTagId();
        boolean canSeeGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        switch (aimState) {
            case LOCKED_AT_ZERO:
                if (robot.turret.getLockMode() != Turret.LockMode.SOFT_LOCK ||
                    Math.abs(robot.turret.getTargetAngle()) > 0.1) {
                    robot.turret.enableSoftLock(0);
                }
                
                if (aPressed && canSeeGoalTag && hasValidPosition) {
                    aimState = AimState.INERTIAL_NAVIGATION;
                    isFlipping = false;
                }
                break;
                
            case INERTIAL_NAVIGATION:
                if (aPressed) {
                    aimState = AimState.LOCKED_AT_ZERO;
                    robot.turret.enableSoftLock(0);
                    isFlipping = false;
                } else {
                    calculateAndApplyTurretAngle();
                }
                break;
        }
        lastAButton = aButton;
    }
    
    // ==================== SHARED METHODS ====================
    private void handleEmergencyDisable() {
        boolean intakeDisableCombo = gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3
                && gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean shooterDisableCombo = gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3
                && gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean turretDisableCombo = gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER)
                && gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                && !intakeDisableCombo && !shooterDisableCombo;
        
        if (intakeDisableCombo && !lastIntakeDisableCombo) {
            robot.intake.toggleDisabled();
        }
        if (shooterDisableCombo && !lastShooterDisableCombo) {
            robot.shooter.toggleDisabled();
        }
        if (turretDisableCombo && !lastTurretDisableCombo && robot.turret != null) {
            robot.turret.toggleDisabled();
        }
        
        lastIntakeDisableCombo = intakeDisableCombo;
        lastShooterDisableCombo = shooterDisableCombo;
        lastTurretDisableCombo = turretDisableCombo;
    }
    
    private void updateAbsolutePosition() {
        if (robot.vision == null) return;
        
        int tagId = robot.vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        if (isGoalTag) {
            double turretAngleDeg = (robot.turret != null && robot.turret.isCalibrated())
                    ? robot.turret.getAngleDegrees() : 0;  // Must be degrees, not radians!
            boolean success = robot.drive.updateAbsolutePositionFromVisionWithTurret(robot.vision, turretAngleDeg);
            if (success) {
                hasValidPosition = true;
            }
        } else {
            robot.drive.updateAbsolutePositionFromOdometry();
        }
    }
    
    private void calculateAndApplyTurretAngle() {
        if (!hasValidPosition || robot.turret == null) return;
        
        double robotX = cachedRobotX;
        double robotY = cachedRobotY;
        double heading = Util.normalizeAngleDegrees0To360(cachedHeadingDeg);
        
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;
        double x = Math.toDegrees(Math.atan2(dy, dx));
        x = Util.normalizeAngleDegrees0To360(x);
        
        double turnRight = heading - x;
        double turnLeft = -((360.0 - heading) + x);
        
        debugGoalFieldAngle = x;
        debugTurnRight = turnRight;
        debugTurnLeft = turnLeft;
        
        boolean rightInLimits = (turnRight >= MIN_TURRET_ANGLE && turnRight <= MAX_TURRET_ANGLE);
        boolean leftInLimits = (turnLeft >= MIN_TURRET_ANGLE && turnLeft <= MAX_TURRET_ANGLE);
        
        double targetAngle;
        if (rightInLimits && leftInLimits) {
            targetAngle = (Math.abs(turnRight) <= Math.abs(turnLeft)) ? turnRight : turnLeft;
        } else if (rightInLimits) {
            targetAngle = turnRight;
        } else if (leftInLimits) {
            targetAngle = turnLeft;
        } else {
            targetAngle = turnRight;
            if (targetAngle < MIN_TURRET_ANGLE) targetAngle = MIN_TURRET_ANGLE;
            if (targetAngle > MAX_TURRET_ANGLE) targetAngle = MAX_TURRET_ANGLE;
        }
        
        debugChosenAngle = targetAngle;
        robot.turret.enableSoftLock(targetAngle);
    }
    
    private void handleIntakeShooter() {
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
    }
    
    private void updateTelemetry() {
        Pose2D pose = robot.drive.getPose();
        
        // Mode indicator
        telemetry.addLine("========== SOLO BLUE ==========");
        telemetry.addData("MODE", controlMode == ControlMode.MANUAL ? "MANUAL (D-Pad)" : "AUTO-AIM");
        telemetry.addLine("Right Stick Click → Toggle Mode");
        
        if (controlMode == ControlMode.AUTO_AIM) {
            telemetry.addData("Aim State", aimState.toString());
            telemetry.addData("Target", "BLUE (4, 140)");
        }
        
        // Vision
        telemetry.addLine("========== VISION ==========");
        if (robot.vision != null) {
            int tagId = robot.vision.getDetectedTagId();
            telemetry.addData("Tag ID", tagId == -1 ? "NONE" : tagId);
            telemetry.addData("TX", String.format("%.1f°", robot.vision.getTx()));
        }
        
        // Position
        telemetry.addLine("========== POSITION ==========");
        telemetry.addData("Has Pos", hasValidPosition ? "YES" : "NO");
        if (hasValidPosition) {
            telemetry.addData("Abs X", String.format("%.1f in", cachedRobotX));
            telemetry.addData("Abs Y", String.format("%.1f in", cachedRobotY));
            telemetry.addData("Abs Heading", String.format("%.1f°", cachedHeadingDeg));
        }
        
        // Turret
        if (robot.turret != null) {
            telemetry.addLine("========== TURRET ==========");
            telemetry.addData("Angle", String.format("%.1f°", robot.turret.getAngleDegrees()));
            telemetry.addData("Target", String.format("%.1f°", robot.turret.getTargetAngle()));
            
            if (controlMode == ControlMode.AUTO_AIM && aimState == AimState.INERTIAL_NAVIGATION) {
                telemetry.addData("Goal Angle", String.format("%.1f°", debugGoalFieldAngle));
                telemetry.addData("Chosen", String.format("%.1f°", debugChosenAngle));
            }
        }
        
        // Shooter
        telemetry.addLine("========== SHOOTER ==========");
        telemetry.addData("READY", robot.shooter.isShooterAtSetPoint());
        telemetry.addData("State", robot.shooter.shooterState);
        telemetry.addData("BOOST", robot.shooter.getBoostStatus());
        
        // Emergency status
        telemetry.addLine("========== DISABLE (GP2) ==========");
        telemetry.addData("Intake", robot.intake.isDisabled() ? "DISABLED" : "OK");
        telemetry.addData("Shooter", robot.shooter.isDisabled() ? "DISABLED" : "OK");
        if (robot.turret != null) {
            telemetry.addData("Turret", robot.turret.isDisabled() ? "DISABLED" : "OK");
        }
        
        telemetry.update();
    }
    
    private void updateDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        
        // Draw robot
        org.firstinspires.ftc.teamcode.utils.DashboardUtil.drawRobot(packet, robot.drive.getPose());
        
        // Draw absolute position
        if (hasValidPosition) {
            double absHeadingRad = Math.toRadians(cachedHeadingDeg);
            
            packet.put("Mode", controlMode.toString());
            packet.put("Abs X", String.format("%.1f", cachedRobotX));
            packet.put("Abs Y", String.format("%.1f", cachedRobotY));
            
            packet.fieldOverlay()
                    .setStroke("#00FF00")
                    .setStrokeWidth(2)
                    .strokeCircle(cachedRobotX, cachedRobotY, 4);
            
            double lineLen = 8;
            double endX = cachedRobotX + lineLen * Math.cos(absHeadingRad);
            double endY = cachedRobotY + lineLen * Math.sin(absHeadingRad);
            packet.fieldOverlay()
                    .setStroke("#00FF00")
                    .strokeLine(cachedRobotX, cachedRobotY, endX, endY);
            
            if (controlMode == ControlMode.AUTO_AIM) {
                packet.fieldOverlay()
                        .setStroke("#0000FF")
                        .setStrokeWidth(1)
                        .strokeLine(cachedRobotX, cachedRobotY, GOAL_X, GOAL_Y);
            }
        }
        
        // Draw goal (blue)
        packet.fieldOverlay()
                .setStroke("#0000FF")
                .setFill("#0000FF44")
                .fillCircle(GOAL_X, GOAL_Y, 3);
        
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
