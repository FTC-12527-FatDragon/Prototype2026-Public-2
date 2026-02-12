package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.utils.Util;

/**
 * SoloBase — shared logic for SoloRed / SoloBlue.
 *
 * Subclasses only override four tiny methods that describe the alliance:
 *   {@link #getAlliance()}, {@link #getGoalX()}, {@link #getGoalY()},
 *   {@link #getGoalColor()}.
 *
 * === MODES ===
 *   MANUAL   — D-Pad turret control, presets (X/B/-90°+90°, Down=0↔180)
 *   AUTO_AIM — Inertial navigation auto-aim to alliance basket
 *
 * === CONTROLS ===
 *   Right Stick Click : toggle MANUAL ↔ AUTO_AIM
 *   A (AUTO_AIM)      : toggle auto-aim on / off
 */
public abstract class SoloBase extends CommandOpMode {

    // ── Alliance hooks (override in subclasses) ────────────────
    protected abstract Turret.Alliance getAlliance();
    protected abstract double getGoalX();
    protected abstract double getGoalY();
    /** Hex color for dashboard drawing, e.g. "#FF0000" or "#0000FF". */
    protected abstract String getGoalColor();

    // ── Core references ────────────────────────────────────────
    protected Robot robot;
    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    private final boolean[] isAuto = {false};

    // ── Mode control ───────────────────────────────────────────
    protected enum ControlMode { MANUAL, AUTO_AIM }
    private ControlMode controlMode = ControlMode.MANUAL;
    private boolean lastRightStickButton = false;

    // ── Manual-mode state ──────────────────────────────────────
    private boolean lastDpadPressed = false;
    private long dpadReleaseTime = 0;
    private static final long TURRET_HOLD_DELAY_MS = 300;

    private boolean lastDpadDown = false;
    private boolean lastXButton  = false;
    private boolean lastBButton  = false;
    private boolean turretAt180  = false;

    // ── Emergency disable edge detection ───────────────────────
    private boolean lastIntakeDisableCombo  = false;
    private boolean lastShooterDisableCombo = false;
    private boolean lastTurretDisableCombo  = false;

    // ── Auto-aim state ─────────────────────────────────────────
    private enum AimState { LOCKED_AT_ZERO, INERTIAL_NAVIGATION }
    private AimState aimState = AimState.LOCKED_AT_ZERO;
    private boolean lastAButton      = false;
    private boolean hasValidPosition = false;
    @SuppressWarnings("unused")
    private boolean isFlipping       = false;

    // Turret software limits
    private static final double MIN_TURRET_ANGLE = -145.0;
    private static final double MAX_TURRET_ANGLE = 226.2;

    // Cached absolute position (updated once per loop)
    private double cachedRobotX    = 0;
    private double cachedRobotY    = 0;
    private double cachedHeadingDeg = 0;

    // Debug telemetry values
    private double debugGoalFieldAngle = 0;
    private double debugChosenAngle    = 0;

    // ================================================================
    //  Lifecycle
    // ================================================================

    @Override
    public void initialize() {
        robot     = new Robot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // Register subsystems
        CommandScheduler cs = CommandScheduler.getInstance();
        cs.registerSubsystem(robot.shooter);
        cs.registerSubsystem(robot.transit);
        cs.registerSubsystem(robot.intake);
        if (robot.turret != null) {
            cs.registerSubsystem(robot.turret);
            robot.turret.setAlliance(getAlliance());
            robot.turret.disableLock();
        }

        // Default drive command
        robot.drive.setDefaultCommand(new TeleOpDriveCommand(
                robot.drive, robot.vision, robot.turret,
                gamepadEx1, gamepadEx2, isAuto));

        // Left stick button → reset heading
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(new InstantCommand(() -> robot.drive.resetHeading()));

        DriverControls.bind(gamepadEx1, robot, isAuto);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dpadReleaseTime = System.currentTimeMillis();

        String allianceName = getAlliance().name();
        telemetry.addLine("=== SOLO " + allianceName + " ===");
        telemetry.addLine("Right Stick Click: Toggle MANUAL/AUTO_AIM");
        telemetry.addData("Target", allianceName + " basket (" +
                (int) getGoalX() + ", " + (int) getGoalY() + ")");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        handleModeToggle();
        handleEmergencyDisable();
        updateAbsolutePosition();
        cachePosition();

        if (controlMode == ControlMode.MANUAL) {
            runManualMode();
        } else {
            runAutoAimMode();
        }

        handleIntakeShooter();
        updateTelemetry();
        updateDashboard();
    }

    // ================================================================
    //  Mode toggle
    // ================================================================

    private void handleModeToggle() {
        boolean btn = gamepadEx1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        if (btn && !lastRightStickButton) {
            if (controlMode == ControlMode.MANUAL) {
                controlMode = ControlMode.AUTO_AIM;
                aimState = AimState.LOCKED_AT_ZERO;
                if (robot.turret != null) robot.turret.enableSoftLock(0);
            } else {
                controlMode = ControlMode.MANUAL;
                if (robot.turret != null) robot.turret.disableLock();
            }
        }
        lastRightStickButton = btn;
    }

    // ================================================================
    //  Manual mode
    // ================================================================

    private void runManualMode() {
        if (robot.turret == null) return;

        boolean dpadLeft  = gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT);
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
            long elapsed = System.currentTimeMillis() - dpadReleaseTime;
            if (dpadReleaseTime > 0 && elapsed >= TURRET_HOLD_DELAY_MS
                    && !robot.turret.isHoldingPosition()) {
                robot.turret.holdCurrentPosition();
            }
        }
        lastDpadPressed = dpadPressed;

        // Presets
        boolean dpadDown = gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN);
        boolean xButton  = gamepadEx1.getButton(GamepadKeys.Button.X);
        boolean bButton  = gamepadEx1.getButton(GamepadKeys.Button.B);

        if (dpadDown && !lastDpadDown) {
            robot.turret.releaseHold();
            if (turretAt180) { robot.turret.enableSoftLock(0);   turretAt180 = false; }
            else              { robot.turret.enableSoftLock(180); turretAt180 = true;  }
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
        lastXButton  = xButton;
        lastBButton  = bButton;
    }

    // ================================================================
    //  Auto-aim mode
    // ================================================================

    private void runAutoAimMode() {
        if (robot.turret == null || robot.vision == null) return;

        boolean aButton  = gamepadEx1.getButton(GamepadKeys.Button.A);
        boolean aPressed = aButton && !lastAButton;

        int tagId = robot.vision.getDetectedTagId();
        boolean canSeeGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID
                              || tagId == Vision.RED_GOAL_TAG_ID);

        switch (aimState) {
            case LOCKED_AT_ZERO:
                if (robot.turret.getLockMode() != Turret.LockMode.SOFT_LOCK
                        || Math.abs(robot.turret.getTargetAngle()) > 0.1) {
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

    // ================================================================
    //  Shared helpers
    // ================================================================

    private void handleEmergencyDisable() {
        boolean intakeCombo = gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3
                && gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean shooterCombo = gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3
                && gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean turretCombo = gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER)
                && gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                && !intakeCombo && !shooterCombo;

        if (intakeCombo  && !lastIntakeDisableCombo)                        robot.intake.toggleDisabled();
        if (shooterCombo && !lastShooterDisableCombo)                       robot.shooter.toggleDisabled();
        if (turretCombo  && !lastTurretDisableCombo && robot.turret != null) robot.turret.toggleDisabled();

        lastIntakeDisableCombo  = intakeCombo;
        lastShooterDisableCombo = shooterCombo;
        lastTurretDisableCombo  = turretCombo;
    }

    private void updateAbsolutePosition() {
        if (robot.vision == null) return;

        int tagId = robot.vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);

        if (isGoalTag) {
            double turretDeg = (robot.turret != null && robot.turret.isCalibrated())
                    ? robot.turret.getAngleDegrees() : 0;
            if (robot.drive.updateAbsolutePositionFromVisionWithTurret(robot.vision, turretDeg)) {
                hasValidPosition = true;
            }
        } else {
            robot.drive.updateAbsolutePositionFromOdometry();
        }
    }

    private void cachePosition() {
        if (!hasValidPosition) return;
        cachedRobotX    = robot.drive.getAbsoluteX();
        cachedRobotY    = robot.drive.getAbsoluteY();
        cachedHeadingDeg = Math.toDegrees(robot.drive.getAbsoluteHeading());
    }

    private void calculateAndApplyTurretAngle() {
        if (!hasValidPosition || robot.turret == null) return;

        double heading = Util.normalizeAngleDegrees0To360(cachedHeadingDeg);
        double dx = getGoalX() - cachedRobotX;
        double dy = getGoalY() - cachedRobotY;
        double fieldAngle = Util.normalizeAngleDegrees0To360(Math.toDegrees(Math.atan2(dy, dx)));

        double turnRight = heading - fieldAngle;
        double turnLeft  = -((360.0 - heading) + fieldAngle);

        debugGoalFieldAngle = fieldAngle;

        boolean rightOk = turnRight >= MIN_TURRET_ANGLE && turnRight <= MAX_TURRET_ANGLE;
        boolean leftOk  = turnLeft  >= MIN_TURRET_ANGLE && turnLeft  <= MAX_TURRET_ANGLE;

        double target;
        if (rightOk && leftOk) {
            target = (Math.abs(turnRight) <= Math.abs(turnLeft)) ? turnRight : turnLeft;
        } else if (rightOk) {
            target = turnRight;
        } else if (leftOk) {
            target = turnLeft;
        } else {
            target = Math.max(MIN_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, turnRight));
        }

        debugChosenAngle = target;
        robot.turret.enableSoftLock(target);
    }

    private void handleIntakeShooter() {
        boolean shooterBtn =
                gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
             || gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
             || gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.3;
        boolean feedBtn = shooterBtn
                && gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.3;
        robot.intake.setShooting(feedBtn);

        boolean intakeBtn = gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.3;
        robot.intake.setFastIntaking(intakeBtn);
    }

    // ================================================================
    //  Telemetry
    // ================================================================

    private void updateTelemetry() {
        String name = getAlliance().name();

        telemetry.addLine("========== SOLO " + name + " ==========");
        telemetry.addData("MODE", controlMode == ControlMode.MANUAL ? "MANUAL (D-Pad)" : "AUTO-AIM");
        telemetry.addLine("Right Stick Click → Toggle Mode");

        if (controlMode == ControlMode.AUTO_AIM) {
            telemetry.addData("Aim State", aimState.toString());
            telemetry.addData("Target", name + " (" + (int) getGoalX() + ", " + (int) getGoalY() + ")");
        }

        // Vision
        telemetry.addLine("========== VISION ==========");
        if (robot.vision != null) {
            int tagId = robot.vision.getDetectedTagId();
            telemetry.addData("Tag ID", tagId == -1 ? "NONE" : tagId);
            telemetry.addData("TX", String.format("%.1f°", robot.vision.getTx()));
        }

        // Absolute position
        telemetry.addLine("========== POSITION ==========");
        telemetry.addData("Has Pos", hasValidPosition ? "YES" : "NO");
        if (hasValidPosition) {
            telemetry.addData("Abs X",       String.format("%.1f in", cachedRobotX));
            telemetry.addData("Abs Y",       String.format("%.1f in", cachedRobotY));
            telemetry.addData("Abs Heading", String.format("%.1f°",   cachedHeadingDeg));
        }

        // Turret
        if (robot.turret != null) {
            telemetry.addLine("========== TURRET ==========");
            telemetry.addData("Angle",  String.format("%.1f°", robot.turret.getAngleDegrees()));
            telemetry.addData("Target", String.format("%.1f°", robot.turret.getTargetAngle()));
            if (controlMode == ControlMode.AUTO_AIM && aimState == AimState.INERTIAL_NAVIGATION) {
                telemetry.addData("Goal Angle", String.format("%.1f°", debugGoalFieldAngle));
                telemetry.addData("Chosen",     String.format("%.1f°", debugChosenAngle));
            }
        }

        // Shooter
        telemetry.addLine("========== SHOOTER ==========");
        telemetry.addData("READY", robot.shooter.isShooterAtSetPoint());
        telemetry.addData("State", robot.shooter.shooterState);
        telemetry.addData("BOOST", robot.shooter.getBoostStatus());

        // Emergency disable
        telemetry.addLine("========== DISABLE (GP2) ==========");
        telemetry.addData("Intake",  robot.intake.isDisabled()  ? "DISABLED" : "OK");
        telemetry.addData("Shooter", robot.shooter.isDisabled() ? "DISABLED" : "OK");
        if (robot.turret != null) {
            telemetry.addData("Turret", robot.turret.isDisabled() ? "DISABLED" : "OK");
        }

        telemetry.update();
    }

    // ================================================================
    //  Dashboard
    // ================================================================

    private void updateDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        org.firstinspires.ftc.teamcode.utils.DashboardUtil.drawRobot(packet, robot.drive.getPose());

        if (hasValidPosition) {
            double headingRad = Math.toRadians(cachedHeadingDeg);

            packet.put("Mode",  controlMode.toString());
            packet.put("Abs X", String.format("%.1f", cachedRobotX));
            packet.put("Abs Y", String.format("%.1f", cachedRobotY));

            // Green circle + heading line
            packet.fieldOverlay()
                    .setStroke("#00FF00").setStrokeWidth(2)
                    .strokeCircle(cachedRobotX, cachedRobotY, 4);
            double len = 8;
            packet.fieldOverlay()
                    .setStroke("#00FF00")
                    .strokeLine(cachedRobotX, cachedRobotY,
                                cachedRobotX + len * Math.cos(headingRad),
                                cachedRobotY + len * Math.sin(headingRad));

            // Aim line in auto-aim mode
            if (controlMode == ControlMode.AUTO_AIM) {
                packet.fieldOverlay()
                        .setStroke(getGoalColor()).setStrokeWidth(1)
                        .strokeLine(cachedRobotX, cachedRobotY, getGoalX(), getGoalY());
            }
        }

        // Goal marker
        String c = getGoalColor();
        packet.fieldOverlay()
                .setStroke(c).setFill(c + "44")
                .fillCircle(getGoalX(), getGoalY(), 3);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
