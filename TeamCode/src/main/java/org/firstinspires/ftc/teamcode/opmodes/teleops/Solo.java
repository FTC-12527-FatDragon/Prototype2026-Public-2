package org.firstinspires.ftc.teamcode.opmodes.teleops;

/**
 * Main TeleOp OpMode — Field-Centric Mecanum drive with manual turret control.
 *
 * Gamepad 1:
 *   Left Stick     Move (field-centric)
 *   Right Stick    Rotate chassis
 *   L-Stick Click  Reset heading
 *   R-Stick Click  Toggle aim mode (Chassis ↔ Turret)
 *   D-Pad ←/→      Turret manual (open-loop)
 *   D-Pad ↓        Turret 0°/180° toggle
 *   X / B          Turret −45° / +45°
 *   A              Auto-aim (chassis or turret, by mode)
 *
 * Gamepad 2:
 *   D-Pad ←/→/↑/↓  Turret presets −45° / +45° / 0° / 180°
 *   LT+LB           Toggle intake disable
 *   RT+RB           Toggle shooter disable
 *   LB+RB           Toggle turret disable
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
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@Config
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Solo", group = "TeleOp")
public class Solo extends CommandOpMode {

    // ── Core ────────────────────────────────────────────────────
    private Robot robot;
    private GamepadEx gp1, gp2;
    private final boolean[] isAuto = {false};

    // ── Emergency-disable edge detection (GP2 combos) ───────────
    private boolean prevIntakeCombo, prevShooterCombo, prevTurretCombo;

    // ── Turret D-Pad manual control ─────────────────────────────
    private boolean prevDpadActive;
    private long    dpadReleaseTime;
    private static final long TURRET_HOLD_DELAY_MS = 300;

    // ── Turret preset edge detection — GP1 ──────────────────────
    private boolean prevDpadDown, prevXButton, prevBButton;
    private boolean turretAt180;

    // ── Turret preset edge detection — GP2 ──────────────────────
    private boolean prevG2Left, prevG2Right, prevG2Up, prevG2Down;

    // ── Aim-mode toggle (R-Stick click) ─────────────────────────
    private final boolean[] turretAimMode = {false};
    private boolean prevRStickBtn;
    private boolean prevAForTurret;

    // ── Turret auto-aim state machine ───────────────────────────
    private enum AimState { IDLE, INERTIAL_TURNING, TX_CORRECTING }
    private AimState aimState = AimState.IDLE;
    private long     aimStartMs;
    private int      lastSeenGoalTag = -1;
    private static final long AIM_TIMEOUT_MS    = 2000;
    private static final long TX_SETTLE_MS      = 300;

    // ═════════════════════════════════════════════════════════════
    //  Initialization
    // ═════════════════════════════════════════════════════════════

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        gp1   = new GamepadEx(gamepad1);
        gp2   = new GamepadEx(gamepad2);

        registerSubsystems();
        configureDefaultDrive();
        configureHeadingReset();
        DriverControls.bind(gp1, robot, isAuto);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (robot.turret != null) {
            robot.turret.disableLock();
            dpadReleaseTime = System.currentTimeMillis();
        }
    }

    private void registerSubsystems() {
        CommandScheduler cs = CommandScheduler.getInstance();
        cs.registerSubsystem(robot.shooter);
        cs.registerSubsystem(robot.transit);
        cs.registerSubsystem(robot.intake);
        if (robot.turret != null) cs.registerSubsystem(robot.turret);
    }

    private void configureDefaultDrive() {
        TeleOpDriveCommand drive = new TeleOpDriveCommand(
                robot.drive, robot.vision, robot.turret, gp1, gp2, isAuto);
        drive.setTurretAimMode(turretAimMode);
        robot.drive.setDefaultCommand(drive);
    }

    private void configureHeadingReset() {
        new FunctionalButton(() -> gp1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                .whenPressed(new InstantCommand(() -> robot.drive.resetHeading()));
    }

    // ═════════════════════════════════════════════════════════════
    //  Main Loop — clean, declarative
    // ═════════════════════════════════════════════════════════════

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        handleEmergencyDisable();
        handleTurretControl();
        updateAbsolutePosition();
        updateIntakeShooterFlags();

        emitTelemetry();
        emitDashboard();
    }

    // ═════════════════════════════════════════════════════════════
    //  Emergency Disable — GP2 combo toggles
    // ═════════════════════════════════════════════════════════════

    private void handleEmergencyDisable() {
        boolean intakeCombo  = gp2Trigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                            && gp2.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean shooterCombo = gp2Trigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                            && gp2.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean turretCombo  = gp2.getButton(GamepadKeys.Button.LEFT_BUMPER)
                            && gp2.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                            && !intakeCombo && !shooterCombo;

        if (risingEdge(intakeCombo,  prevIntakeCombo))                          robot.intake.toggleDisabled();
        if (risingEdge(shooterCombo, prevShooterCombo))                         robot.shooter.toggleDisabled();
        if (risingEdge(turretCombo,  prevTurretCombo) && robot.turret != null)  robot.turret.toggleDisabled();

        prevIntakeCombo  = intakeCombo;
        prevShooterCombo = shooterCombo;
        prevTurretCombo  = turretCombo;
    }

    // ═════════════════════════════════════════════════════════════
    //  Turret Control — all turret logic in one place
    // ═════════════════════════════════════════════════════════════

    private void handleTurretControl() {
        if (robot.turret == null) return;

        trackGoalTag();
        handleAimModeToggle();
        handleTurretAutoAim();
        handleGp2TurretPresets();
        handleDpadManualControl();
        handleGp1TurretPresets();
    }

    /** Remember which goal tag we last saw (for inertial fallback). */
    private void trackGoalTag() {
        if (robot.vision == null) return;
        int tag = robot.vision.getDetectedTagId();
        if (tag == Vision.BLUE_GOAL_TAG_ID || tag == Vision.RED_GOAL_TAG_ID) {
            lastSeenGoalTag = tag;
        }
    }

    /** R-Stick click: toggle between CHASSIS and TURRET aim mode. */
    private void handleAimModeToggle() {
        boolean btn = gp1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        if (risingEdge(btn, prevRStickBtn)) {
            turretAimMode[0] = !turretAimMode[0];
            aimState = AimState.IDLE;
            rumbleBoth(turretAimMode[0] ? 500 : 200);
        }
        prevRStickBtn = btn;
    }

    /**
     * Turret auto-aim state machine (active only in TURRET_AIM mode).
     *
     * A pressed → tag visible   → TX correct (instant, stay IDLE)
     * A pressed → no tag + pos  → inertial turn → wait for tag → TX correct
     */
    private void handleTurretAutoAim() {
        if (!turretAimMode[0]) {
            prevAForTurret = false;
            aimState = AimState.IDLE;
            return;
        }

        boolean a = gp1.getButton(GamepadKeys.Button.A);

        switch (aimState) {
            case IDLE:
                if (risingEdge(a, prevAForTurret)) {
                    attemptTurretAim();
                }
                break;

            case INERTIAL_TURNING:
                if (robot.vision != null && robot.vision.hasTarget()) {
                    applyTxCorrection();
                    aimState    = AimState.TX_CORRECTING;
                    aimStartMs  = now();
                } else if (elapsed(aimStartMs) > AIM_TIMEOUT_MS) {
                    aimState = AimState.IDLE;
                    gp1.gamepad.rumble(100);
                }
                break;

            case TX_CORRECTING:
                if (elapsed(aimStartMs) > TX_SETTLE_MS) {
                    aimState = AimState.IDLE;
                    gp1.gamepad.rumble(150);
                }
                break;
        }

        prevAForTurret = a;
    }

    /** Attempt a single turret aim: TX if tag visible, else inertial. */
    private void attemptTurretAim() {
        boolean hasTag = robot.vision != null && robot.vision.hasTarget();

        if (hasTag) {
            applyTxCorrection();
            gp1.gamepad.rumble(150);
            // Stay IDLE — one-shot correction done
        } else if (robot.drive.hasAbsolutePosition() && lastSeenGoalTag != -1) {
            double targetDeg = calculateInertialTurretAngle();
            robot.turret.releaseHold();
            robot.turret.enableSoftLock(targetDeg);
            aimState   = AimState.INERTIAL_TURNING;
            aimStartMs = now();
        }
        // else: no position & no tag → nothing we can do
    }

    /** Apply one-shot TX correction to turret angle. */
    private void applyTxCorrection() {
        double corrected = robot.turret.getAngleDegrees() + robot.vision.getTx();
        robot.turret.releaseHold();
        robot.turret.enableSoftLock(corrected);
    }

    /** Inertial navigation: turret angle = field angle to goal − robot heading. */
    private double calculateInertialTurretAngle() {
        double goalX, goalY;
        if (lastSeenGoalTag == Vision.BLUE_GOAL_TAG_ID) {
            goalX = TurretConstants.blueGoalX;
            goalY = TurretConstants.blueGoalY;
        } else {
            goalX = TurretConstants.redGoalX;
            goalY = TurretConstants.redGoalY;
        }

        double dx = goalX - robot.drive.getAbsoluteX();
        double dy = goalY - robot.drive.getAbsoluteY();
        double fieldAngleDeg  = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robot.drive.getAbsoluteHeading());

        double target = fieldAngleDeg - robotHeadingDeg;
        return normalizeAngle180(target);
    }

    /** GP2 D-Pad: turret presets (−45° / +45° / 0° / 180°). */
    private void handleGp2TurretPresets() {
        boolean l = gp2.getButton(GamepadKeys.Button.DPAD_LEFT);
        boolean r = gp2.getButton(GamepadKeys.Button.DPAD_RIGHT);
        boolean u = gp2.getButton(GamepadKeys.Button.DPAD_UP);
        boolean d = gp2.getButton(GamepadKeys.Button.DPAD_DOWN);

        if (risingEdge(l, prevG2Left))   turretGoTo(-45);
        if (risingEdge(r, prevG2Right))  turretGoTo(45);
        if (risingEdge(u, prevG2Up))     turretGoTo(0);
        if (risingEdge(d, prevG2Down))   turretGoTo(180);

        prevG2Left = l;  prevG2Right = r;  prevG2Up = u;  prevG2Down = d;
    }

    /** GP1 D-Pad ←/→: open-loop turret rotation with delayed PID hold on release. */
    private void handleDpadManualControl() {
        boolean left   = gp1.getButton(GamepadKeys.Button.DPAD_LEFT);
        boolean right  = gp1.getButton(GamepadKeys.Button.DPAD_RIGHT);
        boolean active = left || right;

        if (left) {
            robot.turret.releaseHold();
            robot.turret.setPower(1.0);
        } else if (right) {
            robot.turret.releaseHold();
            robot.turret.setPower(-1.0);
        } else {
            // Just released → record time, stop motor
            if (prevDpadActive && !active) {
                dpadReleaseTime = now();
                robot.turret.releaseHold();
                robot.turret.setPower(0);
            }
            // After delay → engage PID hold
            if (dpadReleaseTime > 0 && elapsed(dpadReleaseTime) >= TURRET_HOLD_DELAY_MS
                    && !robot.turret.isHoldingPosition()) {
                robot.turret.holdCurrentPosition();
            }
        }

        prevDpadActive = active;
    }

    /** GP1 preset buttons: ↓ = 0°/180° toggle, X = −45°, B = +45°. */
    private void handleGp1TurretPresets() {
        boolean down = gp1.getButton(GamepadKeys.Button.DPAD_DOWN);
        boolean x    = gp1.getButton(GamepadKeys.Button.X);
        boolean b    = gp1.getButton(GamepadKeys.Button.B);

        if (risingEdge(down, prevDpadDown)) {
            turretAt180 = !turretAt180;
            turretGoTo(turretAt180 ? 180 : 0);
        }
        if (risingEdge(x, prevXButton)) { turretGoTo(-45); turretAt180 = false; }
        if (risingEdge(b, prevBButton)) { turretGoTo(45);  turretAt180 = false; }

        prevDpadDown = down;  prevXButton = x;  prevBButton = b;
    }

    /** Command turret to a preset angle. */
    private void turretGoTo(double angleDeg) {
        robot.turret.releaseHold();
        robot.turret.enableSoftLock(angleDeg);
    }

    // ═════════════════════════════════════════════════════════════
    //  Absolute Position Update
    // ═════════════════════════════════════════════════════════════

    private void updateAbsolutePosition() {
        if (robot.vision == null) return;

        boolean turretAtZero = robot.turret != null
                && robot.turret.isCalibrated()
                && Math.abs(robot.turret.getAngleDegrees()) < 0.4;

        int tagId = robot.vision.getDetectedTagId();
        boolean seeGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID
                           || tagId == Vision.RED_GOAL_TAG_ID);

        if (seeGoalTag && turretAtZero) {
            robot.drive.updateAbsolutePositionFromVisionWithTurret(robot.vision, 0);
        } else {
            robot.drive.updateAbsolutePositionFromOdometry();
        }
    }

    // ═════════════════════════════════════════════════════════════
    //  Intake & Shooter Flags
    // ═════════════════════════════════════════════════════════════

    private void updateIntakeShooterFlags() {
        boolean shootHeld = gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                         || gp1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                         || gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.3;
        boolean ltHeld    = gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.3;

        robot.intake.setShooting(shootHeld && ltHeld);
        robot.intake.setFastIntaking(ltHeld);
    }

    // ═════════════════════════════════════════════════════════════
    //  Telemetry
    // ═════════════════════════════════════════════════════════════

    private void emitTelemetry() {
        Pose2D pose = robot.drive.getPose();

        // Odometry
        telemetry.addData("Odo X",       fmt("%.2f in", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Odo Y",       fmt("%.2f in", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Odo Heading",  fmt("%.1f deg", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS))));

        // Vision
        telemetry.addLine("=== VISION ===");
        if (robot.vision != null) {
            int tag = robot.vision.getDetectedTagId();
            telemetry.addData("Tag ID",   tag == -1 ? "NONE" : tag);
            telemetry.addData("TX",        fmt("%.1f°", robot.vision.getTx()));
            telemetry.addData("Distance",  fmt("%.1f in", robot.vision.getDistanceToTag()));
            telemetry.addData("Aligned",   robot.drive.isAligned() ? "YES ✓" : "NO");
        } else {
            telemetry.addLine("Vision not available");
        }

        // Absolute position
        telemetry.addLine("=== ABS POSITION ===");
        if (robot.drive.hasAbsolutePosition()) {
            telemetry.addData("Abs X",       fmt("%.1f in", robot.drive.getAbsoluteX()));
            telemetry.addData("Abs Y",       fmt("%.1f in", robot.drive.getAbsoluteY()));
            telemetry.addData("Abs Heading", fmt("%.1f°",   Math.toDegrees(robot.drive.getAbsoluteHeading())));
        } else {
            telemetry.addData("Has Abs Pos", "NO");
        }

        // Aim mode
        String mode = turretAimMode[0]
                ? "TURRET [" + aimState + "]"
                : "CHASSIS (A=turn robot)";
        telemetry.addData("AIM MODE", mode);
        if (turretAimMode[0]) {
            telemetry.addData("Target Goal", goalTagName(lastSeenGoalTag));
        }
        telemetry.addLine("R-Stick Click = Toggle Aim Mode");

        // Turret
        if (robot.turret != null) {
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Angle",  fmt("%.1f°", robot.turret.getAngleDegrees()));
            telemetry.addData("Target", fmt("%.1f°", robot.turret.getTargetAngle()));
            telemetry.addData("Mode",   robot.turret.getLockMode());
            telemetry.addLine("←→=Manual | ↓=0/180 | X=-45 | B=+45");
        }

        // Emergency disable status
        telemetry.addLine("=== EMERGENCY DISABLE (GP2) ===");
        telemetry.addData("Intake",  robot.intake.isDisabled()  ? "DISABLED" : "OK");
        telemetry.addData("Shooter", robot.shooter.isDisabled() ? "DISABLED" : "OK");
        if (robot.turret != null) {
            telemetry.addData("Turret", robot.turret.isDisabled() ? "DISABLED" : "OK");
        }
        telemetry.addLine("GP2: LT+LB=Intake | RT+RB=Shooter | LB+RB=Turret");

        // Shooter
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("READY",    robot.shooter.isShooterAtSetPoint());
        telemetry.addData("STATE",    robot.shooter.shooterState);
        telemetry.addData("Velocity", fmt("%.0f TPS", robot.shooter.getVelocity()));
        telemetry.addData("BOOST",    robot.shooter.getBoostStatus());

        telemetry.update();
    }

    private void emitDashboard() {
        Pose2D pose = robot.drive.getPose();
        TelemetryPacket packet = new TelemetryPacket();
        DashboardUtil.drawRobot(packet, pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    // ═════════════════════════════════════════════════════════════
    //  Utilities
    // ═════════════════════════════════════════════════════════════

    private static boolean risingEdge(boolean current, boolean previous) {
        return current && !previous;
    }

    private boolean gp2Trigger(GamepadKeys.Trigger trigger) {
        return gp2.getTrigger(trigger) > 0.3;
    }

    private void rumbleBoth(int durationMs) {
        gp1.gamepad.rumble(durationMs);
        if (gp2 != null) gp2.gamepad.rumble(durationMs);
    }

    private static long now() {
        return System.currentTimeMillis();
    }

    private static long elapsed(long startMs) {
        return System.currentTimeMillis() - startMs;
    }

    private static double normalizeAngle180(double deg) {
        while (deg >  180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private static String fmt(String format, Object... args) {
        return String.format(format, args);
    }

    private static String goalTagName(int tagId) {
        if (tagId == Vision.BLUE_GOAL_TAG_ID) return "BLUE";
        if (tagId == Vision.RED_GOAL_TAG_ID)  return "RED";
        return "NONE";
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
