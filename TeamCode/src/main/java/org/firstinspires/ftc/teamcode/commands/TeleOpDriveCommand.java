package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivePinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/**
 * TeleOp drive command — field-centric Mecanum with one-shot chassis auto-aim.
 *
 * <h3>Auto-aim flow (A button, chassis mode only)</h3>
 * <ol>
 *   <li>A pressed → snapshot current TX → compute target heading</li>
 *   <li>PID turns chassis to captured heading</li>
 *   <li>Stops on: target reached, manual override, timeout, or no heading captured</li>
 * </ol>
 *
 * <p>When {@code turretAimMode[0]} is {@code true}, A-button chassis auto-aim
 * is disabled (Solo.java handles turret auto-aim instead).</p>
 */
public class TeleOpDriveCommand extends CommandBase {

    // ── Dependencies ────────────────────────────────────────────
    private final MecanumDrivePinpoint drive;
    private final Vision  vision;
    private final Turret  turret;
    private final GamepadEx gp1;
    private final GamepadEx gp2;           // secondary gamepad (for rumble)
    private final boolean[] isAuto;

    // ── Config ──────────────────────────────────────────────────
    private static final double TRIGGER_THRESHOLD    = 0.3;
    private static final long   AUTO_AIM_TIMEOUT_MS  = 2000;
    private static final double TX_RUMBLE_THRESHOLD   = 5.0;   // degrees

    // ── Auto-aim state ──────────────────────────────────────────
    private boolean autoAimActive   = false;
    private boolean headingCaptured = false;
    private double  targetHeadingRad;
    private long    autoAimStartMs;
    private boolean lastA;

    // ── External flag from Solo.java ────────────────────────────
    private boolean[] turretAimMode = {false};

    // ═════════════════════════════════════════════════════════════
    //  Construction
    // ═════════════════════════════════════════════════════════════

    public TeleOpDriveCommand(MecanumDrivePinpoint drive, Vision vision, Turret turret,
                              GamepadEx gp1, GamepadEx gp2, boolean[] isAuto) {
        this.drive   = drive;
        this.vision  = vision;
        this.turret  = turret;
        this.gp1     = gp1;
        this.gp2     = gp2;
        this.isAuto  = isAuto;
        addRequirements(drive);
    }

    public TeleOpDriveCommand(MecanumDrivePinpoint drive, Vision vision, Turret turret,
                              GamepadEx gp1, boolean[] isAuto) {
        this(drive, vision, turret, gp1, null, isAuto);
    }

    public void setTurretAimMode(boolean[] mode) { turretAimMode = mode; }

    // ═════════════════════════════════════════════════════════════
    //  Execute — called every loop
    // ═════════════════════════════════════════════════════════════

    @Override
    public void execute() {
        if (isAuto[0]) return;

        double rawX  = -gp1.getLeftX();
        double rawY  =  gp1.getLeftY();
        double rawRX =  gp1.getRightX();

        updateAutoAim(rawRX);

        boolean shouldAlign = autoAimActive && headingCaptured;
        boolean hasInput = aboveDeadband(rawX) || aboveDeadband(rawY)
                        || aboveDeadband(rawRX) || shouldAlign;

        if (!hasInput) { drive.setGamepad(false); return; }

        drive.setGamepad(true);
        double fwd    = rawY * Math.abs(rawY);
        double strafe = rawX * Math.abs(rawX);

        boolean useChassisAim = shouldAlign
                && (turret == null || turret.getLockMode() != Turret.LockMode.HARD_LOCK);

        double turn = useChassisAim
                ? drive.getTurnPowerToHeading(targetHeadingRad)
                : rawRX * Math.abs(rawRX);

        turn = Math.max(-1, Math.min(1, turn));
        drive.moveRobotFieldRelative(fwd, strafe, turn);
    }

    // ═════════════════════════════════════════════════════════════
    //  Auto-aim state machine
    // ═════════════════════════════════════════════════════════════

    private void updateAutoAim(double rawRX) {
        boolean a = gp1.getButton(GamepadKeys.Button.A);

        // ── Rising edge: start / cancel ─────────────────────────
        if (!turretAimMode[0] && a && !lastA) {
            if (autoAimActive) {
                cancelAutoAim();
            } else {
                startAutoAim();
            }
        }
        lastA = a;

        // ── Termination conditions ──────────────────────────────
        if (!autoAimActive) return;

        boolean manualOverride = Math.abs(rawRX) > 0.1;
        boolean timeout        = elapsed(autoAimStartMs) > AUTO_AIM_TIMEOUT_MS;
        boolean reached        = headingCaptured && drive.isAtTargetHeading(targetHeadingRad);

        if (reached || manualOverride || timeout || !headingCaptured) {
            if (reached) tryRumble();
            cancelAutoAim();
        }
    }

    private void startAutoAim() {
        autoAimActive  = true;
        headingCaptured = false;
        autoAimStartMs = now();

        if (vision != null) {
            int id = vision.getDetectedTagId();
            boolean goal = (id == Vision.BLUE_GOAL_TAG_ID || id == Vision.RED_GOAL_TAG_ID);
            if (goal) {
                double tx = vision.getTx();
                targetHeadingRad = drive.getHeading() - Math.toRadians(tx);
                headingCaptured = true;
            }
        }
    }

    private void cancelAutoAim() {
        autoAimActive   = false;
        headingCaptured = false;
    }

    /** Rumble both gamepads when TX confirms alignment. */
    private void tryRumble() {
        if (vision == null) return;
        int id = vision.getDetectedTagId();
        boolean goal = (id == Vision.BLUE_GOAL_TAG_ID || id == Vision.RED_GOAL_TAG_ID);
        if (goal && Math.abs(vision.getTx()) < TX_RUMBLE_THRESHOLD) {
            gp1.gamepad.rumble(200);
            if (gp2 != null) gp2.gamepad.rumble(200);
        }
    }

    // ═════════════════════════════════════════════════════════════
    //  Utilities
    // ═════════════════════════════════════════════════════════════

    private static boolean aboveDeadband(double v) {
        return Math.abs(v) > DriveConstants.deadband;
    }

    private static long now()             { return System.currentTimeMillis(); }
    private static long elapsed(long ms)  { return System.currentTimeMillis() - ms; }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
