package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Util;

/**
 * Shooter subsystem — dual-flywheel velocity control with firing boost.
 *
 * <h3>Control Strategy: Pseudo Closed-Loop</h3>
 * <ul>
 *   <li>Far from target → full power acceleration with progressive slowdown</li>
 *   <li>Within deadband → feedforward + PD fine-tuning</li>
 *   <li>Overspeed → reduced power (no reverse braking)</li>
 *   <li>Firing → locked power + linear boost ramp</li>
 * </ul>
 */
public class Shooter extends SubsystemBase {

    // ── Hardware ────────────────────────────────────────────────
    public final DcMotorEx rightShooter;
    public final DcMotorEx leftShooter;
    public final Servo     shooterServo;

    // ── State ───────────────────────────────────────────────────
    public ShooterState shooterState = ShooterState.STOP;
    private ShooterState lastState   = ShooterState.STOP;
    private boolean disabled         = false;

    // ── Adaptive overrides (0 / −1 = use state defaults) ────────
    private double adaptiveVelocity     = 0;
    private double adaptiveServoPosition = -1;

    // ── 50 ms window velocity ───────────────────────────────────
    private int    windowStartPos  = 0;
    private long   windowStartTime = 0;
    private double calculatedVelocity = 0;
    private static final long VELOCITY_WINDOW_MS = 50;

    // ── Firing boost ────────────────────────────────────────────
    private long    firingStartTime = 0;
    private boolean isFiring        = false;
    private boolean transitFiring   = false;
    private boolean boostActive     = false;
    private double  lockedPower     = 0;

    private static final long   BOOST_DELAY_MS     = 200;
    private static final long   BOOST_RAMP_MS      = 1000;
    private static final long   BOOST_MAX_MS       = 1000;
    // Linear ramp ranges (start% → end%)
    private static final double BOOST_SLOW_START = 0.30, BOOST_SLOW_END = 0.32;
    private static final double BOOST_MID_START  = 0.27, BOOST_MID_END  = 0.38;
    private static final double BOOST_FAST_START = 0.30, BOOST_FAST_END = 0.40;

    // ── Fine-tuning PD (within deadband) ────────────────────────
    private double lastError = 0;

    // ── PID controllers (unused by pseudo-CL, kept for alt modes)
    public final PIDController  pidController;
    public final PIDFController velocityPIDF;

    // ═════════════════════════════════════════════════════════════
    //  Shooter State Enum
    // ═════════════════════════════════════════════════════════════

    public enum ShooterState {
        STOP(ShooterConstants.stopVelocity, ShooterConstants.shooterServoMidPos),
        SLOW(ShooterConstants.slowVelocity, ShooterConstants.shooterServoDownPos),
        MID (ShooterConstants.midVelocity,  ShooterConstants.shooterServoMidPos),
        FAST(ShooterConstants.fastVelocity, ShooterConstants.shooterServoUpPos);

        final double shooterVelocity, shooterServoPos;

        ShooterState(double vel, double servo) {
            this.shooterVelocity = vel;
            this.shooterServoPos = servo;
        }
    }

    // ═════════════════════════════════════════════════════════════
    //  Construction
    // ═════════════════════════════════════════════════════════════

    public Shooter(final HardwareMap hardwareMap) {
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        leftShooter  = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        shooterServo = hardwareMap.get(Servo.class,     ShooterConstants.shooterServoName);

        pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        velocityPIDF  = new PIDFController(ShooterConstants.kP, ShooterConstants.kI,
                                           ShooterConstants.kD, ShooterConstants.kF);
    }

    // ═════════════════════════════════════════════════════════════
    //  Public API
    // ═════════════════════════════════════════════════════════════

    public void setShooterState(ShooterState state) {
        shooterState = state;
        adaptiveVelocity      = 0;
        adaptiveServoPosition = -1;
    }

    public void   setAdaptiveVelocity(double v)     { adaptiveVelocity = v; }
    public double getAdaptiveVelocity()              { return adaptiveVelocity; }
    public void   setAdaptiveServoPosition(double p) { adaptiveServoPosition = p; }
    public double getAdaptiveServoPosition()         { return adaptiveServoPosition; }

    public void    setDisabled(boolean d) { disabled = d; }
    public boolean isDisabled()           { return disabled; }
    public void    toggleDisabled()       { disabled = !disabled; }

    public double getVelocity()       { return calculatedVelocity; }
    public double getTargetVelocity() { return shooterState.shooterVelocity; }

    /**
     * Notify the shooter that firing has started/stopped.
     * Resets boost state on both transitions.
     */
    public void setTransitFiring(boolean firing) {
        transitFiring = firing;
        if (firing) {
            firingStartTime = System.currentTimeMillis();
            boostActive = false;
            lockedPower = 0;
        } else {
            boostActive     = false;
            lockedPower     = 0;
            isFiring        = false;
            firingStartTime = 0;
        }
    }

    /**
     * True if velocity is within tolerance of the target.
     * Once reached, stays true through minor dips (for continuous firing).
     */
    public boolean isShooterAtSetPoint() {
        double target = effectiveTargetVelocity();
        if (target == ShooterConstants.stopVelocity && adaptiveVelocity == 0) {
            resetFiringState();
            return false;
        }

        double epsilon = (shooterState == ShooterState.FAST)
                ? ShooterConstants.shooterEpsilonFast
                : ShooterConstants.shooterEpsilon;

        boolean atTarget = Util.epsilonEqual(calculatedVelocity, target, epsilon);

        if (atTarget) {
            if (!isFiring) {
                isFiring        = true;
                firingStartTime = System.currentTimeMillis();
            }
            return true;
        }
        return isFiring;     // keep returning true during minor velocity dips
    }

    public String getBoostStatus() {
        if (boostActive) {
            return "ACTIVE (+" + (int)(boostAmount() * 100) + "% ramping)";
        }
        if (isFiring && transitFiring) {
            long elapsed = System.currentTimeMillis() - firingStartTime;
            if (elapsed < BOOST_DELAY_MS)
                return "WAITING (" + elapsed + "/" + BOOST_DELAY_MS + "ms)";
        }
        if (isFiring)       return "isFiring (no LT+bumper)";
        if (transitFiring)  return "transitFiring (no isFiring)";
        return "OFF";
    }

    // ═════════════════════════════════════════════════════════════
    //  Periodic — main control loop
    // ═════════════════════════════════════════════════════════════

    @Override
    public void periodic() {
        if (disabled) { setMotors(0); return; }
        initVelocityWindow();
        updateVelocity();

        double target = effectiveTargetVelocity();
        double power;

        handleStateChange();
        handleBoostTimeout();
        if (!transitFiring) resetBoost();

        if (isStopState()) {
            power = stopModePower();
        } else if (boostActive) {
            power = boostedPower();
        } else {
            power = pseudoClosedLoop(target);
            power = maybeActivateBoost(power);
        }

        applyMotors(power, effectiveServoPos());
    }

    // ═════════════════════════════════════════════════════════════
    //  Control Logic (private)
    // ═════════════════════════════════════════════════════════════

    /** STOP state: idle power, or reverse-brake if way over idle. */
    private double stopModePower() {
        resetBoost();
        return calculatedVelocity > ShooterConstants.stopVelocity + 10000
                ? -0.3
                : ShooterConstants.idlePower;
    }

    /** Boost mode: locked power + linear ramp. */
    private double boostedPower() {
        return Math.min(1.0, lockedPower + boostAmount());
    }

    /**
     * Pseudo closed-loop velocity control.
     * <ul>
     *   <li>error &gt; deadband → progressive acceleration</li>
     *   <li>|error| ≤ deadband → feedforward + PD fine control</li>
     *   <li>overspeed → reduced power</li>
     * </ul>
     */
    private double pseudoClosedLoop(double target) {
        double current = calculatedVelocity;
        double error   = target - current;
        double ff      = feedforward(target);

        boolean fast = (shooterState == ShooterState.FAST);
        boolean mid  = (shooterState == ShooterState.MID);
        double deadband  = fast ? 4500 : 8000;
        double kPf       = fast ? 0.000012 : 0.000008;
        double kDf       = fast ? 0.00005  : 0.0;

        double errorDeriv = (error - lastError) / 0.02;   // ~20 ms loop
        lastError = error;

        double power;

        if (error > deadband) {
            power = acceleratePower(error, ff, mid);
        } else if (error < -ShooterConstants.motorBrakeThreshold) {
            power = ff * 0.7;                               // gentle slow-down
        } else if (Math.abs(error) <= deadband) {
            power = ff + error * kPf + errorDeriv * kDf;    // PD fine tune
        } else {
            power = ff;                                     // slight overspeed
        }

        return clamp(power, 0, 0.95);
    }

    /** Progressive acceleration from full power down to feedforward. */
    private double acceleratePower(double error, double ff, boolean mid) {
        double target = ff - 0.03;
        double approach = mid ? 0.7 : 0.85;

        if      (error > 50000) return clamp(1.0,      0.3, 0.95);
        else if (error > 40000) return clamp(approach,  0.3, 0.95);
        else if (error > 20000) {
            double t = (40000 - error) / 20000.0;
            return clamp(approach - (approach - target) * t, 0.3, 0.95);
        }
        return clamp(target, 0.3, 0.95);
    }

    /** Activate boost 200 ms after firing starts (if eligible). */
    private double maybeActivateBoost(double power) {
        if (isFiring && transitFiring
                && elapsed(firingStartTime) > BOOST_DELAY_MS) {
            lockedPower = power;
            boostActive = true;
            return Math.min(1.0, power + boostAmount());
        }
        return power;
    }

    // ═════════════════════════════════════════════════════════════
    //  Boost Calculation
    // ═════════════════════════════════════════════════════════════

    /** Linear ramp from startBoost → endBoost over 1 s. */
    private double boostAmount() {
        long elapsed = Math.max(0, elapsed(firingStartTime) - BOOST_DELAY_MS);
        double t = Math.min(1.0, (double) elapsed / BOOST_RAMP_MS);

        double s, e;
        switch (shooterState) {
            case SLOW: s = BOOST_SLOW_START; e = BOOST_SLOW_END; break;
            case FAST: s = BOOST_FAST_START; e = BOOST_FAST_END; break;
            default:   s = BOOST_MID_START;  e = BOOST_MID_END;  break;
        }
        return s + (e - s) * t;
    }

    // ═════════════════════════════════════════════════════════════
    //  Velocity Window
    // ═════════════════════════════════════════════════════════════

    private void initVelocityWindow() {
        if (windowStartTime == 0) {
            windowStartPos  = rightShooter.getCurrentPosition();
            windowStartTime = System.currentTimeMillis();
        }
    }

    private void updateVelocity() {
        int  pos = rightShooter.getCurrentPosition();
        long now = System.currentTimeMillis();
        long dt  = now - windowStartTime;
        if (dt >= VELOCITY_WINDOW_MS) {
            double raw = Math.abs(pos - windowStartPos) * 1000.0 / dt;
            calculatedVelocity = ShooterConstants.filterAlpha * raw
                    + (1 - ShooterConstants.filterAlpha) * calculatedVelocity;
            windowStartPos  = pos;
            windowStartTime = now;
        }
    }

    // ═════════════════════════════════════════════════════════════
    //  Helpers
    // ═════════════════════════════════════════════════════════════

    private double effectiveTargetVelocity() {
        return adaptiveVelocity != 0 ? adaptiveVelocity : shooterState.shooterVelocity;
    }

    private double effectiveServoPos() {
        return adaptiveServoPosition >= 0 ? adaptiveServoPosition : shooterState.shooterServoPos;
    }

    private boolean isStopState() {
        return shooterState == ShooterState.STOP && adaptiveVelocity == 0;
    }

    private double feedforward(double target) {
        return Math.min(0.95, (target / ShooterConstants.maxVelocityTPS) * 1.3);
    }

    private void handleStateChange() {
        if (shooterState != lastState) {
            resetBoost();
            lastState = shooterState;
        }
    }

    private void handleBoostTimeout() {
        if (boostActive && elapsed(firingStartTime) > BOOST_DELAY_MS + BOOST_MAX_MS) {
            resetBoost();
        }
    }

    private void resetBoost()       { boostActive = false; lockedPower = 0; }
    private void resetFiringState() { isFiring = false; firingStartTime = 0; resetBoost(); }

    private void setMotors(double p) {
        leftShooter.setPower(p);
        rightShooter.setPower(-p);
    }

    private void applyMotors(double power, double servo) {
        leftShooter.setPower(power);
        rightShooter.setPower(-power);
        shooterServo.setPosition(servo);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static long elapsed(long start) {
        return System.currentTimeMillis() - start;
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
