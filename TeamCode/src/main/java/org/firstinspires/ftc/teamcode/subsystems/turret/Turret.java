package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Turret (Gimbal) subsystem — single motor, through-bore encoder, PID position control.
 *
 * <h3>Encoder</h3>
 * REV Through Bore Encoder V2 — 8192 CPR, read via {@code turretMotor.getCurrentPosition()}.
 *
 * <h3>Gear Ratio</h3>
 * Motor 116 turns → Turret 22 turns  ⟹  external ratio = 116/22 ≈ 5.2727
 *
 * <h3>Coordinate System</h3>
 * <ul>
 *   <li>0° = turret facing forward</li>
 *   <li>Positive = clockwise (right)</li>
 *   <li>Negative = counterclockwise (left)</li>
 * </ul>
 *
 * <h3>Motor–Angle Relationship (critical!)</h3>
 * Positive motor power → angle <b>decreases</b> (turns left).<br>
 * Negative motor power → angle <b>increases</b> (turns right).
 */
public class Turret extends SubsystemBase {

    // ── Hardware ────────────────────────────────────────────────
    public final DcMotorEx turretMotor;

    // ── PID ─────────────────────────────────────────────────────
    private final PIDFController positionPIDF;

    // ── Manual mode ─────────────────────────────────────────────
    private double targetPower = 0;

    // ── Lock modes ──────────────────────────────────────────────
    public enum LockMode { MANUAL, SOFT_LOCK, HARD_LOCK }
    private LockMode lockMode = LockMode.MANUAL;

    // ── Position hold (MANUAL brake) ────────────────────────────
    private boolean holdingPosition = false;
    private int     holdTicks       = 0;

    // ── Soft-lock target ────────────────────────────────────────
    private double targetAngleDeg = 0;

    // ── Hard-lock: goal tracking ────────────────────────────────
    public enum Alliance { BLUE, RED }
    private Alliance currentAlliance = Alliance.BLUE;
    private double   goalX, goalY;
    private boolean  goalSet = false;

    // ── Robot pose (updated externally) ─────────────────────────
    private double robotX, robotY;
    private double robotHeading;                 // radians

    // ── Flip / Unwind state ─────────────────────────────────────
    private boolean isUnwinding        = false;
    private double  flipTargetAngle    = 0;

    // ── TX tracking (hard lock, tag visible) ────────────────────
    private double  currentTx          = 0;
    private boolean hasValidTx         = false;
    private int     currentDetectedTag = -1;
    private int     targetTagId        = 20;     // blue default

    // TX low-pass filter
    private double  filteredTx         = 0;
    private boolean txFilterInit       = false;
    public static double txFilterAlpha = 0.3;

    // ── Encoder calibration ─────────────────────────────────────
    private int     encoderOffset;
    private int     homeEncoderPos     = 0;
    private int     startupEncoderPos  = 0;
    private boolean isCalibrated       = false;

    // ── Emergency disable ───────────────────────────────────────
    private boolean disabled = false;

    // ── Manual aim offset ───────────────────────────────────────
    private double manualAimOffset = 0.0;
    public static double manualOffsetSpeed = 1.5;

    // ═════════════════════════════════════════════════════════════
    //  Construction
    // ═════════════════════════════════════════════════════════════

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, TurretConstants.turretMotorName);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(TurretConstants.reverseMotor
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        startupEncoderPos = turretMotor.getCurrentPosition();
        encoderOffset     = startupEncoderPos;
        isCalibrated      = true;

        positionPIDF = new PIDFController(
                TurretConstants.kP, TurretConstants.kI,
                TurretConstants.kD, TurretConstants.kF);
    }

    // ═════════════════════════════════════════════════════════════
    //  Manual Power (respects limits & unwind)
    // ═════════════════════════════════════════════════════════════

    public void setPower(double power) {
        if (isUnwinding) return;
        power = applySoftwareLimits(power, getAngleDegrees());
        targetPower = clampPower(power);
    }

    public void stop() {
        if (!isUnwinding) targetPower = 0;
    }

    public void rotateLeft(double speed)  { setPower(-Math.abs(speed)); }
    public void rotateRight(double speed) { setPower( Math.abs(speed)); }

    /** Bypasses PID/lock but still respects limits. */
    public void setMotorPower(double power) {
        if (disabled) { turretMotor.setPower(0); return; }
        power = applySoftwareLimits(power, getAngleDegrees());
        turretMotor.setPower(power);
    }

    // ═════════════════════════════════════════════════════════════
    //  Position Hold (MANUAL mode brake)
    // ═════════════════════════════════════════════════════════════

    public void holdCurrentPosition() {
        if (isUnwinding) return;
        holdTicks       = getEncoderPosition();
        holdingPosition = true;
    }

    public void releaseHold()           { holdingPosition = false; }
    public boolean isHoldingPosition()  { return holdingPosition; }

    // ═════════════════════════════════════════════════════════════
    //  Encoder & Angle
    // ═════════════════════════════════════════════════════════════

    public int getEncoderPosition() {
        return turretMotor.getCurrentPosition() - encoderOffset;
    }

    public int getRawEncoderPosition()    { return turretMotor.getCurrentPosition(); }
    public int getHomeEncoderPosition()   { return homeEncoderPos; }
    public int getStartupEncoderPosition(){ return startupEncoderPos; }

    /**
     * Current turret angle in degrees.
     * <pre>
     * ticks → motorRotations → turretRotations → degrees
     * </pre>
     */
    public double getAngleDegrees() {
        int ticks = getEncoderPosition();
        double motorRot  = (double) ticks / TurretConstants.ENCODER_CPR;
        double turretRot = motorRot / TurretConstants.GEAR_RATIO;
        return TurretConstants.ANGLE_OFFSET - turretRot * 360.0;
    }

    public double getAngleRadians() { return Math.toRadians(getAngleDegrees()); }

    public void resetEncoder() {
        encoderOffset = turretMotor.getCurrentPosition();
        isCalibrated  = true;
    }

    public void setCurrentAsHome() {
        homeEncoderPos = turretMotor.getCurrentPosition();
        encoderOffset  = homeEncoderPos;
        isCalibrated   = true;
    }

    public void goToHome() {
        int relTicks = homeEncoderPos - encoderOffset;
        enableSoftLock(ticksToDegrees(relTicks));
    }

    public boolean isCalibrated() { return isCalibrated; }

    // ═════════════════════════════════════════════════════════════
    //  Emergency Disable
    // ═════════════════════════════════════════════════════════════

    public void setDisabled(boolean d) { disabled = d; }
    public boolean isDisabled()        { return disabled; }
    public void toggleDisabled()       { disabled = !disabled; }

    // ═════════════════════════════════════════════════════════════
    //  Geometry — Limelight offset
    // ═════════════════════════════════════════════════════════════

    /** Distance from Limelight to chassis center (mm). */
    public double getLimelightDistanceFromCenterMM() {
        double θ  = getAngleRadians();
        double Lx = -TurretConstants.turretOffsetMM + TurretConstants.limelightOffsetMM * Math.cos(θ);
        double Ly = TurretConstants.limelightOffsetMM * Math.sin(θ);
        return Math.sqrt(Lx * Lx + Ly * Ly);
    }

    public double getLimelightDistanceFromCenterInches() {
        return getLimelightDistanceFromCenterMM() / 25.4;
    }

    public double getTargetPower() { return targetPower; }

    // ═════════════════════════════════════════════════════════════
    //  Lock Modes
    // ═════════════════════════════════════════════════════════════

    /**
     * SOFT LOCK: hold turret at a fixed angle using PID.
     * @param angleDeg target angle (0 = forward)
     */
    public void enableSoftLock(double angleDeg) {
        if (isUnwinding) return;
        targetAngleDeg = clampAngle(angleDeg);
        lockMode = LockMode.SOFT_LOCK;
        resetPID();
    }

    /** SOFT LOCK at 0°. */
    public void enableSoftLock() { enableSoftLock(0); }

    /** Set alliance — determines goal position and TX tracking tag. */
    public void setAlliance(Alliance alliance) {
        currentAlliance = alliance;
        if (alliance == Alliance.BLUE) {
            goalX = TurretConstants.blueGoalX;  goalY = TurretConstants.blueGoalY;
            targetTagId = 20;
        } else {
            goalX = TurretConstants.redGoalX;   goalY = TurretConstants.redGoalY;
            targetTagId = 24;
        }
        goalSet = true;
    }

    public Alliance getAlliance() { return currentAlliance; }

    /** HARD LOCK — goal tracking. */
    public void enableHardLock() {
        if (isUnwinding) return;
        if (!goalSet) setAlliance(Alliance.BLUE);
        lockMode = LockMode.HARD_LOCK;
        resetPID();
    }

    public void enableHardLockBlue() { if (!isUnwinding) { setAlliance(Alliance.BLUE); lockMode = LockMode.HARD_LOCK; resetPID(); } }
    public void enableHardLockRed()  { if (!isUnwinding) { setAlliance(Alliance.RED);  lockMode = LockMode.HARD_LOCK; resetPID(); } }

    public void enableHardLock(double targetX, double targetY) {
        if (isUnwinding) return;
        goalX = targetX; goalY = targetY; goalSet = true;
        lockMode = LockMode.HARD_LOCK;
        resetPID();
    }

    public void disableLock() {
        if (isUnwinding) return;
        lockMode = LockMode.MANUAL;
        positionPIDF.reset();
    }

    public LockMode getLockMode() { return lockMode; }
    public boolean isLocked()     { return lockMode != LockMode.MANUAL; }

    // ═════════════════════════════════════════════════════════════
    //  Manual Aim Offset
    // ═════════════════════════════════════════════════════════════

    public void adjustManualOffset(double delta) {
        manualAimOffset = Math.max(-90, Math.min(90, manualAimOffset + delta));
    }
    public void   resetManualOffset() { manualAimOffset = 0; }
    public double getManualOffset()   { return manualAimOffset; }

    // ═════════════════════════════════════════════════════════════
    //  Robot Position & TX Updates (called externally each frame)
    // ═════════════════════════════════════════════════════════════

    public void updateRobotPosition(double x, double y, double heading) {
        robotX = x; robotY = y; robotHeading = heading;
    }

    /**
     * Update TX from Vision.
     * @param tx       horizontal offset (degrees, + = target right)
     * @param valid    true if tag visible
     * @param tagId    detected AprilTag ID (-1 = none)
     */
    public void updateTx(double tx, boolean valid, int tagId) {
        currentDetectedTag = tagId;
        hasValidTx = valid;
        if (valid) {
            if (!txFilterInit) { filteredTx = tx; txFilterInit = true; }
            else               { filteredTx = txFilterAlpha * tx + (1 - txFilterAlpha) * filteredTx; }
            currentTx = filteredTx;
        } else {
            txFilterInit = false;
            currentDetectedTag = -1;
        }
    }

    @Deprecated
    public void updateTx(double tx, boolean valid) { updateTx(tx, valid, -1); }

    public boolean hasValidTx()           { return hasValidTx; }
    public double  getFilteredTx()        { return filteredTx; }
    public int     getCurrentDetectedTagId() { return currentDetectedTag; }
    public int     getTargetTagId()       { return targetTagId; }

    /** TX tracking active iff seeing tag 24 and not unwinding. */
    public boolean isTxTrackingActive() {
        return hasValidTx && currentDetectedTag == 24 && !isUnwinding;
    }

    public String getTrackingModeString() {
        if (lockMode != LockMode.HARD_LOCK)            return lockMode.toString();
        if (isUnwinding)                                return String.format("FLIPPING→%.0f°", flipTargetAngle);
        if (isTxTrackingActive())                       return "TX_TRACKING";
        return "INERTIAL";
    }

    // ═════════════════════════════════════════════════════════════
    //  Goal Geometry
    // ═════════════════════════════════════════════════════════════

    /** TX offset so turret aims at basket centre instead of AprilTag. */
    public double calculateTxOffsetToBasket() {
        double tagX, tagY, gX, gY;
        if (currentAlliance == Alliance.BLUE) {
            tagX = TurretConstants.blueTagX;  tagY = TurretConstants.blueTagY;
            gX   = TurretConstants.blueGoalX; gY   = TurretConstants.blueGoalY;
        } else {
            tagX = TurretConstants.redTagX;   tagY = TurretConstants.redTagY;
            gX   = TurretConstants.redGoalX;  gY   = TurretConstants.redGoalY;
        }
        double angleToTag  = Math.toDegrees(Math.atan2(tagY - robotY, tagX - robotX));
        double angleToGoal = Math.toDegrees(Math.atan2(gY   - robotY, gX   - robotX));
        return normalise180(-(angleToGoal - angleToTag));
    }

    /**
     * Required turret angle to aim at goal (Pedro Pathing coordinates).
     * <pre>
     * fieldAngle = atan2(dy, dx)       (CCW +)
     * turretAngle = robotHeading − fieldAngle   (CW +)
     * </pre>
     */
    public double calculateAngleToGoal() {
        double dir   = Math.atan2(goalY - robotY, goalX - robotX);
        double angle = Math.toDegrees(robotHeading - dir);
        angle = normalise180(angle);
        return clampAngle(angle);
    }

    public double getDistanceToGoal() {
        double dx = goalX - robotX, dy = goalY - robotY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public boolean isOnTarget() {
        return lockMode == LockMode.HARD_LOCK
            && Math.abs(getAngleDegrees() - calculateAngleToGoal()) <= TurretConstants.positionTolerance;
    }

    public boolean isAtLockPosition() {
        return lockMode == LockMode.SOFT_LOCK
            && Math.abs(getAngleDegrees() - targetAngleDeg) <= TurretConstants.positionTolerance;
    }

    public boolean isTargetReachable(double a) {
        return a >= TurretConstants.minAngleDeg && a <= TurretConstants.maxAngleDeg;
    }

    // ── Flip / Unwind helpers ───────────────────────────────────

    public boolean shouldUnwind(double raw) {
        return Math.abs(raw) >= TurretConstants.unwindThreshold;
    }

    public double calculateFlipTarget(double raw) {
        if (raw >=  TurretConstants.unwindThreshold) return raw - 360;
        if (raw <= -TurretConstants.unwindThreshold) return raw + 360;
        return raw;
    }

    public boolean isUnwinding()     { return isUnwinding; }
    public void    forceStopUnwind() { isUnwinding = false; }

    // ── Target angle query ──────────────────────────────────────

    public double getTargetAngle() {
        return lockMode == LockMode.HARD_LOCK ? calculateAngleToGoal() : targetAngleDeg;
    }

    public double getPositionError() { return getTargetAngle() - getAngleDegrees(); }

    // ── Legacy wrappers ─────────────────────────────────────────

    /** @deprecated Use {@link #enableSoftLock(double)}. */
    @Deprecated public void    setTargetAngle(double a)     { enableSoftLock(a); }
    /** @deprecated Use {@link #disableLock()}. */
    @Deprecated public void    disablePositionControl()     { disableLock(); }
    /** @deprecated Use {@link #getLockMode()}. */
    @Deprecated public boolean isPositionControlEnabled()   { return lockMode == LockMode.SOFT_LOCK; }

    // ═════════════════════════════════════════════════════════════
    //  Periodic — main control loop
    // ═════════════════════════════════════════════════════════════

    @Override
    public void periodic() {
        if (disabled) { turretMotor.setPower(0); return; }

        double current = getAngleDegrees();
        double output;

        switch (lockMode) {
            case SOFT_LOCK:
                output = runSoftLock(current);
                break;
            case HARD_LOCK:
                output = runHardLock(current);
                break;
            default: // MANUAL
                output = runManual(current);
                break;
        }

        // Final software-limit safety net
        output = applySoftwareLimits(output, current);
        turretMotor.setPower(output);
    }

    // ═════════════════════════════════════════════════════════════
    //  Control Modes (private)
    // ═════════════════════════════════════════════════════════════

    /** SOFT LOCK: PID to fixed target angle. */
    private double runSoftLock(double current) {
        if (!isCalibrated) return 0;
        return pidToAngle(current, targetAngleDeg);
    }

    /** HARD LOCK: TX tracking or inertial, with flip protection. */
    private double runHardLock(double current) {
        if (!isCalibrated) return 0;

        // ── Active flip: must complete before anything else ─────
        if (isUnwinding) {
            double result = driveFlip(current);
            if (!isUnwinding) {
                // Flip just completed → fall through to normal logic
            } else {
                return result;
            }
        }

        // ── Determine desired angle ────────────────────────────
        double raw;
        boolean canUseTx = hasValidTx && (currentDetectedTag == 24);

        if (canUseTx) {
            double txOffset = calculateTxOffsetToBasket();
            raw = current + (currentTx - txOffset);
        } else {
            raw = calculateAngleToGoal();
        }
        raw += manualAimOffset;

        // ── Flip check ─────────────────────────────────────────
        boolean outOfBounds = current < TurretConstants.minAngleDeg
                           || current > TurretConstants.maxAngleDeg;
        if (shouldUnwind(raw) || outOfBounds) {
            if (!isUnwinding) {
                isUnwinding     = true;
                flipTargetAngle = calculateFlipTarget(raw);
            }
            return pidToAngle(current, clampAngle(flipTargetAngle));
        }

        // ── Normal PID ─────────────────────────────────────────
        double desired = clampAngle(raw);
        double error   = desired - current;

        // Shortest-path normalisation (only if safe)
        if (!outOfBounds) {
            double norm = normalise180(error);
            double shortTarget = current + norm;
            if (isTargetReachable(shortTarget)) error = norm;
        }

        return pidToAngle(current, current + error);
    }

    /** MANUAL: direct power, or PID position-hold when D-pad released. */
    private double runManual(double current) {
        if (holdingPosition) {
            return pidToTicks(getEncoderPosition(), holdTicks);
        }
        return targetPower;
    }

    /** Drive toward flip target; clear {@code isUnwinding} when close. */
    private double driveFlip(double current) {
        double err = flipTargetAngle - current;
        if (Math.abs(err) <= TurretConstants.positionTolerance * 2) {
            isUnwinding = false;
            return 0;
        }
        return pidToAngle(current, clampAngle(flipTargetAngle));
    }

    // ═════════════════════════════════════════════════════════════
    //  PID Helpers (single source of truth)
    // ═════════════════════════════════════════════════════════════

    /**
     * Degree-based PID.  Handles tolerance dead-zone, F compensation, output clamping.
     * Used by SOFT_LOCK, HARD_LOCK, and flip.
     */
    private double pidToAngle(double current, double target) {
        double error = target - current;
        if (Math.abs(error) <= TurretConstants.positionTolerance) return 0;

        positionPIDF.setPIDF(TurretConstants.kP_deg, TurretConstants.kI_deg,
                             TurretConstants.kD_deg, 0);
        double pid = -positionPIDF.calculate(current, target);           // negate (motor polarity)
        double ff  = (error > 0) ? -TurretConstants.kF_deg              // static friction
                                 :  TurretConstants.kF_deg;
        return clampOutput(pid + ff);
    }

    /**
     * Tick-based PID for MANUAL hold.  Uses the original tick-tuned constants.
     */
    private double pidToTicks(int current, int target) {
        double error = target - current;
        if (Math.abs(error) <= 100) return 0;                           // ~0.83° dead-zone

        positionPIDF.setPIDF(TurretConstants.kP, TurretConstants.kI,
                             TurretConstants.kD, 0);
        double pid = positionPIDF.calculate(current, target);
        double ff  = (error > 0) ?  TurretConstants.kF
                                 : -TurretConstants.kF;
        return clampOutput(pid + ff);
    }

    // ═════════════════════════════════════════════════════════════
    //  Utilities
    // ═════════════════════════════════════════════════════════════

    private void resetPID() {
        positionPIDF.setPIDF(TurretConstants.kP, TurretConstants.kI,
                             TurretConstants.kD, TurretConstants.kF);
        positionPIDF.reset();
    }

    /** Clamp angle to physical turret limits. */
    private static double clampAngle(double deg) {
        return Math.max(TurretConstants.minAngleDeg,
                        Math.min(TurretConstants.maxAngleDeg, deg));
    }

    /** Clamp motor power to configured bounds. */
    private static double clampPower(double p) {
        return Math.max(TurretConstants.minPower, Math.min(TurretConstants.maxPower, p));
    }

    /** Clamp PID output to max output power. */
    private static double clampOutput(double p) {
        return Math.max(-TurretConstants.maxOutputPower,
                        Math.min(TurretConstants.maxOutputPower, p));
    }

    /** Software limits: stop motor from driving past physical boundaries. */
    private double applySoftwareLimits(double power, double angle) {
        if (!isCalibrated) return power;
        if (angle <= TurretConstants.minAngleDeg && power > 0) return 0;  // +power → left
        if (angle >= TurretConstants.maxAngleDeg && power < 0) return 0;  // −power → right
        return power;
    }

    private static double normalise180(double deg) {
        while (deg >  180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private double ticksToDegrees(int ticks) {
        double ticksPerRev = TurretConstants.ENCODER_CPR * TurretConstants.GEAR_RATIO;
        return (ticks / ticksPerRev) * 360.0;
    }

    @SuppressWarnings("unused")
    private double degreesToTicks(double deg) {
        double ticksPerRev = TurretConstants.ENCODER_CPR * TurretConstants.GEAR_RATIO;
        return (deg / 360.0) * ticksPerRev;
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
