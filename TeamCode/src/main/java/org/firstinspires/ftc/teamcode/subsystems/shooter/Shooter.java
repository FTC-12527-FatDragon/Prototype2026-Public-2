package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.releaseVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Util;

/**
 * Subsystem handling the Shooter mechanism.
 * Controls the shooter flywheels and the angle adjustment servo.
 */
public class Shooter extends SubsystemBase {
    public final DcMotorEx rightShooter;
    public final DcMotorEx leftShooter;
    public final Servo shooterServo;
    public final TelemetryPacket packet = new TelemetryPacket();
    
    // PID Controller (Currently unused, replaced by Pseudo Closed-loop)
    public final PIDController pidController;
    
    // ==================== TRUE PIDF VELOCITY CONTROL ====================
    // Uncomment the PIDF section in periodic() to use this instead of Pseudo Closed-loop
    public final PIDFController velocityPIDF;
    
    // Current state of the shooter
    public ShooterState shooterState = ShooterState.STOP;
    
    // Emergency disable flag (controlled by gamepad2)
    private boolean disabled = false;
    
    // Stability check: must be at setpoint for 0.3s before firing
    private long stableStartTime = 0;
    private boolean wasAtSetpoint = false;
    private static final long STABLE_TIME_MS = 300;  // 0.3 seconds
    
    // Firing boost: increase power 0.2s after transit opens
    private long firingStartTime = 0;
    private boolean isFiring = false;
    private static final long FIRING_BOOST_DELAY_MS = 200;  // 0.2 seconds
    private static final double FIRING_BOOST_AMOUNT_FAST = 0.08;  // 8% boost for FAST (far shot)
    private static final double FIRING_BOOST_AMOUNT_MID = 0.16;   // 16% boost for MID
    private static final double FIRING_BOOST_AMOUNT_SLOW = 0.18;  // 18% boost for SLOW (close shot)
    private static final long FIRING_BOOST_MAX_DURATION_MS = 1000;  // Max 1 second boost
    private double lockedPower = 0;  // Power locked when boost activates
    private boolean boostActive = false;  // True when using locked power + boost
    private ShooterState lastShooterState = ShooterState.STOP;  // Track state changes
    private boolean transitFiring = false;  // True when LT + bumper pressed (external control)
    
    // 50ms window velocity calculation (for external encoder stability)
    private int windowStartPos = 0;
    private long windowStartTime = 0;
    private double calculatedVelocity = 0;
    private static final long VELOCITY_WINDOW_MS = 50;  // Calculate velocity every 50ms

    /**
     * Constructor for Shooter.
     * Initializes motors, servo, and PID controller.
     *
     * @param hardwareMap The hardware map.
     */
    public Shooter(final HardwareMap hardwareMap) {
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        shooterServo = hardwareMap.get(Servo.class, ShooterConstants.shooterServoName);
        
        // No setMode needed - default mode works for both setPower() and getVelocity()
        // (Same as original Prototype2026-Public)
        
        pidController = new PIDController(ShooterConstants.kP,
                ShooterConstants.kI, ShooterConstants.kD);
        
        // Initialize PIDF controller for true velocity closed-loop
        velocityPIDF = new PIDFController(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                ShooterConstants.kF
        );
    }

    /**
     * Enum representing the various states of the shooter.
     * Each state defines a target velocity and a servo position.
     */
    public enum ShooterState {
        STOP(ShooterConstants.stopVelocity, ShooterConstants.shooterServoMidPos),   // 0.5
        SLOW(ShooterConstants.slowVelocity, ShooterConstants.shooterServoDownPos),  // 0.04
        MID(ShooterConstants.midVelocity, ShooterConstants.shooterServoMidPos),     // 0.5
        FAST(ShooterConstants.fastVelocity, ShooterConstants.shooterServoUpPos);    // 1.0

        final double shooterVelocity, shooterServoPos;

        ShooterState(double shooterVelocity, double shooterServoPos) {
            this.shooterVelocity = shooterVelocity;
            this.shooterServoPos = shooterServoPos;
        }
    }

    /**
     * Sets the target state for the shooter.
     * @param shooterState The desired ShooterState.
     */
    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
        this.adaptiveVelocity = 0;  // Clear adaptive velocity when using state
        this.adaptiveServoPosition = -1;  // Clear adaptive servo position when using state
    }
    
    // Adaptive velocity for auto-fire (0 means use state velocity)
    private double adaptiveVelocity = 0;
    
    /**
     * Sets an adaptive velocity for auto-fire.
     * This takes priority over the state velocity.
     * @param velocity Target velocity in TPS (negative value)
     */
    public void setAdaptiveVelocity(double velocity) {
        this.adaptiveVelocity = velocity;
    }
    
    /**
     * Gets the current adaptive velocity setting.
     */
    public double getAdaptiveVelocity() {
        return adaptiveVelocity;
    }
    
    // Adaptive servo position for auto-fire (-1 means use state servo position)
    private double adaptiveServoPosition = -1;
    
    /**
     * Sets an adaptive servo position for auto-fire.
     * This takes priority over the state servo position.
     * @param position Servo position (0-1), or -1 to use state position
     */
    public void setAdaptiveServoPosition(double position) {
        this.adaptiveServoPosition = position;
    }
    
    /**
     * Gets the current adaptive servo position setting.
     */
    public double getAdaptiveServoPosition() {
        return adaptiveServoPosition;
    }
    
    /**
     * Sets the emergency disable flag.
     * When disabled, the shooter motors will be set to 0 power.
     * Controlled by gamepad2 (RT + RB to toggle).
     *
     * @param disabled True to disable shooter.
     */
    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }
    
    /**
     * Checks if the shooter is disabled.
     * @return True if disabled.
     */
    public boolean isDisabled() {
        return disabled;
    }
    
    /**
     * Toggles the disabled state.
     */
    public void toggleDisabled() {
        disabled = !disabled;
    }

    /**
     * Gets the current velocity of the shooter (calculated via 50ms window).
     * @return Velocity in ticks per second (always positive).
     */
    public double getVelocity() {
        return calculatedVelocity;
    }
    
    /**
     * Gets boost status for debugging.
     * @return String describing current boost state
     */
    public String getBoostStatus() {
        if (boostActive) {
            return "ACTIVE (+" + (int)(((shooterState == ShooterState.FAST) ? FIRING_BOOST_AMOUNT_FAST 
                    : (shooterState == ShooterState.SLOW) ? FIRING_BOOST_AMOUNT_SLOW : FIRING_BOOST_AMOUNT_MID) * 100) + "%)";
        } else if (isFiring && transitFiring) {
            long elapsed = System.currentTimeMillis() - firingStartTime;
            if (elapsed < FIRING_BOOST_DELAY_MS) {
                return "WAITING (" + elapsed + "/" + FIRING_BOOST_DELAY_MS + "ms)";
            }
        } else if (isFiring) {
            return "isFiring (no LT+bumper)";
        } else if (transitFiring) {
            return "transitFiring (no isFiring)";
        }
        return "OFF";
    }
    
    /**
     * Updates velocity using 50ms window calculation.
     * Called every loop iteration, but only recalculates when window expires.
     * Always returns positive velocity (shooter spins one direction).
     */
    private void updateVelocity() {
        int currentPos = rightShooter.getCurrentPosition();
        long currentTime = System.currentTimeMillis();
        long elapsed = currentTime - windowStartTime;
        
        if (elapsed >= VELOCITY_WINDOW_MS) {
            // Calculate velocity: |deltaPos| / deltaTime
            int deltaPos = Math.abs(currentPos - windowStartPos);  // Always positive
            double rawVelocity = deltaPos * 1000.0 / elapsed;      // TPS
            
            // Light smoothing
            calculatedVelocity = ShooterConstants.filterAlpha * rawVelocity 
                    + (1 - ShooterConstants.filterAlpha) * calculatedVelocity;
            
            // Reset window
            windowStartPos = currentPos;
            windowStartTime = currentTime;
        }
    }

    /**
     * Gets the target velocity based on the current state.
     * @return Target velocity in ticks per second.
     */
    public double getTargetVelocity() {
        return shooterState.shooterVelocity;
    }

    /**
     * Sets whether transit is actively firing (LT + bumper pressed).
     * This controls when firing boost can activate.
     * @param firing True when actively firing (LT + bumper held)
     */
    public void setTransitFiring(boolean firing) {
        this.transitFiring = firing;
        // If stopped firing, reset boost
        if (!firing) {
            boostActive = false;
            lockedPower = 0;
        }
    }
    
    /**
     * Checks if the shooter has reached its target velocity.
     * Considers adaptiveVelocity if set.
     * @return True if current velocity is within epsilon of target.
     */
    public boolean isShooterAtSetPoint() {
        // Determine target velocity: use adaptive if set, otherwise use state velocity
        double targetVel = (adaptiveVelocity != 0) ? adaptiveVelocity : shooterState.shooterVelocity;
        
        // If using state velocity and state is STOP, return false
        if (adaptiveVelocity == 0 && shooterState == ShooterState.STOP) {
            wasAtSetpoint = false;  // Reset stability tracking
            stableStartTime = 0;
            isFiring = false;  // Stop firing boost
            firingStartTime = 0;
            return false;
        }
        
        // Check if current velocity is close to target velocity
        // FAST mode uses tighter tolerance for better accuracy
        double epsilon = (shooterState == ShooterState.FAST) ? 15000 : ShooterConstants.shooterEpsilon;
        boolean atSetpoint = Util.epsilonEqual(
                calculatedVelocity,
                targetVel,
                epsilon
        );
        
        // No stability delay - fire immediately when at setpoint
        if (atSetpoint) {
            // Start firing boost timer when first reaching setpoint
            if (!isFiring) {
                isFiring = true;
                firingStartTime = System.currentTimeMillis();
            }
            return true;
        } else {
            // Not at setpoint
            // Keep firing boost active if already firing (allow continuous fire during boost)
            return isFiring;
        }
    }


    /**
     * Periodic update method.
     * Implements Pseudo Closed-loop control with Feedforward for velocity regulation.
     * STOP state uses open-loop idle power (no PID).
     * 
     * Alternative: TRUE PIDF VELOCITY CONTROL (commented out below)
     */
    @Override
    public void periodic() {
        // Emergency disable check - highest priority
        if (disabled) {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
            return;
        }
        
        // Initialize velocity window on first call
        if (windowStartTime == 0) {
            windowStartPos = rightShooter.getCurrentPosition();
            windowStartTime = System.currentTimeMillis();
        }
        
        // Update velocity using 50ms window (external encoder)
        updateVelocity();
        
        // Control loop runs always (even in STOP state) to maintain idle speed if set
        double currentVel = calculatedVelocity;  // Always positive
        
        // Use adaptive velocity if set, otherwise use state velocity
        double targetVel = (adaptiveVelocity != 0) ? adaptiveVelocity : shooterState.shooterVelocity;
        double power;
        
        // Check for state change (档位切换) - reset boost
        if (shooterState != lastShooterState) {
            boostActive = false;
            lockedPower = 0;
            lastShooterState = shooterState;
        }
        
        // Check boost timeout (max 1 second)
        if (boostActive && (System.currentTimeMillis() - firingStartTime) > (FIRING_BOOST_DELAY_MS + FIRING_BOOST_MAX_DURATION_MS)) {
            boostActive = false;
            lockedPower = 0;
        }
        
        // If not actively firing (LT + bumper), reset boost
        if (!transitFiring) {
            boostActive = false;
            lockedPower = 0;
        }

        // =================================================================
        // OPTION 1: PSEUDO CLOSED-LOOP - ACTIVE (Optimized)
        // =================================================================
        if (shooterState == ShooterState.STOP && adaptiveVelocity == 0) {
            power = ShooterConstants.idlePower;
            boostActive = false;  // Reset boost when STOP
            lockedPower = 0;
        } else if (boostActive) {
            // BOOST MODE: Use locked power + boost, bypass pseudo closed-loop
            double boostAmount = (shooterState == ShooterState.FAST) ? FIRING_BOOST_AMOUNT_FAST 
                    : (shooterState == ShooterState.SLOW) ? FIRING_BOOST_AMOUNT_SLOW : FIRING_BOOST_AMOUNT_MID;
            power = Math.min(1.0, lockedPower + boostAmount);
        } else {
            // NORMAL MODE: Pseudo closed-loop control
            double overspeedThreshold = ShooterConstants.motorBrakeThreshold;
            double deadband = 15000;  // Stability zone: ±15000 TPS around target
            double error = targetVel - currentVel;
            
            // Calculate feedforward with correction factor (motors don't reach theoretical max)
            double feedforward = (targetVel / ShooterConstants.maxVelocityTPS) * 1.3;
            feedforward = Math.min(feedforward, 0.95);  // Cap at 95%
            
            // Mode-specific parameters
            boolean isMidMode = (shooterState == ShooterState.MID);
            double approachPower = isMidMode ? 0.7 : 0.85;  // MID uses gentler approach
            // Reduced power for overspeed (no reverse, just lower power to let motor slow naturally)
            double reducedPower = feedforward * 0.7;  // 70% of feedforward when overspeed
            
            if (error > deadband) {
                // Below target by more than deadband: accelerate
                if (error > 50000) {
                    power = 1.0;  // Far from target: full power
                } else {
                    power = approachPower;  // Close to target: reduced power for smoother approach
                }
            } else if (error < -overspeedThreshold) {
                // Overspeed beyond threshold: reduce power (no reverse braking)
                power = reducedPower;
            } else {
                // Within deadband or slightly over: use feedforward to maintain
                power = feedforward;
            }
            
            // Check if boost should activate (0.2s after firing starts, only when LT + bumper held)
            if (isFiring && transitFiring && (System.currentTimeMillis() - firingStartTime) > FIRING_BOOST_DELAY_MS) {
                // Lock current power and activate boost mode
                lockedPower = power;
                boostActive = true;
                double boostAmount = (shooterState == ShooterState.FAST) ? FIRING_BOOST_AMOUNT_FAST 
                    : (shooterState == ShooterState.SLOW) ? FIRING_BOOST_AMOUNT_SLOW : FIRING_BOOST_AMOUNT_MID;
                power = Math.min(1.0, lockedPower + boostAmount);
            }
        }
        
        // =================================================================
        // OPTION 2: TRUE PIDF VELOCITY CONTROL (Disabled)
        // =================================================================
        /*
        velocityPIDF.setPIDF(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
        if (shooterState == ShooterState.STOP && adaptiveVelocity == 0) {
            power = ShooterConstants.idlePower;
            velocityPIDF.reset();
        } else {
            double pidfOutput = velocityPIDF.calculate(currentVel, targetVel);
            power = Math.max(0, Math.min(1, pidfOutput));
        }
        */
        
        // =================================================================
        // OPTION 3: HYBRID (Pseudo Closed-loop + PIDF) - Disabled
        // Far from target: Full power acceleration (fast response)
        // Near target: PIDF fine control (precision)
        // Overspeed: Motor brake
        // =================================================================
        /*
        // Update PIDF coefficients (allows Dashboard tuning)
        velocityPIDF.setPIDF(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                ShooterConstants.kF
        );
        
        if (shooterState == ShooterState.STOP && adaptiveVelocity == 0) {
            // Idle mode: fixed open-loop power
            power = ShooterConstants.idlePower;
            velocityPIDF.reset();
        } else {
            // Pure PIDF velocity control (tuned parameters from ShooterPIDTuner)
            double pidfOutput = velocityPIDF.calculate(currentVel, targetVel);
            power = Math.max(0, Math.min(1, pidfOutput));
        }
        */

        // Apply power (reversed from original - this robot's motors are wired differently)
        leftShooter.setPower(power);
        rightShooter.setPower(-power);

        // Update Servo Position
        // Use adaptive servo position if set, otherwise use state servo position
        double servoPos = (adaptiveServoPosition >= 0) ? adaptiveServoPosition : shooterState.shooterServoPos;
        shooterServo.setPosition(servoPos);

        // Telemetry handled centrally
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
