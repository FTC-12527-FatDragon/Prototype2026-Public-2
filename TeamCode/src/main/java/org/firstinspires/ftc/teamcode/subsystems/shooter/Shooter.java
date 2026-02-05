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
    
    // ==================== FIRING BOOST ====================
    // When transit is open (firing), apply extra power to compensate for ball drag
    private boolean isTransitFiring = false;
    private long firingStartTime = 0;
    
    // Linear boost: power increases from START to END over BOOST_DURATION_MS
    private static final long BOOST_DURATION_MS = 1000;  // 1 second ramp
    
    // Boost amounts by state (start%, end%)
    private static final double BOOST_START_SLOW = 0.30;
    private static final double BOOST_END_SLOW = 0.45;
    private static final double BOOST_START_MID = 0.27;
    private static final double BOOST_END_MID = 0.38;
    private static final double BOOST_START_FAST = 0.25;
    private static final double BOOST_END_FAST = 0.40;
    
    // ==================== MANUAL VELOCITY CALCULATION ====================
    // REV Through Bore Encoder V2 (8192 CPR) - 50ms window for stable readings
    private static final long VELOCITY_WINDOW_MS = 50;  // Calculate every 50ms
    private int windowStartPos = 0;
    private long windowStartTime = 0;
    private double calculatedVelocity = 0;  // Manually calculated velocity (TPS)

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
        
        // Initialize manual velocity calculation window
        windowStartPos = rightShooter.getCurrentPosition();
        windowStartTime = System.currentTimeMillis();
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
     * Sets the transit firing state for boost activation.
     * Call with true when transit opens (start firing), false when transit closes.
     * @param firing True to start boost, false to end boost.
     */
    public void setTransitFiring(boolean firing) {
        if (firing && !isTransitFiring) {
            // Starting to fire - reset boost timer
            firingStartTime = System.currentTimeMillis();
        }
        isTransitFiring = firing;
    }
    
    /**
     * Checks if transit is currently firing (boost active).
     * @return True if firing.
     */
    public boolean isTransitFiring() {
        return isTransitFiring;
    }
    
    /**
     * Calculates the current boost amount based on linear ramping.
     * @return Boost power to add (0.0 to ~0.45)
     */
    private double calculateBoostAmount() {
        if (!isTransitFiring) return 0;
        
        long elapsed = System.currentTimeMillis() - firingStartTime;
        double progress = Math.min(1.0, (double) elapsed / BOOST_DURATION_MS);
        
        double startBoost, endBoost;
        switch (shooterState) {
            case SLOW:
                startBoost = BOOST_START_SLOW;
                endBoost = BOOST_END_SLOW;
                break;
            case MID:
                startBoost = BOOST_START_MID;
                endBoost = BOOST_END_MID;
                break;
            case FAST:
                startBoost = BOOST_START_FAST;
                endBoost = BOOST_END_FAST;
                break;
            default:
                return 0;
        }
        
        // Linear interpolation: start + (end - start) * progress
        return startBoost + (endBoost - startBoost) * progress;
    }
    
    /**
     * Gets the current boost status as a string for telemetry.
     * @return Boost status string (e.g., "OFF", "SLOW 32%", etc.)
     */
    public String getBoostStatus() {
        if (!isTransitFiring) return "OFF";
        
        double boost = calculateBoostAmount();
        long elapsed = System.currentTimeMillis() - firingStartTime;
        
        return String.format("%s %.0f%% (%dms)", 
                shooterState.toString(), 
                boost * 100, 
                elapsed);
    }

    /**
     * Gets the current velocity of the shooter.
     * Uses manual 50ms window calculation for stable REV V2 encoder readings.
     * @return Velocity in ticks per second (always positive).
     */
    public double getVelocity() {
        // Return manually calculated velocity (always positive, stable)
        return calculatedVelocity;
    }

    /**
     * Gets the target velocity based on the current state.
     * @return Target velocity in ticks per second.
     */
    public double getTargetVelocity() {
        return shooterState.shooterVelocity;
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
            return false;
        }
        
        // Check if current velocity is close to target velocity
        // Use manually calculated velocity (stable, always positive)
        return Util.epsilonEqual(
                calculatedVelocity,
                targetVel,
                ShooterConstants.shooterEpsilon
        );
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
        
        // ==================== MANUAL VELOCITY CALCULATION ====================
        // REV Through Bore V2 encoder - 50ms window for stable readings
        long currentTime = System.currentTimeMillis();
        int currentPos = rightShooter.getCurrentPosition();
        long elapsed = currentTime - windowStartTime;
        
        if (elapsed >= VELOCITY_WINDOW_MS) {
            // Calculate velocity: deltaPos / deltaTime * 1000 = TPS
            int deltaPos = Math.abs(currentPos - windowStartPos);  // Always positive
            double rawVelocity = deltaPos * 1000.0 / elapsed;
            
            // Low-pass filter for smoothing
            calculatedVelocity = ShooterConstants.filterAlpha * rawVelocity 
                    + (1 - ShooterConstants.filterAlpha) * calculatedVelocity;
            
            // Reset window
            windowStartPos = currentPos;
            windowStartTime = currentTime;
        }
        
        // Control loop uses manually calculated velocity (stable, always positive)
        double currentVel = calculatedVelocity;
        
        // Use adaptive velocity if set, otherwise use state velocity
        double targetVel = (adaptiveVelocity != 0) ? adaptiveVelocity : shooterState.shooterVelocity;
        double power;

        // =================================================================
        // OPTION 1: PSEUDO CLOSED-LOOP (Current Implementation)
        // Pros: Fast acceleration, simple, with motor braking
        // Cons: Not smooth, no fine control
        // =================================================================
        if (shooterState == ShooterState.STOP && adaptiveVelocity == 0) {
            // Idle mode: Use fixed open-loop power, no closed-loop control
            power = ShooterConstants.idlePower;
        } else {
            // Pseudo Closed-loop with Feedforward (NO reverse braking)
            // Note: Velocities are positive (e.g., Target: 1500, Current: 1200)
            // 
            // Two states:
            // 1. Too slow (currentVel < targetVel): Full power to accelerate
            // 2. At or above target: Feedforward power to maintain (let it coast down naturally)
            
            if (currentVel < targetVel) {
                // Too slow, apply max power to accelerate
                power = 1.0;
            } else {
                // At or above target speed, use feedforward to maintain
                // Ratio = target / maxVelocityTPS
                power = targetVel / ShooterConstants.maxVelocityTPS;
            }
            
            // Apply firing boost if transit is open
            double boost = calculateBoostAmount();
            if (boost > 0) {
                power = Math.min(1.0, power + boost);
            }
        }
        
        // =================================================================
        // OPTION 2: TRUE PIDF VELOCITY CONTROL (Uncomment to use)
        // Pros: Smooth, precise velocity control
        // Cons: Requires tuning kP, kI, kD, kF
        // 
        // To switch: Comment out OPTION 1 above, uncomment below
        // =================================================================
        /*
        // Update PIDF coefficients from constants (allows Dashboard tuning)
        velocityPIDF.setPIDF(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                ShooterConstants.kF
        );
        
        if (shooterState == ShooterState.STOP && adaptiveVelocity == 0) {
            // Idle mode: Use fixed open-loop power, no closed-loop control
            power = ShooterConstants.idlePower;
            velocityPIDF.reset();  // Reset integrator when idle
        } else {
            // PIDF Velocity Control
            // Note: currentVel and targetVel are both positive
            // PIDF calculates: error = setpoint - measurement = targetVel - currentVel
            // Output = kP*error + kI*integral + kD*derivative + kF*setpoint
            
            // Calculate PIDF output
            double pidfOutput = velocityPIDF.calculate(currentVel, targetVel);
            
            // Clamp output to [0, 1] (shooter only runs one direction)
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
