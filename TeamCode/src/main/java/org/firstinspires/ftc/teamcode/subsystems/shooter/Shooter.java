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
    
    // FAST mode stability check: must be at setpoint for 0.3s before firing
    private long fastModeStableStartTime = 0;
    private boolean fastModeWasAtSetpoint = false;
    private static final long FAST_MODE_STABLE_TIME_MS = 300;  // 0.3 seconds
    
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
     * Checks if the shooter has reached its target velocity.
     * Considers adaptiveVelocity if set.
     * @return True if current velocity is within epsilon of target.
     */
    public boolean isShooterAtSetPoint() {
        // Determine target velocity: use adaptive if set, otherwise use state velocity
        double targetVel = (adaptiveVelocity != 0) ? adaptiveVelocity : shooterState.shooterVelocity;
        
        // If using state velocity and state is STOP, return false
        if (adaptiveVelocity == 0 && shooterState == ShooterState.STOP) {
            fastModeWasAtSetpoint = false;  // Reset stability tracking
            return false;
        }
        
        // Check if current velocity is close to target velocity
        // Uses calculated velocity from 50ms window (always positive)
        boolean atSetpoint = Util.epsilonEqual(
                calculatedVelocity,
                targetVel,
                ShooterConstants.shooterEpsilon
        );
        
        // FAST mode: require 0.3s of continuous stability before allowing fire
        if (shooterState == ShooterState.FAST) {
            if (atSetpoint) {
                if (!fastModeWasAtSetpoint) {
                    // Just entered setpoint range, start timing
                    fastModeStableStartTime = System.currentTimeMillis();
                    fastModeWasAtSetpoint = true;
                }
                // Check if stable for required duration
                long stableTime = System.currentTimeMillis() - fastModeStableStartTime;
                return stableTime >= FAST_MODE_STABLE_TIME_MS;
            } else {
                // Lost setpoint, reset timing
                fastModeWasAtSetpoint = false;
                return false;
            }
        }
        
        // Other modes: immediate fire when at setpoint
        return atSetpoint;
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

        // =================================================================
        // OPTION 1: PSEUDO CLOSED-LOOP (Disabled)
        // =================================================================
        /*
        if (shooterState == ShooterState.STOP && adaptiveVelocity == 0) {
            power = ShooterConstants.idlePower;
        } else {
            double overspeedThreshold = ShooterConstants.motorBrakeThreshold;
            if (currentVel < targetVel) {
                if (shooterState == ShooterState.FAST && currentVel >= targetVel - 500) {
                    power = 0.8;
                } else {
                    power = 1;
                }
            } else if (currentVel > targetVel + overspeedThreshold) {
                power = -ShooterConstants.motorBrakePower;
            } else {
                power = targetVel / ShooterConstants.maxVelocityTPS;
            }
        }
        */
        
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
        // OPTION 3: HYBRID (Pseudo Closed-loop + PIDF) - ACTIVE
        // Far from target: Full power acceleration (fast response)
        // Near target: PIDF fine control (precision)
        // Overspeed: Motor brake
        // =================================================================
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
            double error = targetVel - currentVel;
            double overspeedThreshold = ShooterConstants.motorBrakeThreshold;
            double pidSwitchThreshold = ShooterConstants.pidSwitchThreshold;
            
            if (error > pidSwitchThreshold) {
                // Far from target: Full power acceleration (pseudo closed-loop)
                power = 1.0;
            } else if (error > 0) {
                // Near target but still below: PIDF fine control
                double pidfOutput = velocityPIDF.calculate(currentVel, targetVel);
                power = Math.max(0, Math.min(1, pidfOutput));
            } else if (error < -overspeedThreshold) {
                // Overspeed by more than threshold: brake
                power = -ShooterConstants.motorBrakePower;
                velocityPIDF.reset();  // Reset integrator when braking
            } else {
                // At target or slightly over: PIDF maintain
                double pidfOutput = velocityPIDF.calculate(currentVel, targetVel);
                power = Math.max(-0.3, Math.min(1, pidfOutput));  // Allow slight negative for correction
            }
        }

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
