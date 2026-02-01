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
     * Gets the current velocity of the right shooter motor.
     * @return Velocity in ticks per second.
     */
    public double getVelocity() {
        // leftShooter runs positive, but target velocities are negative
        // Return negative to match the convention
        return -leftShooter.getVelocity();
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
     * @return True if at or above (more negative) target speed.
     */
    public boolean isShooterAtSetPoint() {
        // Determine target velocity: use adaptive if set, otherwise use state velocity
        double targetVel = (adaptiveVelocity != 0) ? adaptiveVelocity : shooterState.shooterVelocity;
        
        // If using state velocity and state is STOP, return false
        if (adaptiveVelocity == 0 && shooterState == ShooterState.STOP) {
            return false;
        }
        
        // Check if current velocity is close to target velocity
        // leftShooter runs positive, but targetVel is negative, so negate for comparison
        return Util.epsilonEqual(
                -leftShooter.getVelocity(),
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
        
        // Control loop runs always (even in STOP state) to maintain idle speed if set
        // leftShooter runs positive, but target velocities are negative, so negate
        double currentVel = -leftShooter.getVelocity();
        
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
            // Pseudo Closed-loop with Feedforward + Motor Braking
            // Note: Velocities are negative (e.g., Target: -1500, Current: -1800)
            // 
            // Three states:
            // 1. Too slow (currentVel > targetVel): Full power to accelerate
            // 2. Too fast by > 200 TPS (currentVel < targetVel - 200): Reverse motor to brake
            // 3. Near target: Feedforward power to maintain
            
            double overspeedThreshold = ShooterConstants.motorBrakeThreshold;  // 200 TPS
            
            if (currentVel > targetVel) {
                // Too slow, apply max power to accelerate
                power = 1.0;
            } else if (currentVel < targetVel - overspeedThreshold) {
                // Too fast by more than threshold, apply reverse power to brake
                // Since we don't have physical brake, use motor reverse as brake
                power = -ShooterConstants.motorBrakePower;
            } else {
                // Near target speed, use feedforward to maintain
                // Ratio = |target| / maxVelocityTPS
                power = Math.abs(targetVel) / ShooterConstants.maxVelocityTPS;
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
            // Note: currentVel and targetVel are both negative
            // PIDF calculates: error = setpoint - measurement = targetVel - currentVel
            // Output = kP*error + kI*integral + kD*derivative + kF*setpoint
            
            // Calculate PIDF output
            double pidfOutput = velocityPIDF.calculate(currentVel, targetVel);
            
            // Clamp output to [0, 1] (shooter only runs one direction)
            power = Math.max(0, Math.min(1, pidfOutput));
        }
        */

        // Apply power
        // leftShooter runs positive, rightShooter runs negative
        leftShooter.setPower(power);
        rightShooter.setPower(-power);

        // Update Servo Position
        // Use adaptive servo position if set, otherwise use state servo position
        double servoPos = (adaptiveServoPosition >= 0) ? adaptiveServoPosition : shooterState.shooterServoPos;
        shooterServo.setPosition(servoPos);

        // Telemetry handled centrally
    }
}
