package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for the Shooter subsystem.
 */
@Config
public class ShooterConstants {
    // Hardware map names
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";
    public static String shooterServoName = "shooterServo";

    // Velocity tolerance (TPS) - Used to check if shooter is at target speed
    // Increased from 20 to 100 for more reliable firing
    public static double shooterEpsilon = 100;

    // ==================== PIDF VELOCITY CONTROL (for true closed-loop) ====================
    // Currently unused - using Pseudo Closed-loop with feedforward instead
    // Uncomment in Shooter.java to enable PIDF control
    public static double kP = 0.0005;   // Start small for velocity control
    public static double kI = 0.0;      // Usually keep at 0
    public static double kD = 0.0;      // Add if oscillating
    public static double kF = 0.0004;   // Feedforward: kF * targetVelocity added to output
    // Note: For velocity control, kF ≈ 1/maxVelocityTPS ≈ 1/2800 ≈ 0.00036

    // Theoretical Max TPS for Feedforward Calculation
    // GoBilda 6000RPM Motor (5203-2402-0001): 28 ticks/revolution
    // Max TPS = (6000 / 60) * 28 = 2800 TPS
    public static double maxVelocityTPS = 2800.0;
    
    // ==================== MOTOR BRAKING (Pseudo Closed-loop) ====================
    // When overspeed exceeds threshold, reverse motor to brake (no physical brake)
    public static double motorBrakeThreshold = 100;  // TPS - if overspeed > this, apply motor brake
    public static double motorBrakePower = 0.5;      // Reverse power for braking (0-1, tune as needed)

    /**
     * Target Velocities (in Ticks Per Second)
     * Positive values - matches original Prototype2026-Public design.
     */
    // Idle power (open-loop, no PID control)
    public static double idlePower = 0.27;
    
    public static double stopVelocity = 600;   // Legacy, used for reference only
    public static double fastVelocity = 2100;  // Far shots (~69% power)
    public static double midVelocity = 1500;   // Mid-range shots (~46% power)
    public static double slowVelocity = 950;   // Close shots (~34% power)
    public static double releaseVelocity = 200; // Threshold to consider "stopped" or "too slow"
    
    // Velocity tolerances for transit engagement (ticks per second)
    // Defines the acceptable range around the target velocity.
    // Upper: Max allowed speed ABOVE target (less negative magnitude)
    // Lower: Max allowed speed BELOW target (more negative magnitude)
    
    // MID Tolerances
    public static double toleranceMidUpper = 50; // Allow being slightly slower
    public static double toleranceMidLower = 50; // Allow being slightly faster
    
    // FAST Tolerances
    public static double toleranceFastUpper = 20;
    public static double toleranceFastLower = 20; // Usually okay to be faster
    
    // SLOW Tolerances
    public static double toleranceSlowUpper = 50;
    public static double toleranceSlowLower = 50;

    // Servo Positions for Angle Adjustment
    // STOP/MID (0.5), SLOW (0.04), FAST (1.0)
    public static double shooterServoUpPos = 1.0;    // Position for FAST/Long range
    public static double shooterServoMidPos = 0.5;   // Position for MID range and STOP
    public static double shooterServoDownPos = 0.04; // Position for SLOW/Short range
    
    // ==================== ADAPTIVE SHOOTING ====================
    // Goal coordinates (in inches)
    public static double blueGoalX = 4;
    public static double blueGoalY = 140;
    public static double redGoalX = 140;
    public static double redGoalY = 140;
    
    // Distance range for velocity interpolation (needs recalibration)
    public static double nearDistance = 24.4;   // Distance for slowVelocity
    public static double midDistance = 77.4;    // Distance for midVelocity
    public static double farDistance = 128.4;   // Distance for fastVelocity
    
    // Distance range for servo angle interpolation (non-linear)
    public static double servoNearDistance = 25;   // Distance for shooterServoDownPos (0.85)
    public static double servoFarDistance = 134;   // Distance for shooterServoUpPos (0.29)
    
    // Auto-fire threshold (degrees)
    public static double autoFireTxThreshold = 1.0;  // Allow fire when |tx| < this
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
