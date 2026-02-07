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
    // For external encoder (8192 CPR)
    public static double shooterEpsilon = 12000;       // SLOW / MID tolerance
    public static double shooterEpsilonFast = 4000;    // FAST (远射) tolerance

    // ==================== PIDF VELOCITY CONTROL (External Encoder) ====================
    // Tuned for REV Through Bore Encoder V2 (8192 CPR) on rightShooterMotor
    // Using 50ms window velocity calculation for stability
    // Auto-tuned values from ShooterAutoTuner
    public static double kP = 0.000015;   // Auto-tuned
    public static double kI = 0.0;        // Keep at 0
    public static double kD = 0.0;        // Auto-tuned
    public static double kF = 0.00000028; // Auto-tuned feedforward
    public static double filterAlpha = 0.15;  // Velocity smoothing (lower = smoother)
    
    // Theoretical Max TPS for External Encoder
    // REV Through Bore V2: 8192 CPR, GoBilda 6000RPM Motor
    // Max TPS = (6000 / 60) * 8192 ≈ 819,200 TPS
    public static double maxVelocityTPS = 819200.0;
    
    // ==================== MOTOR BRAKING (Hybrid Control) ====================
    // When overspeed exceeds threshold, reverse motor to brake (no physical brake)
    // Scaled for external encoder (8192 CPR)
    public static double motorBrakeThreshold = 30000;  // TPS - if overspeed > this, apply motor brake
    public static double motorBrakePower = 0.5;        // Reverse power for braking (0-1, tune as needed)
    public static double pidSwitchThreshold = 90000;   // Switch to PID when within this of target (~300 * 292)

    /**
     * Target Velocities (in Ticks Per Second)
     * Values scaled for REV Through Bore V2 (8192 CPR) external encoder
     * Conversion: old_value * (8192/28) ≈ old_value * 292.57
     */
    // Idle power (open-loop, no PID control)
    public static double idlePower = 0.27;
    
    public static double stopVelocity = 175000;   // ~600 * 292 (idle reference)
    public static double fastVelocity = 385000;   // Far shots (远射)
    public static double midVelocity = 280000;    // Mid-range shots (中射)
    public static double slowVelocity = 224000;   // Close shots (近射)
    public static double releaseVelocity = 60000; // Threshold to consider "stopped"
    
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
    public static double shooterServoMidPos = 0.54;   // Position for MID range and STOP
    public static double shooterServoDownPos = 0.04; // Position for SLOW/Short range
    
    // ==================== ADAPTIVE SHOOTING ====================
    // Goal coordinates (Pedro Pathing: origin at bottom-left, X right, Y up)
    // Blue basket at top-left: (4, 140)
    // Red basket at top-right: (140, 140)
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
