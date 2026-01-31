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

    // PID Coefficients (Currently unused for main control loop)
    public static double kP = 0.5;
    public static double kI = 0;
    public static double kD = 0;

    // Theoretical Max TPS for Feedforward Calculation
    // GoBilda 6000RPM Motor (5203-2402-0001): 28 ticks/revolution
    // Max TPS = (6000 / 60) * 28 = 2800 TPS
    public static double maxVelocityTPS = 2800.0;

    /**
     * Target Velocities (in Ticks Per Second)
     * Negative values indicate direction.
     */
    // Idle power (open-loop, no PID control)
    public static double idlePower = 0.27;
    
    public static double stopVelocity = -600;  // Legacy, used for reference only
    public static double fastVelocity = -1420; // Far shots (128.4") (~51% power)
    public static double midVelocity = -950;   // Mid-range shots (77.4") (~34% power)
    public static double slowVelocity = -700;  // Close shots (24.4") (~25% power)
    public static double releaseVelocity = -200; // Threshold to consider "stopped" or "too slow"
    
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
    // Updated based on user request: Close (0.85) -> Far (0.29)
    public static double shooterServoUpPos = 0.29;   // Position for FAST/Long range
    public static double shooterServoMidPos = 0.29;  // Position for MID range (Calculated average)
    public static double shooterServoDownPos = 0.85; // Position for SLOW/Short range or Stowed
    
    // ==================== ADAPTIVE SHOOTING ====================
    // Goal coordinates (in inches)
    public static double blueGoalX = 4;
    public static double blueGoalY = 140;
    public static double redGoalX = 140;
    public static double redGoalY = 140;
    
    // Distance range for velocity interpolation (calibrated from real data)
    // Near: 24.4" -> 700 TPS, Mid: 77.4" -> 950 TPS, Far: 128.4" -> 1420 TPS
    public static double nearDistance = 24.4;   // Distance for slowVelocity (700 TPS)
    public static double midDistance = 77.4;    // Distance for midVelocity (950 TPS)
    public static double farDistance = 128.4;   // Distance for fastVelocity (1420 TPS)
    
    // Distance range for servo angle interpolation (non-linear)
    public static double servoNearDistance = 25;   // Distance for shooterServoDownPos (0.85)
    public static double servoFarDistance = 134;   // Distance for shooterServoUpPos (0.29)
    
    // Auto-fire threshold (degrees)
    public static double autoFireTxThreshold = 1.0;  // Allow fire when |tx| < this
}
