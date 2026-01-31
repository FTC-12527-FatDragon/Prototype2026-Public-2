package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for the Turret (Gimbal) subsystem.
 * Uses REV Through Bore Encoder V2 for angle measurement.
 * All values are tunable via FTC Dashboard.
 */
@Config
public class TurretConstants {
    // Hardware names (must match robot configuration)
    public static String turretMotorName = "turretMotor";
    public static String turretEncoderName = "turretEncoder";  // REV Through Bore Encoder V2
    
    // Motor power limits
    public static double maxPower = 1.0;        // Maximum motor power
    public static double minPower = -1.0;       // Minimum motor power (reverse)
    
    // ==================== REV Through Bore Encoder V2 ====================
    // Incremental mode: 2048 CPR (8192 counts per revolution)
    // Accuracy: ±0.5°
    public static int ENCODER_CPR = 8192;       // Counts per revolution
    
    // Gear ratio: motor rotations per turret rotation
    // e.g., if motor spins 10 times for turret to spin once, GEAR_RATIO = 10.0
    // TODO: measure and fill in actual gear ratio
    public static double GEAR_RATIO = 1.0;      // Motor rotations : Turret rotations
    
    // Angle offset: encoder 0° corresponds to this turret angle
    // If encoder reads 0° when turret faces right (90°), set this to 90.0
    public static double ANGLE_OFFSET = 90.0;   // degrees
    
    // Angle limits (degrees, 0 = forward)
    // Positive = clockwise (right), Negative = counterclockwise (left)
    public static double minAngleDeg = -90.0;   // Minimum angle (left limit)
    public static double maxAngleDeg = 90.0;    // Maximum angle (right limit)
    
    // ==================== GEOMETRY (Limelight on turret) ====================
    // Turret center is behind chassis center
    public static double turretOffsetMM = 47.0;           // mm, turret center behind chassis center
    // Limelight distance from turret center
    public static double limelightOffsetMM = 140.86521;   // mm, limelight from turret center
    
    // ==================== POSITION PID (Soft Lock) ====================
    // PID constants for position control (angle in degrees)
    public static double kP = 0.0;      // TODO: tune
    public static double kI = 0.0;      // TODO: tune
    public static double kD = 0.0;      // TODO: tune
    
    // Position control parameters
    public static double positionTolerance = 0.0;    // Degrees, within this = at setpoint (TODO: tune)
    public static double maxOutputPower = 0.0;       // Maximum output from PID (TODO: tune)
    public static double minOutputPower = 0.0;       // Minimum output to overcome static friction (TODO: tune)
    
    // Feedforward coefficient for PIDF controller
    // In FTCLib, F term is multiplied by setpoint: output += kF * setpoint
    // For turret position control, this can help with:
    // - Gravity compensation (if turret is tilted)
    // - Predictive positioning
    public static double kF = 0.0;      // TODO: tune if needed
    
    // ==================== HARD LOCK (Absolute Position Tracking) ====================
    // Uses robot absolute position to calculate turret angle to goal
    
    // Goal positions (field coordinates, inches)
    public static double blueGoalX = 4.0;     // Blue basket X
    public static double blueGoalY = 140.0;   // Blue basket Y
    public static double redGoalX = 140.0;    // Red basket X
    public static double redGoalY = 140.0;    // Red basket Y
    
    // Calibration speed (for finding zero position)
    public static double calibrationSpeed = 0.2;
}


