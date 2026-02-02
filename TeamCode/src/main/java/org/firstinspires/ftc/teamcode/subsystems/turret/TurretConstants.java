package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for the Turret (Gimbal) subsystem.
 * Uses motor built-in encoder for angle measurement.
 * All values are tunable via FTC Dashboard.
 */
@Config
public class TurretConstants {
    // Hardware names (must match robot configuration)
    public static String turretMotorName = "turretMotor";
    // Note: Encoder is built into the motor, no separate encoder port needed
    
    // Motor power limits
    public static double maxPower = 1.0;        // Maximum motor power
    public static double minPower = -1.0;       // Minimum motor power (reverse)
    
    // ==================== REV THROUGH BORE ENCODER V2 ====================
    // External encoder mounted on motor shaft, connected to motor encoder port
    // REV Through Bore Encoder V2 (REV-11-3174) specs:
    // - Incremental mode: 8192 counts per revolution
    // - Accuracy: ±0.5°
    // Read via turretMotor.getCurrentPosition() (encoder wired to motor port)
    public static double ENCODER_CPR = 8192;   // REV Through Bore Encoder V2 CPR
    
    // External gear ratio: motor turns 116 times -> turret turns 22 times
    // GEAR_RATIO = motor rotations / turret rotations = 116 / 22 ≈ 5.2727
    public static double GEAR_RATIO = 116.0 / 22.0;  // Motor rotations : Turret rotations
    
    // Angle offset: encoder 0° corresponds to this turret angle
    // If encoder reads 0° when turret faces forward (0°), set this to 0.0
    public static double ANGLE_OFFSET = 0.0;   // degrees
    
    // Angle limits (degrees, 0 = forward)
    // Positive = clockwise (right), Negative = counterclockwise (left)
    // Total range ~190° (no slip ring, wires will tangle if exceeded)
    public static double minAngleDeg = -95.0;   // Minimum angle (left limit)
    public static double maxAngleDeg = 95.0;    // Maximum angle (right limit)
    
    // Unwind threshold: if target is beyond this angle, turret will return to 0°
    // This prevents the turret from "chasing" targets behind the robot
    public static double unwindThreshold = 100.0;  // degrees - if |targetAngle| > this, unwind to 0°
    
    // ==================== GEOMETRY (Limelight on turret) ====================
    // Turret center is behind chassis center
    public static double turretOffsetMM = 47.0;           // mm, turret center behind chassis center
    // Limelight distance from turret center
    public static double limelightOffsetMM = 140.86521;   // mm, limelight from turret center
    
    // ==================== POSITION PID (Soft Lock) ====================
    // PID constants for position control (in TICKS, not degrees!)
    // Tuned via TurretMotorTuner on 2026-02-02
    // IMPORTANT: Motor direction is REVERSED (reverseMotor = true)
    public static double kP = 0.0004;   // Tuned 2026-02-02
    public static double kI = 0.0;      // Keep at 0
    public static double kD = 0.0000185;// Tuned 2026-02-02
    public static double kF = 0.058;    // Static friction compensation, direction-aware
    
    // Position control parameters
    public static double positionTolerance = 100.0;  // Ticks, within this = at setpoint (~0.83°)
    public static double maxOutputPower = 1.0;       // Maximum output from PID
    public static double minOutputPower = 0.0;       // Minimum output
    
    // Motor direction: true = REVERSE, false = FORWARD
    // Determined by tuning - ensures encoder and motor direction match
    public static boolean reverseMotor = true;
    
    // ==================== HARD LOCK (Absolute Position Tracking) ====================
    // Uses robot absolute position to calculate turret angle to goal
    
    // Goal positions (field coordinates, inches) - BASKET CENTER
    public static double blueGoalX = 4.0;     // Blue basket X
    public static double blueGoalY = 140.0;   // Blue basket Y
    public static double redGoalX = 140.0;    // Red basket X
    public static double redGoalY = 140.0;    // Red basket Y
    
    // AprilTag positions (field coordinates, inches) - TAG POSITION (NOT basket!)
    // Tags are in front of the baskets, so TX tracking sees the tag, not the basket
    public static double blueTagX = 17.0;     // Blue tag (ID 20) X
    public static double blueTagY = 131.0;    // Blue tag (ID 20) Y
    public static double redTagX = 127.0;     // Red tag (ID 24) X  (144 - 17)
    public static double redTagY = 131.0;     // Red tag (ID 24) Y
    
    // Calibration speed (for finding zero position)
    public static double calibrationSpeed = 0.2;
}



// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
