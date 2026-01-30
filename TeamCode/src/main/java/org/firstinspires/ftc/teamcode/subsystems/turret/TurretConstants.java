package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for the Turret (Gimbal) subsystem.
 * All values are tunable via FTC Dashboard.
 */
@Config
public class TurretConstants {
    // Hardware name (must match robot configuration)
    public static String turretMotorName = "turretMotor";
    
    // Motor power limits
    public static double maxPower = 1.0;        // Maximum motor power
    public static double minPower = -1.0;       // Minimum motor power (reverse)
    
    // Position limits (encoder ticks, adjust based on your setup)
    public static int minPosition = 0;          // Minimum encoder position
    public static int maxPosition = 1000;       // Maximum encoder position
    
    // PID constants for position control (if using)
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;
}


