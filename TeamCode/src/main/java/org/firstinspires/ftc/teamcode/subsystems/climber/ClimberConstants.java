package org.firstinspires.ftc.teamcode.subsystems.climber;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for the Climber subsystem.
 * All values are tunable via FTC Dashboard.
 */
@Config
public class ClimberConstants {
    // Hardware names (must match robot configuration)
    public static String leftClimberServoName = "leftClimberServo";
    public static String rightClimberServoName = "rightClimberServo";
    
    // Servo positions
    public static double leftRetractedPos = 0.0;   // Left servo retracted position
    public static double leftExtendedPos = 1.0;    // Left servo extended position
    public static double rightRetractedPos = 0.0;  // Right servo retracted position
    public static double rightExtendedPos = 1.0;   // Right servo extended position
}

