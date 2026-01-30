package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TransitConstants {
    public static String transitServoName = "transitServo";
    public static String limitServoName = "limitServo";

    public static double transitUpPos = 0.87;
    public static double transitDownPos = 0.67;
    
    // Limit servo positions (TODO: calibrate actual values)
    public static double limitOpenPos = 0;    // Open when transit is UP
    public static double limitClosedPos = 0;  // Closed when transit is DOWN
}
