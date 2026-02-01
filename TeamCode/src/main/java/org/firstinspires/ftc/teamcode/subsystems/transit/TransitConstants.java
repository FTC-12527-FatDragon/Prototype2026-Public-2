package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TransitConstants {
    public static String transitServoName = "transitServo";
    public static String limitServoName = "limitServo";

    public static double transitUpPos = 0.36;    // Firing
    public static double transitDownPos = 0.62;  // Not firing
    
    // Limit servo positions
    public static double limitOpenPos = 0.6;     // Open
    public static double limitClosedPos = 0.3;   // Closed
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
