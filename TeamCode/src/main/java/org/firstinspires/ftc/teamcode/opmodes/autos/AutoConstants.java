package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.geometry.Pose;

/**
 * Stores coordinates and poses for Autonomous OpModes.
 * Values restored to "Sane" Blue Near coordinates.
 */
public class AutoConstants {

    // =========================================================
    // Safety Bounds
    // =========================================================
    // If robot position goes below this value, localization is likely wrong
    // Auto will emergency stop to prevent damage
    public static final double POSITION_LOWER_BOUND = -10.0;  // inches

    // Helper to convert degrees to radians
    public static double toRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    // =========================================================
    // Start Poses
    // =========================================================
    
    // NEAR Side (Basket Side)
    public static final Pose BLUE_START_POSE = new Pose(24.854, 128.901, toRadians(144));
    // Mirror X: 144 - 24.854 = 119.146, Heading: 180 - 144 = 36
    public static final Pose RED_START_POSE = new Pose(119.146, 128.901, toRadians(36));

    // FAR Side
    public static final Pose BLUE_FAR_START_POSE = new Pose(47.892, 8.063, toRadians(90));
    public static final Pose RED_FAR_START_POSE = new Pose(96.108, 8.063, toRadians(90));

    // =========================================================
    // NEAR Side: Scoring & Samples
    // =========================================================
    
    // Scoring Pose (Basket) - The critical one!
    // Using reasonable values from previous context
    public static final Pose BLUE_BASKET_POSE = new Pose(17.5, 128, toRadians(135));
    public static final Pose RED_BASKET_POSE = new Pose(126.5, 128, toRadians(45));

    // =========================================================
    // Sample Pickup Poses
    // =========================================================
    
    // Sample 1
    // Using Sample 1 Y ~ 120-130 range or pickup Y ~ 80?
    // User JSON Sample 1 was Y=83.87. That seems reasonable for a sample on the floor.
    // Start -> Intermediate -> Sample 1
    public static final Pose BLUE_SAMPLE_1_INTERMEDIATE = new Pose(42.071, 100.0, toRadians(180)); // Guessing Y to be safe
    public static final Pose RED_SAMPLE_1_INTERMEDIATE = new Pose(101.929, 100.0, toRadians(0));

    public static final Pose BLUE_SAMPLE_1_POSE = new Pose(23.785, 83.87, toRadians(180));
    public static final Pose RED_SAMPLE_1_POSE = new Pose(120.215, 83.87, toRadians(0));

    // Sample 2
    public static final Pose BLUE_SAMPLE_2_INTERMEDIATE = new Pose(42.492, 126.702, toRadians(180)); // Restored from history
    public static final Pose RED_SAMPLE_2_INTERMEDIATE = new Pose(101.508, 126.702, toRadians(0));
    
    public static final Pose BLUE_SAMPLE_2_POSE = new Pose(24.052, 116.326, toRadians(180)); // Restored from history
    public static final Pose RED_SAMPLE_2_POSE = new Pose(119.948, 116.326, toRadians(0));

    // Sample 3
    public static final Pose BLUE_SAMPLE_3_INTERMEDIATE = new Pose(42.091, 127.466, toRadians(180)); // Restored from history
    public static final Pose RED_SAMPLE_3_INTERMEDIATE = new Pose(101.909, 127.466, toRadians(0));
    
    public static final Pose BLUE_SAMPLE_3_POSE = new Pose(23.918, 115.15, toRadians(180)); // Restored from history
    public static final Pose RED_SAMPLE_3_POSE = new Pose(120.082, 115.15, toRadians(0));

    // =========================================================
    // Gate / Far Shoot / Park
    // =========================================================
    public static final Pose BLUE_GATE_POSE = new Pose(15.266, 69.725, toRadians(0));
    public static final Pose RED_GATE_POSE = new Pose(128.734, 69.725, toRadians(180));

    public static final Pose BLUE_FAR_SHOOT_POSE = new Pose(62.889, 16.287, toRadians(203.25));
    public static final Pose RED_FAR_SHOOT_POSE = new Pose(81.111, 16.287, toRadians(-23.25));

    public static final Pose BLUE_PARK_POSE = new Pose(60, 96, toRadians(90)); // Approximate park
    public static final Pose RED_PARK_POSE = new Pose(84, 96, toRadians(90));
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
