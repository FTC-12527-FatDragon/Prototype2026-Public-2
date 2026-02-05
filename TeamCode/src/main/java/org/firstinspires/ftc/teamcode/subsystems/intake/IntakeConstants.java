package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for the Intake subsystem.
 */
@Config
public class IntakeConstants {
    // Hardware map name for the intake motor
    public static String intakeMotorName = "intakeMotor";

    // Power levels
    public static double intakePower = 0.6;       // Standard running power (+0.1)
    public static double fullPower = 0.65;         // Power for intake during path2 and path5
    public static double fastShootingPower = 0.8; // Power during approach to shoot pose in auto
    public static double transitPower = 1;     // Power when transferring/shooting
    public static double fastIntakePower = 1.0; // Power for fast intaking (Left Trigger)
    /**
     * Motor Specifications:
     * GoBilda 5203-2402-0051 (1150RPM, 5.2:1 Ratio)
     * Resolution: 145.1 PPR
     * Max TPS = (1150 / 60) * 145.1 ≈ 2781 TPS
     */
    public static double maxVelocityTPS = 2781.0;

    // Jamming protection constants (currently unused in logic)
    // Threshold ratio for jamming detection (20% of max speed)
    public static double jammingThresholdRatio = 0.2;
    // Power to apply when jamming is detected to clear or protect motor
    public static double jammingPower = 0.5;
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
