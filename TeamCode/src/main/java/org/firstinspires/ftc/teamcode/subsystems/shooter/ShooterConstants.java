package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";
    public static String shooterServoName = "shooterServo";

    public static double shooterEpsilon = 20;

    public static double kP = 0.5;
    public static double kI = 0;
    public static double kD = 0;

    /**
     * In Ticks Per Second
     */
    public static double stopVelocity = 0;
    public static double fastVelocity = 1520;
    public static double slowVelocity = 1300;
    public static double releaseVelocity = 1000;

    public static double shooterServoUpPos = 0.59;
    public static double shooterServoDownPos = 0.0;
}
