package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Units;

/**
 * Constants for the Drive subsystem.
 * Includes motor names, localization offsets, and drive behavior parameters.
 *
 */
@Config
public class DriveConstants {
    // Motor hardware map names
    public static String leftFrontMotorName = "leftFrontMotor";
    public static String leftBackMotorName = "leftBackMotor";
    public static String rightFrontMotorName = "rightFrontMotor";
    public static String rightBackMotorName = "rightBackMotor";

    // Pinpoint (Dead Wheels) Offsets
    // These values represent the position of the odometry pods relative to the robot's center.
    // xPoseDW corresponds to the Strafe Pod's X offset.
    // yPoseDW corresponds to the Forward Pod's Y offset.
    // Values are converted from mm (measured) to inches.
    public static double xPoseDW = -93.45 / 25.4, yPoseDW = 24.05 / 25.4, headingDW = Math.PI;

    // Drive behavior constants
    public static double strafingBalance = 1.1; // Multiplier to correct strafing drift/inefficiency
    public static double headingEpsilon = 0.1;  // Tolerance for heading checks (radians)
    public static DistanceUnit distanceUnit = DistanceUnit.INCH;
    public static AngleUnit angleUnit = AngleUnit.RADIANS;

    // Pedro Pathing / Tuning Constants (Empirically determined)
    public static double linearScalar = -1.1404723265, angularScalar = 0.99446;
    public static double forwardVelocity = 05.7063, strafeVelocity = 68.933;
    public static double forwardAcceleration = -29.424, strafeAcceleration = -60.009;

    // PID Coefficients (if used for active control)
    public static double kP_xy = 0.02;
    public static double kP_h = -0.8;
    
    // Position Hold (Brake) PID Coefficients
    public static double kP_brakeXY = 0.05;  // P gain for XY position hold
    public static double kP_brakeH = 0.8;    // P gain for heading hold

    // Gamepad Input Deadband
    // Minimum input value required to trigger robot movement.
    public static double deadband = 0.03;
    
    // D-Pad Turn Speed
    // Speed for rotation when using D-Pad Left/Right
    public static double dpadTurnSpeed = 0.3;
    
    // ==================== AUTO-AIM TARGET COORDINATES ====================
    // Goal positions for auto-aim (in inches) - BASKET CENTER
    // Blue basket: (4, 140)
    // Red basket: (140, 140)
    public static double blueGoalX = 4;
    public static double blueGoalY = 140;
    public static double redGoalX = 140;
    public static double redGoalY = 140;
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
