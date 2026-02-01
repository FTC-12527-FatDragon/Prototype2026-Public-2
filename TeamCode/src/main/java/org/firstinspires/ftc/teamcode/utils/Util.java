package org.firstinspires.ftc.teamcode.utils;

/**
 * General utility functions.
 * Includes vision pose conversion, deadband processing, and epsilon comparison.
 */

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
public class Util {
    // Adjustable offsets for vision calibration (tune these in FTC Dashboard)
    public static double visionXOffset = 72;  // X offset in inches
    public static double visionYOffset = 72;  // Y offset in inches
    public static double visionHeadingOffset = -Math.PI / 2;  // Heading offset in radians
    public static boolean swapXY = true;  // Whether to swap X and Y from Limelight
    public static boolean negateX = false;  // Whether to negate X
    public static boolean negateY = true;   // Whether to negate Y (original: true)
    
    public static Pose Pose2DToPose(Pose2D pose2D) {
        return new Pose(pose2D.getX(DistanceUnit.INCH),
                pose2D.getY(DistanceUnit.INCH),
                pose2D.getHeading(AngleUnit.RADIANS));
    }

    public static boolean epsilonEqual(double a, double b, double epsilon) {
        return Math.abs(a - b) <= epsilon;
    }
    
    /**
     * Normalizes an angle in degrees to the range [-180, 180].
     * @param angle Angle in degrees.
     * @return Normalized angle in degrees.
     */
    public static double normalizeAngleDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    
    /**
     * Normalizes an angle in radians to the range [-π, π].
     * @param angle Angle in radians.
     * @return Normalized angle in radians.
     */
    public static double normalizeAngleRadians(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    /**
     * Converts Limelight Pose3D (meters, field-space) to Pinpoint Pose2D (inches).
     * Uses configurable offsets and axis swapping for easy tuning.
     *
     * @param pose3D The Pose3D from Limelight (in meters and degrees)
     * @return Pose2D for Pinpoint (in inches and radians)
     */
    public static Pose2D visionPoseToPinpointPose(Pose3D pose3D) {
        double llX = Units.metersToInches(pose3D.getPosition().x);
        double llY = Units.metersToInches(pose3D.getPosition().y);
        
        // Apply axis swapping
        double rawX, rawY;
        if (swapXY) {
            rawX = llY;
            rawY = llX;
        } else {
            rawX = llX;
            rawY = llY;
        }
        
        // Apply negation
        if (negateX) rawX = -rawX;
        if (negateY) rawY = -rawY;
        
        // Apply offsets
        double finalX = rawX + visionXOffset;
        double finalY = rawY + visionYOffset;
        
        // Heading conversion (getYaw returns degrees, convert to radians)
        double heading = pose3D.getOrientation().getYaw() / 180.0 * Math.PI + visionHeadingOffset;
        
        return new Pose2D(
                DistanceUnit.INCH,
                finalX,
                finalY,
                AngleUnit.RADIANS,
                heading
        );
    }
    
    /**
     * Returns a debug string showing the conversion steps.
     */
    public static String debugVisionConversion(Pose3D pose3D) {
        double llX = Units.metersToInches(pose3D.getPosition().x);
        double llY = Units.metersToInches(pose3D.getPosition().y);
        
        double rawX, rawY;
        if (swapXY) {
            rawX = llY;
            rawY = llX;
        } else {
            rawX = llX;
            rawY = llY;
        }
        
        if (negateX) rawX = -rawX;
        if (negateY) rawY = -rawY;
        
        double finalX = rawX + visionXOffset;
        double finalY = rawY + visionYOffset;
        
        return String.format("LL(%.1f,%.1f) -> swap(%s) -> negate(X:%s,Y:%s) -> +offset(%.0f,%.0f) -> (%.1f,%.1f)",
                llX, llY, swapXY, negateX, negateY, visionXOffset, visionYOffset, finalX, finalY);
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
