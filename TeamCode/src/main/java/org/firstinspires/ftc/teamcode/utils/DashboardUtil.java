package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Utility class for FTC Dashboard visualization.
 */
public class DashboardUtil {

    private static final double ROBOT_RADIUS = 9.0; // Inches
    private static final String ROBOT_COLOR = "#3F51B5"; // Blue

    /**
     * Draws the robot's position and heading on the Dashboard field overlay.
     * @param packet The TelemetryPacket to draw on.
     * @param pose The current pose of the robot (assumed to be in Inches and Radians/Degrees handled by Pose2D).
     */
    public static void drawRobot(TelemetryPacket packet, Pose2D pose) {
        Canvas fieldOverlay = packet.fieldOverlay();
        
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke(ROBOT_COLOR);
        
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);

        // Draw robot body (circle)
        fieldOverlay.strokeCircle(x, y, ROBOT_RADIUS);
        
        // Draw heading vector
        double endX = x + ROBOT_RADIUS * Math.cos(heading);
        double endY = y + ROBOT_RADIUS * Math.sin(heading);
        fieldOverlay.strokeLine(x, y, endX, endY);
    }
}


// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
