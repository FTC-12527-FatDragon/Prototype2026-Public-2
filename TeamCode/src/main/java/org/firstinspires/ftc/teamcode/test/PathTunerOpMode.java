package org.firstinspires.ftc.teamcode.test;

/**
 * Path Tuner OpMode.
 * Visualizes auto paths on Panels Dashboard for path debugging and tuning.
 */

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;

/**
 * Path Tuner OpMode
 * Allows dynamic path testing via Panels Dashboard.
 * Supports both BezierLine and BezierCurve.
 * 
 * Instructions:
 * 1. Open Panels Dashboard (192.168.43.1:8001).
 * 2. Edit start/end/control point values in the Configuration sidebar.
 * 3. Set pathType: 0 = BezierLine, 1 = BezierCurve (1 ctrl), 2 = BezierCurve (2 ctrl), 3 = BezierCurve (3 ctrl)
 * 4. Init this OpMode. The robot will set its pose to 'startPose'.
 * 5. Press Start. The robot will drive to 'endPose'.
 * 6. Press A to re-run the path.
 */
@Config
@TeleOp(name = "Path Tuner", group = "Test")
public class PathTunerOpMode extends OpMode {

    private Follower follower;
    private FieldManager panelsField;
    private TelemetryManager telemetryM;

    private static final double ROBOT_RADIUS = 9;

    // Styles for drawing
    private static final Style pathStyle = new Style("", "#FF5722", 1.0); // Orange path
    private static final Style robotStyle = new Style("", "#4CAF50", 0.75); // Green robot
    private static final Style startStyle = new Style("", "#2196F3", 0.75); // Blue start
    private static final Style endStyle = new Style("", "#F44336", 0.75); // Red end
    private static final Style ctrlStyle = new Style("", "#FF9800", 0.5); // Orange control points

    // Path Type: 0 = Line, 1 = Curve (1 ctrl), 2 = Curve (2 ctrl), 3 = Curve (3 ctrl)
    public static int pathType = 0;

    // Start Pose
    public static double startX = 25.509;
    public static double startY = 129.474;
    public static double startHeadingDeg = 144.0;

    // End Pose
    public static double endX = 37.200;
    public static double endY = 110.342;
    public static double endHeadingDeg = 135.0;

    // Control Point 1 (for BezierCurve)
    public static double ctrl1X = 50.0;
    public static double ctrl1Y = 120.0;

    // Control Point 2 (for BezierCurve with 2+ control points)
    public static double ctrl2X = 60.0;
    public static double ctrl2Y = 100.0;

    // Control Point 3 (for BezierCurve with 3 control points)
    public static double ctrl3X = 45.0;
    public static double ctrl3Y = 90.0;

    private PathChain testPath;
    private boolean pathRunning = false;

    @Override
    public void init() {
        // Initialize Panels Dashboard
        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Follower
        follower = Constants.createFollower(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Dashboard", "Use Panels (192.168.43.1:8001)");
        telemetry.addData("Path Type", getPathTypeName());
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Draw preview path on Panels Dashboard
        drawExpectedPath();
        drawStartEndPoints();

        // Send to Panels
        panelsField.update();

        // Update Driver Station Telemetry
        telemetry.addData("Path Type", getPathTypeName());
        telemetry.addData("Start", String.format("(%.1f, %.1f, %.0f°)", startX, startY, startHeadingDeg));
        telemetry.addData("End", String.format("(%.1f, %.1f, %.0f°)", endX, endY, endHeadingDeg));
        if (pathType >= 1) telemetry.addData("Ctrl1", String.format("(%.1f, %.1f)", ctrl1X, ctrl1Y));
        if (pathType >= 2) telemetry.addData("Ctrl2", String.format("(%.1f, %.1f)", ctrl2X, ctrl2Y));
        if (pathType >= 3) telemetry.addData("Ctrl3", String.format("(%.1f, %.1f)", ctrl3X, ctrl3Y));
        telemetry.update();
    }

    @Override
    public void start() {
        buildAndFollowPath();
    }

    @Override
    public void loop() {
        follower.update();

        // Draw the expected path
        drawExpectedPath();
        drawStartEndPoints();

        // Draw Robot
        drawRobot(follower.getPose(), robotStyle);

        // Draw current path if following
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), pathStyle);
        }

        // Send to Panels
        panelsField.update();

        // Dashboard Telemetry via Panels
        Pose currentPose = follower.getPose();
        telemetryM.debug("Path Running: " + follower.isBusy());
        telemetryM.debug("X: " + currentPose.getX());
        telemetryM.debug("Y: " + currentPose.getY());
        telemetryM.debug("Heading: " + Math.toDegrees(currentPose.getHeading()));

        double distToEnd = Math.hypot(endX - currentPose.getX(), endY - currentPose.getY());
        telemetryM.debug("Dist to End: " + distToEnd);
        telemetryM.update(telemetry);

        // Driver Station Telemetry
        telemetry.addData("Path Running", follower.isBusy());
        telemetry.addData("Current X", currentPose.getX());
        telemetry.addData("Current Y", currentPose.getY());
        telemetry.addData("Current Deg", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Dist to End", distToEnd);
        telemetry.addData("Press A", "to re-run path");
        telemetry.update();

        // Press A to re-run path
        if (gamepad1.a && !pathRunning) {
            buildAndFollowPath();
        }

        if (!follower.isBusy()) {
            pathRunning = false;
        }
    }

    private void buildAndFollowPath() {
        Pose startPose = new Pose(startX, startY, Math.toRadians(startHeadingDeg));
        Pose endPose = new Pose(endX, endY, Math.toRadians(endHeadingDeg));

        follower.setStartingPose(startPose);

        switch (pathType) {
            case 0: // BezierLine
                testPath = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, endPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                        .build();
                break;

            case 1: // BezierCurve with 1 control point
                testPath = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                startPose,
                                new Pose(ctrl1X, ctrl1Y, 0),
                                endPose
                        ))
                        .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                        .build();
                break;

            case 2: // BezierCurve with 2 control points
                testPath = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                startPose,
                                new Pose(ctrl1X, ctrl1Y, 0),
                                new Pose(ctrl2X, ctrl2Y, 0),
                                endPose
                        ))
                        .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                        .build();
                break;

            case 3: // BezierCurve with 3 control points
            default:
                testPath = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                startPose,
                                new Pose(ctrl1X, ctrl1Y, 0),
                                new Pose(ctrl2X, ctrl2Y, 0),
                                new Pose(ctrl3X, ctrl3Y, 0),
                                endPose
                        ))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
                break;
        }

        follower.followPath(testPath);
        pathRunning = true;
    }

    private void drawExpectedPath() {
        // Draw the expected path based on type
        int samples = 30;

        for (int i = 0; i < samples; i++) {
            double t1 = (double) i / samples;
            double t2 = (double) (i + 1) / samples;

            double[] p1 = calculateBezierPoint(t1);
            double[] p2 = calculateBezierPoint(t2);

            panelsField.setStyle(pathStyle);
            panelsField.moveCursor(p1[0], p1[1]);
            panelsField.line(p2[0], p2[1]);
        }

        // Draw control points for curves
        if (pathType >= 1) {
            drawCircle(ctrl1X, ctrl1Y, 2, ctrlStyle);
            // Draw line from start to ctrl1
            panelsField.setStyle(ctrlStyle);
            panelsField.moveCursor(startX, startY);
            panelsField.line(ctrl1X, ctrl1Y);
        }
        if (pathType >= 2) {
            drawCircle(ctrl2X, ctrl2Y, 2, ctrlStyle);
        }
        if (pathType >= 3) {
            drawCircle(ctrl3X, ctrl3Y, 2, ctrlStyle);
            // Draw line from ctrl3 to end
            panelsField.setStyle(ctrlStyle);
            panelsField.moveCursor(ctrl3X, ctrl3Y);
            panelsField.line(endX, endY);
        }
    }

    private void drawStartEndPoints() {
        // Draw start point (blue)
        drawCircle(startX, startY, 4, startStyle);
        // Draw end point (red)
        drawCircle(endX, endY, 4, endStyle);
    }

    private void drawCircle(double x, double y, double radius, Style style) {
        panelsField.setStyle(style);
        panelsField.moveCursor(x, y);
        panelsField.circle(radius);
    }

    private void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        // Draw heading line
        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2;
        double y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent();
        double y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    private void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();
        if (points == null || points.length < 2) return;

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    private double[] calculateBezierPoint(double t) {
        double x, y;

        switch (pathType) {
            case 0: // Linear
                x = (1 - t) * startX + t * endX;
                y = (1 - t) * startY + t * endY;
                break;

            case 1: // Quadratic Bezier (1 control point)
                x = Math.pow(1 - t, 2) * startX + 2 * (1 - t) * t * ctrl1X + Math.pow(t, 2) * endX;
                y = Math.pow(1 - t, 2) * startY + 2 * (1 - t) * t * ctrl1Y + Math.pow(t, 2) * endY;
                break;

            case 2: // Cubic Bezier (2 control points)
                x = Math.pow(1 - t, 3) * startX + 3 * Math.pow(1 - t, 2) * t * ctrl1X
                        + 3 * (1 - t) * Math.pow(t, 2) * ctrl2X + Math.pow(t, 3) * endX;
                y = Math.pow(1 - t, 3) * startY + 3 * Math.pow(1 - t, 2) * t * ctrl1Y
                        + 3 * (1 - t) * Math.pow(t, 2) * ctrl2Y + Math.pow(t, 3) * endY;
                break;

            case 3: // Quartic Bezier (3 control points)
            default:
                double u = 1 - t;
                x = Math.pow(u, 4) * startX + 4 * Math.pow(u, 3) * t * ctrl1X
                        + 6 * Math.pow(u, 2) * Math.pow(t, 2) * ctrl2X
                        + 4 * u * Math.pow(t, 3) * ctrl3X + Math.pow(t, 4) * endX;
                y = Math.pow(u, 4) * startY + 4 * Math.pow(u, 3) * t * ctrl1Y
                        + 6 * Math.pow(u, 2) * Math.pow(t, 2) * ctrl2Y
                        + 4 * u * Math.pow(t, 3) * ctrl3Y + Math.pow(t, 4) * endY;
                break;
        }

        return new double[]{x, y};
    }

    private String getPathTypeName() {
        switch (pathType) {
            case 0: return "BezierLine";
            case 1: return "BezierCurve (1 ctrl)";
            case 2: return "BezierCurve (2 ctrl)";
            case 3: return "BezierCurve (3 ctrl)";
            default: return "Unknown";
        }
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
