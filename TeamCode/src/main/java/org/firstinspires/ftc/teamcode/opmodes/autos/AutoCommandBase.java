package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
// import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;  // DISABLED: Vision not used in auto currently

/**
 * Base class for Autonomous OpModes.
 * Handles initialization of subsystems, telemetry, and the command scheduler loop.
 * NOTE: Do NOT initialize MecanumDrivePinpoint here - it would reset Pinpoint and conflict with Follower!
 */
public abstract class AutoCommandBase extends LinearOpMode {
    protected Shooter shooter;
    protected Transit transit;
    protected Intake intake;
    protected Follower follower;
    // protected Vision vision;  // DISABLED: Vision not used in auto currently
    protected Turret turret;
    
    // Dashboard for drawing robot position
    protected FtcDashboard dashboard;
    
    // Robot dimensions (inches) for drawing
    private static final double ROBOT_WIDTH = 14.187;
    private static final double ROBOT_LENGTH = 15.748;

    /**
     * Abstract method to define the autonomous command sequence.
     * @return The Command to run.
     */
    public abstract Command runAutoCommand();

    /**
     * Abstract method to define the starting pose of the robot.
     * @return The starting Pose.
     */
    public abstract Pose getStartPose();

    /**
     * Initializes subsystems and telemetry.
     */
    private void initialize() {
        // Setup Dashboard for drawing
        dashboard = FtcDashboard.getInstance();
        
        // Setup MultipleTelemetry to display on both Driver Station and Dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize Follower with hardware map
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getStartPose());

        // Initialize subsystems
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap);
        // vision = new Vision(hardwareMap);  // DISABLED: Vision not used in auto currently
        turret = new Turret(hardwareMap);
        // NOTE: MecanumDrivePinpoint is NOT initialized here to avoid resetting Pinpoint and conflicting with Follower
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        // Get the command sequence defined in the child class
        Command toRun = runAutoCommand();

        // Schedule the command
        CommandScheduler.getInstance().schedule(toRun);

        waitForStart();
        
        // Start intake at full power for entire auto
        intake.setFullPower(true);
        intake.startIntake();
        
        // Start shooter idle at SLOW (close shot) speed for entire auto
        // This ensures shooter is always spinning and ready to fire
        shooter.setShooterState(Shooter.ShooterState.SLOW);

        // Main Loop (aligned with Prototype2026-Public)
        while (opModeIsActive() && !isStopRequested()) {
            // Get current position
            Pose currentPose = follower.getPose();
            
            // === SAFETY CHECK: Position bounds ===
            // If X or Y < -10, something is seriously wrong with localization
            if (currentPose.getX() < AutoConstants.POSITION_LOWER_BOUND || 
                currentPose.getY() < AutoConstants.POSITION_LOWER_BOUND) {
                // Emergency stop!
                follower.breakFollowing();  // Stop path following
                shooter.setShooterState(Shooter.ShooterState.STOP);
                intake.setReversed(false);
                intake.setFullPower(false);
                
                // Display error message
                telemetry.clearAll();
                telemetry.addLine("========== ERROR ==========");
                telemetry.addLine("Auto Localization Error!");
                telemetry.addLine("Position out of bounds!");
                telemetry.addLine("===========================");
                telemetry.addData("X", String.format("%.2f", currentPose.getX()));
                telemetry.addData("Y", String.format("%.2f", currentPose.getY()));
                telemetry.addLine("Robot stopped for safety.");
                telemetry.update();
                
                // Stay here until stop is pressed
                while (opModeIsActive() && !isStopRequested()) {
                    sleep(100);
                }
                break;  // Exit main loop
            }
            
            // Run the CommandScheduler to execute scheduled commands
            // follower.update() is now called inside AutoDriveCommand.execute()
            CommandScheduler.getInstance().run();

            // Real-time position telemetry
            telemetry.addData("X", String.format("%.2f", currentPose.getX()));
            telemetry.addData("Y", String.format("%.2f", currentPose.getY()));
            telemetry.addData("Heading (deg)", String.format("%.1f", Math.toDegrees(currentPose.getHeading())));
            telemetry.addData("Path Active", follower.isBusy());
            telemetry.update();
            
            // Draw robot on Dashboard
            drawRobotOnDashboard(currentPose);
        }

        onAutoStopped();
        CommandScheduler.getInstance().reset();
    }

    /**
     * Executes when auto is stopped. Can be overridden for cleanup.
     */
    public void onAutoStopped() {
        // Stop intake when auto ends
        intake.stopIntake();
        intake.setFullPower(false);
        
        // Stop shooter
        shooter.setShooterState(Shooter.ShooterState.STOP);
        
        // Return turret to 0
        turret.enableSoftLock(0);
    }
    
    /**
     * Draws the robot position and heading on FTC Dashboard Field view.
     * Dashboard Field uses center-origin coordinates (-72 to 72 inches).
     * Our pose uses corner-origin coordinates (0 to 144 inches).
     */
    private void drawRobotOnDashboard(Pose pose) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        
        // Convert from corner-origin (0-144) to center-origin (-72 to 72)
        double x = pose.getX() - 72;
        double y = pose.getY() - 72;
        double heading = pose.getHeading();
        
        // Draw robot as a circle with direction indicator
        canvas.setStroke("#00FF00");  // Green
        canvas.setStrokeWidth(1);
        canvas.strokeCircle(x, y, ROBOT_WIDTH / 2);
        
        // Draw heading direction line (front of robot)
        canvas.setStroke("#FF0000");  // Red
        canvas.setStrokeWidth(2);
        double arrowLength = ROBOT_LENGTH * 0.7;
        double arrowX = x + arrowLength * Math.cos(heading);
        double arrowY = y + arrowLength * Math.sin(heading);
        canvas.strokeLine(x, y, arrowX, arrowY);
        
        // Add telemetry data
        packet.put("X (in)", String.format("%.2f", pose.getX()));
        packet.put("Y (in)", String.format("%.2f", pose.getY()));
        packet.put("Heading (deg)", String.format("%.1f", Math.toDegrees(heading)));
        
        dashboard.sendTelemetryPacket(packet);
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
