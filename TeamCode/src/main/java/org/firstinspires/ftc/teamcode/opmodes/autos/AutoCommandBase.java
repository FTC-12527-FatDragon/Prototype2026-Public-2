package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

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
    protected Vision vision;

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
        // Setup MultipleTelemetry to display on both Driver Station and Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Follower with hardware map
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getStartPose());

        // Initialize subsystems
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap);
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
        }

        onAutoStopped();
        CommandScheduler.getInstance().reset();
    }

    /**
     * Executes when auto is stopped. Can be overridden for cleanup.
     */
    public void onAutoStopped() {}
}
