package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
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
    protected Turret turret;
    
    // Dashboard for drawing robot position
    protected FtcDashboard dashboard;
    
    // Robot dimensions (inches) for drawing
    private static final double ROBOT_WIDTH = 14.187;
    private static final double ROBOT_LENGTH = 15.748;

    // ── Shared shooting constants ──
    protected static final long SHOOTER_SPINUP_TIMEOUT_MS = 1500;
    protected static final long TRANSIT_OPEN_MS = 1000;
    protected static final long TURRET_SETTLE_MS = 200;

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
        vision = new Vision(hardwareMap);
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
        
        // Shooter starts at IDLE (STOP state = low idle power)
        // Each shot sequence will accelerate to SLOW, then return to STOP after firing
        shooter.setShooterState(Shooter.ShooterState.STOP);

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

    // ─────────────────────────────────────────────────────────────
    //  Shared Shooting Sequence
    // ─────────────────────────────────────────────────────────────

    /**
     * Creates a complete shoot command sequence usable by any auto.
     * <p>
     * Sequence:
     * 1. TX auto-aim — if Limelight sees a goal tag, fine-tune turret with TX offset;
     *    otherwise ensure turret is at the requested base angle.
     * 2. Wait for turret to settle (200 ms).
     * 3. Accelerate shooter to the requested state.
     * 4. Fire via TransitCommand (continuously checks shooter speed).
     * 5. Return shooter to STOP and turret to 0°.
     *
     * @param turretAngleDeg Target turret angle in degrees (sign = alliance).
     * @param shooterState   Shooter speed to use (MID for near, FAST for far).
     * @return A self-contained shoot Command.
     */
    protected Command createShootSequence(double turretAngleDeg,
                                          Shooter.ShooterState shooterState) {
        return new SequentialCommandGroup(
                // 1. TX auto-aim or fall back to base angle
                new InstantCommand(() -> {
                    if (vision != null && vision.hasTarget()) {
                        turret.enableSoftLock(turretAngleDeg + vision.getTx());
                    } else {
                        turret.enableSoftLock(turretAngleDeg);
                    }
                }),
                new WaitCommand(TURRET_SETTLE_MS),
                // 2. Accelerate shooter
                new InstantCommand(() -> shooter.setShooterState(shooterState)),
                // 3. Fire (TransitCommand checks speed continuously)
                new TransitCommand(transit, shooter)
                        .withTimeout(SHOOTER_SPINUP_TIMEOUT_MS + TRANSIT_OPEN_MS),
                // 4. Clean up
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new InstantCommand(() -> turret.enableSoftLock(0))
        );
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
