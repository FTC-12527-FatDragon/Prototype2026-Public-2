package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.WaitForShooterCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.WaitForTurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/**
 * Blue Near Auto - Sample collection auto (Blue Alliance, Near side)
 * 
 * Turret starts moving during path to shoot position (saves time).
 * Shooting only starts after turret reaches target angle.
 */
@Config
@Autonomous(name = "Blue Near Auto", group = "Autos")
public class BlueNearAuto extends AutoCommandBase {
    
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;
    
    // Key positions (Blue side)
    private static final Pose START_POSE = new Pose(25.68, 127.97, Math.toRadians(143.5));
    private static final Pose SHOOT_POSE = new Pose(45.20, 101.15, Math.toRadians(180));
    
    private static final Pose SAMPLE1_POSE = new Pose(8.79, 59.30, Math.toRadians(180));
    private static final Pose SAMPLE1_CTRL = new Pose(67.62, 56.74);
    
    private static final Pose INTAKE1_POSE = new Pose(16.45, 69.66, Math.toRadians(180));
    private static final Pose INTAKE1_CTRL = new Pose(26.89, 61.55);
    
    private static final Pose INTAKE2_POSE = new Pose(16.17, 83.46, Math.toRadians(180));
    private static final Pose INTAKE2_CTRL = new Pose(46.74, 81.85);
    
    private static final Pose SAMPLE2_POSE = new Pose(14.19, 35.31, Math.toRadians(180));
    private static final Pose SAMPLE2_CTRL1 = new Pose(63.62, 43.59);
    private static final Pose SAMPLE2_CTRL2 = new Pose(70.33, 33.14);
    
    private static final Pose FINAL_INTAKE_POSE = new Pose(20.69, 64.06, Math.toRadians(180));
    
    // Wait times (ms)
    public static long INTAKE_WAIT_MS = 500;
    public static long SHOOTER_SPINUP_TIMEOUT_MS = 2000;  // Max wait for shooter to reach speed
    public static long TRANSIT_OPEN_MS = 1200;            // Time to keep transit open for ball to exit
    public static long TURRET_TIMEOUT_MS = 1000;
    
    // Blue Near: +43.3° (aim right toward blue basket at 4, 140)
    public static double TURRET_SHOOT_ANGLE_DEG = 43.3;
    
    @Override
    public Pose getStartPose() {
        return START_POSE;
    }
    
    /**
     * Shoot command - Shooter is already running in SLOW mode.
     * Uses same firing logic as Solo: waits for shooter speed, triggers boost, then fires.
     */
    private Command shootAfterTurretReady() {
        return new SequentialCommandGroup(
                // 1. Wait for turret to reach angle
                new WaitForTurretCommand(turret, TURRET_TIMEOUT_MS),
                // 2. Wait for shooter to reach target velocity
                new WaitForShooterCommand(shooter, 2000),
                // 3. Start firing boost timer (same as Solo LT + bumper)
                new InstantCommand(() -> shooter.setTransitFiring(true)),
                // 4. Wait for boost delay (200ms) before opening transit
                new WaitCommand(200),
                // 5. Open transit to release ball (boost now active)
                new InstantCommand(() -> transit.setTransitState(Transit.TransitState.UP)),
                // 6. Wait for ball to exit (boost continues ramping during this time)
                new WaitCommand(TRANSIT_OPEN_MS),
                // 7. Close transit
                new InstantCommand(() -> transit.setTransitState(Transit.TransitState.DOWN)),
                // 8. End firing boost
                new InstantCommand(() -> shooter.setTransitFiring(false)),
                // 9. Return turret to forward (shooter keeps running)
                new InstantCommand(() -> turret.enableSoftLock(0))
        );
    }
    
    private Command intakeWaitCommand() {
        return new WaitCommand(INTAKE_WAIT_MS);
    }
    
    @Override
    public Command runAutoCommand() {
        // Path 1: Start → Shoot
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), Math.toRadians(180))
                .build();
        
        // Path 2: Shoot → Sample 1
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE1_CTRL, SAMPLE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 3: Sample 1 → Intake 1
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(SAMPLE1_POSE, INTAKE1_CTRL, INTAKE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 4: Intake 1 → Shoot (curve)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(INTAKE1_POSE, new Pose(53.40, 64.83), SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 5: Shoot → Intake 2
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, INTAKE2_CTRL, INTAKE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 6: Intake 2 → Shoot
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 7: Shoot → Sample 2
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE2_CTRL1, SAMPLE2_CTRL2, SAMPLE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 8: Sample 2 → Shoot
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(SAMPLE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 9: Shoot → Final
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, FINAL_INTAKE_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        return new SequentialCommandGroup(
                // START: Keep shooter running at SLOW throughout entire auto
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                
                // Path 1: Go to shoot position + FIRST SHOT (preloaded ball)
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path1),
                shootAfterTurretReady(),
                
                // Path 2-3: Get sample 1
                new AutoDriveCommand(follower, path2),
                new AutoDriveCommand(follower, path3),
                intakeWaitCommand(),
                
                // Start turret, path 4 to shoot, then shoot
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path4),
                shootAfterTurretReady(),
                
                // Path 5: Get intake 2
                new AutoDriveCommand(follower, path5),
                intakeWaitCommand(),
                
                // Start turret, path 6 to shoot, then shoot
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path6),
                shootAfterTurretReady(),
                
                // Path 7: Get sample 2
                new AutoDriveCommand(follower, path7),
                intakeWaitCommand(),
                
                // Start turret, path 8 to shoot, then shoot
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path8),
                shootAfterTurretReady(),
                
                // Path 9: Final position + STOP shooter
                new AutoDriveCommand(follower, path9),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
