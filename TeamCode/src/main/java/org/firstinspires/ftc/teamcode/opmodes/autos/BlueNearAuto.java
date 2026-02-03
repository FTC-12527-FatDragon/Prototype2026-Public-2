package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.WaitForTurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

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
    private static final Pose START_POSE = new Pose(25.68, 127.97, Math.toRadians(90));
    private static final Pose SHOOT_POSE = new Pose(45.20, 101.15, Math.toRadians(180));
    
    private static final Pose SAMPLE1_POSE = new Pose(8.79, 59.30, Math.toRadians(180));
    private static final Point SAMPLE1_CTRL = new Point(67.62, 56.74);
    
    private static final Pose INTAKE1_POSE = new Pose(16.45, 69.66, Math.toRadians(180));
    private static final Point INTAKE1_CTRL = new Point(26.89, 61.55);
    
    private static final Pose INTAKE2_POSE = new Pose(16.17, 83.46, Math.toRadians(180));
    private static final Point INTAKE2_CTRL = new Point(46.74, 81.85);
    
    private static final Pose SAMPLE2_POSE = new Pose(14.19, 35.31, Math.toRadians(180));
    private static final Point SAMPLE2_CTRL1 = new Point(63.62, 43.59);
    private static final Point SAMPLE2_CTRL2 = new Point(70.33, 33.14);
    
    private static final Pose FINAL_INTAKE_POSE = new Pose(15.96, 101.11, Math.toRadians(180));
    
    // Wait times (ms)
    public static long INTAKE_WAIT_MS = 500;
    public static long SHOOT_WAIT_MS = 1500;
    public static long TURRET_TIMEOUT_MS = 1000;
    
    // Blue Near: +43.3° (aim right toward blue basket at 4, 140)
    public static double TURRET_SHOOT_ANGLE_DEG = 43.3;
    
    @Override
    public Pose getStartPose() {
        return START_POSE;
    }
    
    private Command shootAfterTurretReady() {
        return new SequentialCommandGroup(
                new WaitForTurretCommand(turret, TURRET_TIMEOUT_MS),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new WaitCommand(SHOOT_WAIT_MS),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
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
                .addPath(new BezierLine(
                        new Point(START_POSE.getX(), START_POSE.getY()),
                        new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        
        // Path 2: Shoot → Sample 1
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                        SAMPLE1_CTRL,
                        new Point(SAMPLE1_POSE.getX(), SAMPLE1_POSE.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 3: Sample 1 → Intake 1
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(SAMPLE1_POSE.getX(), SAMPLE1_POSE.getY()),
                        INTAKE1_CTRL,
                        new Point(INTAKE1_POSE.getX(), INTAKE1_POSE.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 4: Intake 1 → Shoot
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(INTAKE1_POSE.getX(), INTAKE1_POSE.getY()),
                        new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 5: Shoot → Intake 2
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                        INTAKE2_CTRL,
                        new Point(INTAKE2_POSE.getX(), INTAKE2_POSE.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 6: Intake 2 → Shoot
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(INTAKE2_POSE.getX(), INTAKE2_POSE.getY()),
                        new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 7: Shoot → Sample 2
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                        SAMPLE2_CTRL1,
                        SAMPLE2_CTRL2,
                        new Point(SAMPLE2_POSE.getX(), SAMPLE2_POSE.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 8: Sample 2 → Shoot
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(SAMPLE2_POSE.getX(), SAMPLE2_POSE.getY()),
                        new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 9: Shoot → Final
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                        new Point(FINAL_INTAKE_POSE.getX(), FINAL_INTAKE_POSE.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        return new SequentialCommandGroup(
                // Path 1: Go to shoot position
                new AutoDriveCommand(follower, path1),
                
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
                
                // Path 9: Final position
                new AutoDriveCommand(follower, path9)
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
