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
import org.firstinspires.ftc.teamcode.commands.autocommands.WaitForTurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Blue Near Infinite - Extended sample collection auto (Blue Alliance, Near side)
 * 
 * Turret starts moving during path to shoot position (saves time).
 * Shooting only starts after turret reaches target angle.
 */
@Config
@Autonomous(name = "Blue Near Infinite", group = "Autos")
public class BlueNearInfinite extends AutoCommandBase {
    
    private PathChain path1, path2, path3, path4;
    private PathChain pathBN5, pathBN6, pathBN7, pathBN8, pathBN9;
    
    // Key positions (Blue side)
    private static final Pose START_POSE = new Pose(25.68, 127.97, Math.toRadians(143.5));
    private static final Pose SHOOT_POSE = new Pose(45.20, 101.15, Math.toRadians(180));
    
    private static final Pose SAMPLE1_POSE = new Pose(8.79, 59.30, Math.toRadians(180));
    private static final Pose SAMPLE1_CTRL = new Pose(67.62, 56.74);
    
    private static final Pose INTAKE1_POSE = new Pose(16.45, 69.66, Math.toRadians(180));
    private static final Pose INTAKE1_CTRL = new Pose(26.89, 61.55);
    
    private static final Pose SAMPLE2_POSE = new Pose(12.08, 61.02, Math.toRadians(145));
    private static final Pose SAMPLE2_CTRL = new Pose(31.89, 62.83);
    
    private static final Pose BN_INTAKE2_POSE = new Pose(16.17, 83.46, Math.toRadians(180));
    private static final Pose BN_INTAKE2_CTRL = new Pose(46.74, 81.85);
    
    private static final Pose BN_SAMPLE2_POSE = new Pose(14.19, 35.31, Math.toRadians(180));
    private static final Pose BN_SAMPLE2_CTRL1 = new Pose(63.62, 43.59);
    private static final Pose BN_SAMPLE2_CTRL2 = new Pose(70.33, 33.14);
    
    private static final Pose BN_FINAL_POSE = new Pose(15.96, 101.11, Math.toRadians(180));
    
    // Wait times (ms)
    public static long INTAKE_WAIT_MS = 500;
    public static long SHOOT_WAIT_MS = 1500;
    public static long TURRET_TIMEOUT_MS = 1000;
    
    // Blue: +43.3° (aim right toward blue basket at 4, 140)
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
    
    private PathChain buildPath5() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE2_CTRL, SAMPLE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                .build();
    }
    
    private PathChain buildPath6() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SAMPLE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();
    }
    
    private Command sample2CycleCommand() {
        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, buildPath5()),
                intakeWaitCommand(),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, buildPath6()),
                shootAfterTurretReady()
        );
    }
    
    @Override
    public Command runAutoCommand() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE1_CTRL, SAMPLE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(SAMPLE1_POSE, INTAKE1_CTRL, INTAKE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE1_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        pathBN5 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, BN_INTAKE2_CTRL, BN_INTAKE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        pathBN6 = follower.pathBuilder()
                .addPath(new BezierLine(BN_INTAKE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        pathBN7 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, BN_SAMPLE2_CTRL1, BN_SAMPLE2_CTRL2, BN_SAMPLE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        pathBN8 = follower.pathBuilder()
                .addPath(new BezierLine(BN_SAMPLE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        pathBN9 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, BN_FINAL_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, path1),
                
                new AutoDriveCommand(follower, path2),
                new AutoDriveCommand(follower, path3),
                intakeWaitCommand(),
                
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path4),
                shootAfterTurretReady(),
                
                // Sample 2 cycles (turret moves during path to shoot)
                sample2CycleCommand(),
                sample2CycleCommand(),
                sample2CycleCommand(),
                
                // BlueNear continuation
                new AutoDriveCommand(follower, pathBN5),
                intakeWaitCommand(),
                
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, pathBN6),
                shootAfterTurretReady(),
                
                new AutoDriveCommand(follower, pathBN7),
                intakeWaitCommand(),
                
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, pathBN8),
                shootAfterTurretReady(),
                
                new AutoDriveCommand(follower, pathBN9)
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
