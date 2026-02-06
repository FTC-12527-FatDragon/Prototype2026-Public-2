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

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.WaitForTurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/**
 * Red Far Auto - Continuous intake/shoot cycles (Red Alliance, Far side)
 * Mirrored from BlueFarAuto along x=72 axis
 * 
 * Turret starts moving during path to shoot position (saves time).
 * Shooting only starts after turret reaches target angle.
 */
@Config
@Autonomous(name = "Red Far Auto", group = "Autos")
public class RedFarAuto extends AutoCommandBase {
    
    // Path declarations
    private PathChain path1, path2, path3;
    
    // Key positions (Red side - mirrored at x=72)
    private static final Pose START_POSE = new Pose(89.03, 9.10, Math.toRadians(90));
    private static final Pose CURVE_END = new Pose(120.25, 7.30, Math.toRadians(0));
    private static final Pose CURVE_CONTROL = new Pose(104.14, 14.95);
    private static final Pose INTAKE_POSE = new Pose(134.40, 8.13, Math.toRadians(0));
    private static final Pose SHOOT_POSE = new Pose(89.03, 9.10, Math.toRadians(0));
    
    // Wait times (ms)
    public static long INTAKE_WAIT_MS = 1000;
    public static long SHOOTER_SPINUP_TIMEOUT_MS = 2000;
    public static long TRANSIT_OPEN_MS = 1500;
    public static long TURRET_TIMEOUT_MS = 1000;
    
    // Turret angle: Red Far: -21.3° (aim left toward red basket at 140, 140)
    public static double TURRET_SHOOT_ANGLE_DEG = -21.3;
    
    @Override
    public Pose getStartPose() {
        return START_POSE;
    }
    
    /**
     * Shoot command - uses TransitCommand (same as manual TeleOp).
     * TransitCommand continuously checks shooter speed:
     * - Opens transit ONLY when shooter is at target velocity
     * - Closes transit if speed drops
     * - Manages firing boost automatically
     */
    private Command shootAfterTurretReady() {
        return new SequentialCommandGroup(
                new WaitForTurretCommand(turret, TURRET_TIMEOUT_MS),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                // Use TransitCommand (same as manual fire) with timeout
                new TransitCommand(transit, shooter)
                        .withTimeout(SHOOTER_SPINUP_TIMEOUT_MS + TRANSIT_OPEN_MS),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new InstantCommand(() -> turret.enableSoftLock(0))
        );
    }
    
    private Command intakeWaitCommand() {
        return new WaitCommand(INTAKE_WAIT_MS);
    }
    
    private Command oneCycleCommand() {
        PathChain toIntake = follower
                .pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, INTAKE_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        PathChain toShoot = follower
                .pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, toIntake),
                intakeWaitCommand(),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, toShoot),
                shootAfterTurretReady()
        );
    }
    
    @Override
    public Command runAutoCommand() {
        path1 = follower
                .pathBuilder()
                .addPath(new BezierCurve(START_POSE, CURVE_CONTROL, CURVE_END))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
        
        path2 = follower
                .pathBuilder()
                .addPath(new BezierLine(CURVE_END, INTAKE_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        path3 = follower
                .pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, path1),
                new AutoDriveCommand(follower, path2),
                intakeWaitCommand(),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path3),
                shootAfterTurretReady(),
                oneCycleCommand(),
                oneCycleCommand(),
                oneCycleCommand(),
                oneCycleCommand(),
                oneCycleCommand(),
                oneCycleCommand(),
                oneCycleCommand(),
                oneCycleCommand(),
                oneCycleCommand(),
                oneCycleCommand()
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
