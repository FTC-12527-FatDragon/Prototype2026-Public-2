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
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Red Near Infinite - Extended sample collection auto (Red Alliance, Near side)
 * Mirrored from BlueNearInfinite along x=72 axis.
 */
@Config
@Autonomous(name = "Red Near Infinite", group = "Autos")
public class RedNearInfinite extends AutoCommandBase {

    // ── Poses (mirrored) ──
    private static final Pose START_POSE       = new Pose(118.32, 127.97, Math.toRadians(36.5));
    private static final Pose SHOOT_POSE       = new Pose(84.13, 90.57, Math.toRadians(0));
    private static final Pose SAMPLE1_POSE     = new Pose(125, 58, Math.toRadians(0));
    private static final Pose SAMPLE1_CTRL     = new Pose(76.38, 56.74);
    private static final Pose INTAKE1_POSE     = new Pose(131, 69.66, Math.toRadians(0));
    private static final Pose INTAKE1_CTRL     = new Pose(117.11, 61.55);
    private static final Pose SAMPLE2_POSE     = new Pose(126, 61.02, Math.toRadians(35));
    private static final Pose SAMPLE2_CTRL     = new Pose(112.11, 62.83);
    private static final Pose RN_INTAKE2_POSE  = new Pose(126, 83.46, Math.toRadians(0));
    private static final Pose RN_INTAKE2_CTRL  = new Pose(97.26, 81.85);
    private static final Pose RN_SAMPLE2_POSE  = new Pose(125, 35.31, Math.toRadians(0));
    private static final Pose RN_SAMPLE2_CTRL1 = new Pose(80.38, 43.59);
    private static final Pose RN_SAMPLE2_CTRL2 = new Pose(73.67, 33.14);
    private static final Pose RN_FINAL_POSE    = new Pose(126.86, 58.25, Math.toRadians(0));

    // Red: -44.3° (aim left toward red basket at 144, 140)
    public static double TURRET_SHOOT_ANGLE_DEG = -44.3;

    private PathChain path1, path2, path3, path4;
    private PathChain pathRN5, pathRN6, pathRN7, pathRN8, pathRN9;

    @Override
    public Pose getStartPose() {
        return START_POSE;
    }

    private PathChain buildSample2PickupPath() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE2_CTRL, SAMPLE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                .build();
    }

    private PathChain buildSample2ReturnPath() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SAMPLE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .build();
    }

    private Command sample2CycleCommand() {
        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, buildSample2PickupPath()),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, buildSample2ReturnPath()),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID)
        );
    }

    @Override
    public Command runAutoCommand() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), Math.toRadians(0))
                .build();
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE1_CTRL, SAMPLE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(SAMPLE1_POSE, INTAKE1_CTRL, INTAKE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(INTAKE1_POSE, new Pose(90.60, 64.83), SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        pathRN5 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, RN_INTAKE2_CTRL, RN_INTAKE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        pathRN6 = follower.pathBuilder()
                .addPath(new BezierLine(RN_INTAKE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        pathRN7 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, RN_SAMPLE2_CTRL1, RN_SAMPLE2_CTRL2, RN_SAMPLE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        pathRN8 = follower.pathBuilder()
                .addPath(new BezierLine(RN_SAMPLE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        pathRN9 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, RN_FINAL_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        return new SequentialCommandGroup(
                // Shot 1
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path1),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID),

                // Cycle 1: sample 1 → intake 1 → shoot
                new AutoDriveCommand(follower, path2),
                new AutoDriveCommand(follower, path3).setMaxPower(0.8).withTimeout(1300),
                new WaitCommand(700),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path4),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID),

                // 3× sample-2 cycles
                sample2CycleCommand(),
                sample2CycleCommand(),
                sample2CycleCommand(),

                // RedNear continuation: intake 2 → shoot
                new AutoDriveCommand(follower, pathRN5).withTimeout(1300),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, pathRN6),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID),

                // RedNear continuation: sample 2 → shoot
                new AutoDriveCommand(follower, pathRN7).withTimeout(1300),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, pathRN8),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID),

                // Final: park
                new AutoDriveCommand(follower, pathRN9),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}
