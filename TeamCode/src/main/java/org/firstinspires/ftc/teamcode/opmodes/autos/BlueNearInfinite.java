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
 * Blue Near Infinite - Extended sample collection auto (Blue Alliance, Near side)
 * Includes 3× sample-2 cycles before the BlueNear continuation paths.
 */
@Config
@Autonomous(name = "Blue Near Infinite", group = "Autos")
public class BlueNearInfinite extends AutoCommandBase {

    // ── Poses ──
    private static final Pose START_POSE       = new Pose(25.68, 127.97, Math.toRadians(143.5));
    private static final Pose SHOOT_POSE       = new Pose(59.87, 90.57, Math.toRadians(180));
    private static final Pose SAMPLE1_POSE     = new Pose(19, 58, Math.toRadians(180));
    private static final Pose SAMPLE1_CTRL     = new Pose(67.62, 56.74);
    private static final Pose INTAKE1_POSE     = new Pose(13, 69.66, Math.toRadians(180));
    private static final Pose INTAKE1_CTRL     = new Pose(26.89, 61.55);
    private static final Pose SAMPLE2_POSE     = new Pose(18, 61.02, Math.toRadians(145));
    private static final Pose SAMPLE2_CTRL     = new Pose(31.89, 62.83);
    private static final Pose BN_INTAKE2_POSE  = new Pose(18, 83.46, Math.toRadians(180));
    private static final Pose BN_INTAKE2_CTRL  = new Pose(46.74, 81.85);
    private static final Pose BN_SAMPLE2_POSE  = new Pose(19, 35.31, Math.toRadians(180));
    private static final Pose BN_SAMPLE2_CTRL1 = new Pose(63.62, 43.59);
    private static final Pose BN_SAMPLE2_CTRL2 = new Pose(70.33, 33.14);
    private static final Pose BN_FINAL_POSE    = new Pose(17.14, 58.25, Math.toRadians(180));

    // Blue: +44.3° (aim right toward blue basket at 0, 140)
    public static double TURRET_SHOOT_ANGLE_DEG = 44.3;

    private PathChain path1, path2, path3, path4;
    private PathChain pathBN5, pathBN6, pathBN7, pathBN8, pathBN9;

    @Override
    public Pose getStartPose() {
        return START_POSE;
    }

    /** Builds a fresh sample-2 pickup path (needs rebuild each cycle for Pedro Pathing). */
    private PathChain buildSample2PickupPath() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE2_CTRL, SAMPLE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                .build();
    }

    /** Builds a fresh sample-2 return path. */
    private PathChain buildSample2ReturnPath() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SAMPLE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();
    }

    /** One sample-2 cycle: pickup → return → shoot. */
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
                .setLinearHeadingInterpolation(START_POSE.getHeading(), Math.toRadians(180))
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
                .addPath(new BezierCurve(INTAKE1_POSE, new Pose(53.40, 64.83), SHOOT_POSE))
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

                // BlueNear continuation: intake 2 → shoot
                new AutoDriveCommand(follower, pathBN5).withTimeout(1300),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, pathBN6),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID),

                // BlueNear continuation: sample 2 → shoot
                new AutoDriveCommand(follower, pathBN7).withTimeout(1300),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, pathBN8),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID),

                // Final: park
                new AutoDriveCommand(follower, pathBN9),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}
