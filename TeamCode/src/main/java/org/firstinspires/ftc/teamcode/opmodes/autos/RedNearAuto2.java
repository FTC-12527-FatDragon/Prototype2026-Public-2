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
 * Red Near Auto 2 - Modified sample collection auto (Red Alliance, Near side)
 * Mirrored from BlueNearAuto2 along x=72 axis.
 *
 * New Sequence (6 paths) + RedNear continuation (paths 6–9).
 */
@Config
@Autonomous(name = "Red Near Auto 2", group = "Autos")
public class RedNearAuto2 extends AutoCommandBase {

    // ── Poses (mirrored: x' = 144 - x, heading 180° → 0°) ──
    private static final Pose START_POSE      = new Pose(118.32, 127.97, Math.toRadians(36.5));
    private static final Pose SHOOT_POSE      = new Pose(84.13, 90.57, Math.toRadians(0));
    private static final Pose SAMPLE1_POSE    = new Pose(125, 58, Math.toRadians(0));
    private static final Pose SAMPLE1_CTRL    = new Pose(76.38, 56.74);
    private static final Pose INTAKE1_POSE    = new Pose(131, 69.66, Math.toRadians(0));
    private static final Pose INTAKE1_CTRL    = new Pose(117.11, 61.55);
    private static final Pose INTAKE2_POSE    = new Pose(127, 83.46, Math.toRadians(0));
    private static final Pose INTAKE2_CTRL    = new Pose(97.26, 81.85);
    private static final Pose NEW_POS         = new Pose(131, 68.97, Math.toRadians(0));
    private static final Pose NEW_POS_CTRL    = new Pose(111.15, 70.64);
    private static final Pose RN_SAMPLE2_POSE = new Pose(125, 35.31, Math.toRadians(0));
    private static final Pose RN_SAMPLE2_CTRL1 = new Pose(80.38, 43.59);
    private static final Pose RN_SAMPLE2_CTRL2 = new Pose(73.67, 33.14);
    private static final Pose RN_FINAL_POSE   = new Pose(126.86, 58.25, Math.toRadians(0));

    // Red: -44.3° (aim left toward red basket at 144, 140)
    public static double TURRET_SHOOT_ANGLE_DEG = -44.3;

    private PathChain path1, path2, path3, path4, path5, path6;
    private PathChain pathRN6, pathRN7, pathRN8, pathRN9;

    @Override
    public Pose getStartPose() {
        return START_POSE;
    }

    @Override
    public Command runAutoCommand() {
        // === New paths (1-6) ===
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
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, INTAKE2_CTRL, INTAKE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(INTAKE2_POSE, NEW_POS_CTRL, NEW_POS))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // === RedNear continuation (RN 6-9) ===
        pathRN6 = follower.pathBuilder()
                .addPath(new BezierLine(NEW_POS, SHOOT_POSE))
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
                new AutoDriveCommand(follower, path4),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID),

                // Intake 2 → new position
                new AutoDriveCommand(follower, path5).withTimeout(1300),
                new AutoDriveCommand(follower, path6).setMaxPower(0.8).withTimeout(1300),
                new WaitCommand(700),

                // RedNear continuation: return → shoot
                new AutoDriveCommand(follower, pathRN6),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID),

                // Sample 2 → shoot
                new AutoDriveCommand(follower, pathRN7).withTimeout(1300),
                new AutoDriveCommand(follower, pathRN8),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID),

                // Final: park
                new AutoDriveCommand(follower, pathRN9),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
