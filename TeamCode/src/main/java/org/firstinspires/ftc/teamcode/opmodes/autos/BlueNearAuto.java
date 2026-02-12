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
 * Blue Near Auto - Sample collection auto (Blue Alliance, Near side)
 *
 * Turret pre-positions during drive paths to overlap movement time.
 * Shooting uses base class createShootSequence() with TX auto-aim.
 */
@Config
@Autonomous(name = "Blue Near Auto", group = "Autos")
public class BlueNearAuto extends AutoCommandBase {

    // ── Poses ──
    private static final Pose START_POSE       = new Pose(25.68, 127.97, Math.toRadians(143.5));
    private static final Pose SHOOT_POSE       = new Pose(59.87, 90.57, Math.toRadians(180));
    private static final Pose SAMPLE1_POSE     = new Pose(19, 58, Math.toRadians(180));
    private static final Pose SAMPLE1_CTRL     = new Pose(67.62, 56.74);
    private static final Pose INTAKE1_POSE     = new Pose(13, 69.66, Math.toRadians(180));
    private static final Pose INTAKE1_CTRL     = new Pose(26.89, 61.55);
    private static final Pose INTAKE2_POSE     = new Pose(17, 83.46, Math.toRadians(180));
    private static final Pose INTAKE2_CTRL     = new Pose(46.74, 81.85);
    private static final Pose SAMPLE2_POSE     = new Pose(19, 35.31, Math.toRadians(180));
    private static final Pose SAMPLE2_CTRL1    = new Pose(63.62, 43.59);
    private static final Pose SAMPLE2_CTRL2    = new Pose(70.33, 33.14);
    private static final Pose FINAL_INTAKE_POSE = new Pose(17.14, 58.25, Math.toRadians(180));

    // Blue Near: +44.3° (aim right toward blue basket at 0, 140)
    public static double TURRET_SHOOT_ANGLE_DEG = 44.3;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

    @Override
    public Pose getStartPose() {
        return START_POSE;
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
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, INTAKE2_CTRL, INTAKE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE2_CTRL1, SAMPLE2_CTRL2, SAMPLE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(SAMPLE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, FINAL_INTAKE_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        return new SequentialCommandGroup(
                // Shot 1: drive to shoot, fire
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

                // Cycle 2: intake 2 → shoot
                new AutoDriveCommand(follower, path5).withTimeout(1300),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path6),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID),

                // Cycle 3: sample 2 → shoot
                new AutoDriveCommand(follower, path7).withTimeout(1300),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path8),
                createShootSequence(TURRET_SHOOT_ANGLE_DEG, Shooter.ShooterState.MID),

                // Final: park
                new AutoDriveCommand(follower, path9),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}
