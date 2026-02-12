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
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Red Far Auto - Far side autonomous (Red Alliance)
 *
 * Sequence:
 * 1. Start at shoot position → FAST shot toward red basket (NO TX auto-aim)
 * 2. Push samples via curve + line
 * 3. Return to start, wait, then park
 *
 * Intake runs at full power throughout (handled by AutoCommandBase).
 *
 * NOTE: This auto uses its own shooting logic (not createShootSequence) because:
 *   - FAST shot needs longer spinup timeout (2000ms vs 1500ms)
 *   - Turret needs longer settle time (300ms vs 200ms)
 *   - TX auto-aim is intentionally disabled for far shots
 */
@Config
@Autonomous(name = "Red Far Auto", group = "Autos")
public class RedFarAuto extends AutoCommandBase {

    // ── Poses ──
    private static final Pose START_POSE  = new Pose(88.508, 11.194, Math.toRadians(90));
    private static final Pose PUSH1_POSE  = new Pose(121.263, 11.772, Math.toRadians(0));
    private static final Pose PUSH1_CTRL  = new Pose(107.488, 20.966);
    private static final Pose PUSH2_POSE  = new Pose(134.002, 8.708, Math.toRadians(0));
    private static final Pose RETURN_CTRL = new Pose(110.288, 25.011);
    private static final Pose FINAL_POSE  = new Pose(110.288, 13.794, Math.toRadians(90));

    // Turret angle for red basket from START_POSE
    public static double TURRET_SHOOT_ANGLE_DEG = 20;

    // Far-shot specific timeouts (longer than Near-shot defaults)
    private static final long FAR_SHOOTER_SPINUP_TIMEOUT_MS = 2000;
    private static final long FAR_TRANSIT_OPEN_MS = 1000;
    private static final long FAR_TURRET_SETTLE_MS = 300;

    private PathChain path1, path2, path3, path4;

    @Override
    public Pose getStartPose() {
        return START_POSE;
    }

    @Override
    public Command runAutoCommand() {
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(START_POSE, PUSH1_CTRL, PUSH1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(PUSH1_POSE, PUSH2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(PUSH2_POSE, RETURN_CTRL, START_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, FINAL_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        return new SequentialCommandGroup(
                // === SHOOT AT START (custom far-shot sequence, NO TX auto-aim) ===
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new WaitCommand(FAR_TURRET_SETTLE_MS),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new TransitCommand(transit, shooter)
                        .withTimeout(FAR_SHOOTER_SPINUP_TIMEOUT_MS + FAR_TRANSIT_OPEN_MS),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new InstantCommand(() -> turret.enableSoftLock(0)),

                // === PUSH SAMPLES ===
                new AutoDriveCommand(follower, path1),
                new AutoDriveCommand(follower, path2).withTimeout(5000),
                new AutoDriveCommand(follower, path3),
                new WaitCommand(1000),

                // === PARK ===
                new AutoDriveCommand(follower, path4),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
