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
 * 1. Start at shoot position, shoot FAST toward red basket
 * 2. Path 1: Push sample (curve)
 * 3. Path 2: Push sample to edge (line)
 * 4. Path 3: Return to start/shoot position (curve)
 * 5. Wait 1 second at start position
 * 6. Path 4: Go to final position (line)
 * 
 * Intake runs at full power throughout (handled by AutoCommandBase).
 */
@Config
@Autonomous(name = "Red Far Auto", group = "Autos")
public class RedFarAuto extends AutoCommandBase {

    private PathChain path1, path2, path3, path4;

    // Start position = Shoot position
    private static final Pose START_POSE = new Pose(88.508, 11.194, Math.toRadians(90));

    // Path endpoints
    private static final Pose PUSH1_POSE = new Pose(121.263, 11.772, Math.toRadians(0));
    private static final Pose PUSH1_CTRL = new Pose(107.488, 20.966);

    private static final Pose PUSH2_POSE = new Pose(134.002, 8.708, Math.toRadians(0));

    // Path 3 returns to START_POSE
    private static final Pose RETURN_CTRL = new Pose(110.288, 25.011);

    // Final parking position
    private static final Pose FINAL_POSE = new Pose(110.288, 13.794, Math.toRadians(90));

    // Wait times (ms)
    public static long SHOOTER_SPINUP_TIMEOUT_MS = 2000;
    public static long TRANSIT_OPEN_MS = 1000;

    // Turret angle for red basket from START_POSE
    // atan2(140 - 11.194, 140 - 88.508) - 90° ≈ -21.8°
    public static double TURRET_SHOOT_ANGLE_DEG = 20;

    @Override
    public Pose getStartPose() {
        return START_POSE;
    }

    @Override
    public Command runAutoCommand() {
        // Path 1: Start → Push1 (curve, heading 90° → 0°)
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(START_POSE, PUSH1_CTRL, PUSH1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        // Path 2: Push1 → Push2 (line, heading 0° → 0°)
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(PUSH1_POSE, PUSH2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 3: Push2 → Start (curve, heading 0° → 90°)
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(PUSH2_POSE, RETURN_CTRL, START_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        // Path 4: Start → Final (line, heading 90° → 90°)
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, FINAL_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        return new SequentialCommandGroup(
                // === SHOOT AT START ===
                // 1. Turn turret to calculated angle toward red basket
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new WaitCommand(300),  // Wait for turret to settle
                // 2. Far shot (FAST)
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new TransitCommand(transit, shooter)
                        .withTimeout(SHOOTER_SPINUP_TIMEOUT_MS + TRANSIT_OPEN_MS),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new InstantCommand(() -> turret.enableSoftLock(0)),

                // === PUSH SAMPLES ===
                // Path 1: Push sample (curve)
                new AutoDriveCommand(follower, path1),

                // Path 2: Push to edge (line, 5s timeout protection)
                new AutoDriveCommand(follower, path2).withTimeout(5000),

                // Path 3: Return to start (curve)
                new AutoDriveCommand(follower, path3),

                // Wait 1 second at start position
                new WaitCommand(1000),

                // Path 4: Go to final position
                new AutoDriveCommand(follower, path4),

                // Stop
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
