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
 * Blue Far Auto - Continuous intake/shoot cycles (Blue Alliance, Far side)
 * 
 * Turret starts moving during path to shoot position (saves time).
 * Shooting only starts after turret reaches target angle.
 */
@Config
@Autonomous(name = "Blue Far Auto", group = "Autos")
public class BlueFarAuto extends AutoCommandBase {
    
    // Path declarations (initial approach only, cycles are built dynamically)
    private PathChain path1;  // Start → Curve to intake approach
    private PathChain path2;  // → Intake position
    private PathChain path3;  // Intake → Shoot position
    
    // Key positions (Blue side - left side of field)
    private static final Pose START_POSE = new Pose(54.97, 9.10, Math.toRadians(90));
    private static final Pose CURVE_END = new Pose(23.75, 7.30, Math.toRadians(180));
    private static final Pose CURVE_CONTROL = new Pose(39.86, 14.95);
    private static final Pose INTAKE_POSE = new Pose(9.60, 8.13, Math.toRadians(180));
    private static final Pose SHOOT_POSE = new Pose(54.97, 9.10, Math.toRadians(180));
    
    // Wait times (ms)
    public static long INTAKE_WAIT_MS = 1000;
    public static long SHOOTER_SPINUP_TIMEOUT_MS = 2000;
    public static long TRANSIT_OPEN_MS = 1500;
    public static long TURRET_TIMEOUT_MS = 1000;
    
    // Turret angle when at SHOOT_POSE (relative to robot heading)
    // Positive = clockwise from robot front (right)
    // Blue Far: +21.3° (aim right toward blue basket at 4, 140)
    public static double TURRET_SHOOT_ANGLE_DEG = 21.3;
    
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
    
    /**
     * Intake command - just wait (intake runs continuously)
     */
    private Command intakeWaitCommand() {
        return new WaitCommand(INTAKE_WAIT_MS);
    }
    
    /**
     * One cycle: Intake → Shoot
     * Turret starts moving during path to shoot position.
     */
    private Command oneCycleCommand() {
        // Need to rebuild paths each time since PathChain can only be used once
        PathChain toIntake = follower
                .pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, INTAKE_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        PathChain toShoot = follower
                .pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        return new SequentialCommandGroup(
                // Go to intake
                new AutoDriveCommand(follower, toIntake),
                intakeWaitCommand(),
                
                // Start turret moving BEFORE path to shoot
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                // Drive to shoot (turret moving simultaneously)
                new AutoDriveCommand(follower, toShoot),
                // Wait for turret + shoot
                shootAfterTurretReady()
        );
    }
    
    @Override
    public Command runAutoCommand() {
        // Path 1: Start → Curve approach (heading 90° → 180°)
        path1 = follower
                .pathBuilder()
                .addPath(new BezierCurve(START_POSE, CURVE_CONTROL, CURVE_END))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        
        // Path 2: Curve end → Intake position (heading 180° → 180°)
        path2 = follower
                .pathBuilder()
                .addPath(new BezierLine(CURVE_END, INTAKE_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 3: Intake → Shoot position (heading 180° → 180°)
        path3 = follower
                .pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Build command sequence
        return new SequentialCommandGroup(
                // === INITIAL APPROACH ===
                // Path 1: Curve to intake approach
                new AutoDriveCommand(follower, path1),
                // Path 2: Drive to intake position
                new AutoDriveCommand(follower, path2),
                
                // Wait for intake
                intakeWaitCommand(),
                
                // Start turret moving BEFORE path to shoot
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                // Path 3: Drive to shoot position (turret moving simultaneously)
                new AutoDriveCommand(follower, path3),
                // Wait for turret + shoot
                shootAfterTurretReady(),
                
                // === CONTINUOUS CYCLES ===
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
