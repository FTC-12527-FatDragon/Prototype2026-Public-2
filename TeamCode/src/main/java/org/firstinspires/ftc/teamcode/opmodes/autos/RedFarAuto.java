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
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Red Far Auto - Continuous intake/shoot cycles (Red Alliance, Far side)
 * Mirrored from BlueFarAuto along x=72 axis
 * 
 * Sequence:
 * 1. Path 1: Curve to intake approach
 * 2. Path 2: Drive to intake position
 * 3. Wait 1s for intake
 * 4. Path 3: Drive to shoot position
 * 5. Wait 3s for shoot
 * 
 * LOOP (until auto ends):
 * 6. Path 4: Drive to intake position
 * 7. Wait 1s for intake
 * 8. Path 5: Drive to shoot position
 * (repeat 6-7-8)
 */
@Config
@Autonomous(name = "Red Far Auto", group = "Autos")
public class RedFarAuto extends AutoCommandBase {
    
    // Path declarations (initial approach only, cycles are built dynamically)
    private PathChain path1;  // Start → Curve to intake approach
    private PathChain path2;  // → Intake position
    private PathChain path3;  // Intake → Shoot position
    
    // Key positions (Red side - right side of field, mirrored from Blue at x=72)
    // Mirror formula: new_x = 144 - old_x, heading 180° → 0°
    private static final Pose START_POSE = new Pose(89.03, 9.10, Math.toRadians(90));      // 144 - 54.97
    private static final Pose CURVE_END = new Pose(120.25, 7.30, Math.toRadians(0));       // 144 - 23.75, 180° → 0°
    private static final Point CURVE_CONTROL = new Point(104.14, 14.95);                   // 144 - 39.86
    private static final Pose INTAKE_POSE = new Pose(134.40, 8.13, Math.toRadians(0));     // 144 - 9.60, 180° → 0°
    private static final Pose SHOOT_POSE = new Pose(89.03, 9.10, Math.toRadians(0));       // 144 - 54.97, 180° → 0°
    
    // Wait times (ms)
    public static long INTAKE_WAIT_MS = 1000;
    public static long SHOOT_WAIT_MS = 3000;
    
    @Override
    public Pose getStartPose() {
        return START_POSE;
    }
    
    /**
     * Shoot command sequence - starts shooter, waits, then stops
     */
    private Command shootCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new WaitCommand(SHOOT_WAIT_MS),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
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
     * Path 4 (to intake) → Wait → Path 5 (to shoot)
     */
    private Command oneCycleCommand() {
        // Need to rebuild paths each time since PathChain can only be used once
        PathChain toIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                new Point(INTAKE_POSE.getX(), INTAKE_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        PathChain toShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(INTAKE_POSE.getX(), INTAKE_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, toIntake),
                intakeWaitCommand(),
                new AutoDriveCommand(follower, toShoot)
        );
    }
    
    @Override
    public Command runAutoCommand() {
        // Path 1: Start → Curve approach (heading 90° → 0°)
        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(START_POSE.getX(), START_POSE.getY()),
                                CURVE_CONTROL,
                                new Point(CURVE_END.getX(), CURVE_END.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
        
        // Path 2: Curve end → Intake position (heading 0° → 0°)
        path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(CURVE_END.getX(), CURVE_END.getY()),
                                new Point(INTAKE_POSE.getX(), INTAKE_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // Path 3: Intake → Shoot position (heading 0° → 0°)
        path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(INTAKE_POSE.getX(), INTAKE_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // Note: Path 4 and 5 (cycle paths) are built dynamically in oneCycleCommand()
        
        // Build command sequence
        return new SequentialCommandGroup(
                // === INITIAL APPROACH ===
                // Path 1: Curve to intake approach
                new AutoDriveCommand(follower, path1),
                // Path 2: Drive to intake position
                new AutoDriveCommand(follower, path2),
                
                // Wait for intake
                intakeWaitCommand(),
                
                // Path 3: Drive to shoot position
                new AutoDriveCommand(follower, path3),
                
                // Shoot
                shootCommand(),
                
                // === CONTINUOUS CYCLES (6-7-8 repeating) ===
                // Each cycle: Intake → Wait → Shoot
                oneCycleCommand(),  // Cycle 2
                oneCycleCommand(),  // Cycle 3
                oneCycleCommand(),  // Cycle 4
                oneCycleCommand(),  // Cycle 5
                oneCycleCommand(),  // Cycle 6
                oneCycleCommand(),  // Cycle 7
                oneCycleCommand(),  // Cycle 8
                oneCycleCommand(),  // Cycle 9
                oneCycleCommand(),  // Cycle 10
                oneCycleCommand()   // Cycle 11 (auto will end before completing all)
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
