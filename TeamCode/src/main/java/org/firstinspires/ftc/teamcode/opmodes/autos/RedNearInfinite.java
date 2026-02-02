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
 * Red Near Infinite - Extended sample collection auto (Red Alliance, Near side)
 * Mirrored from BlueNearInfinite along x=72 axis
 * 
 * New Sequence (6 paths) + RedNear Path 5+ continuation
 * 
 * Sequence:
 * 1. Path 1: Start → Shoot position
 * 2. Path 2: → Sample 1 (far curve)
 * 3. Path 3: → Intake position 1 (curve)
 * 4. Path 4: → Shoot position [shoot]
 * 5. Path 5: → Sample 2 (angle 35°)
 * 6. Path 7: → Shoot position [shoot]
 * 
 * Then continue with RedNear Path 5+:
 * 7. → Intake position 2 (curve)
 * 8. → Shoot position [shoot]
 * 9. → Sample 3 (bottom, double curve)
 * 10. → Shoot position [shoot]
 * 11. → Final position
 */
@Config
@Autonomous(name = "Red Near Infinite", group = "Autos")
public class RedNearInfinite extends AutoCommandBase {
    
    // Path declarations
    private PathChain path1, path2, path3, path4;
    private PathChain pathRN5, pathRN6, pathRN7, pathRN8, pathRN9; // RedNear continuation
    
    // Key positions (Red side - mirrored at x=72)
    // Mirror formula: new_x = 144 - old_x, heading mirror: 180° → 0°, 145° → 35°
    private static final Pose START_POSE = new Pose(118.32, 127.97, Math.toRadians(90));     // 144 - 25.68
    private static final Pose SHOOT_POSE = new Pose(98.80, 101.15, Math.toRadians(0));       // 144 - 45.20
    
    // New path positions (mirrored)
    private static final Pose SAMPLE1_POSE = new Pose(135.21, 59.30, Math.toRadians(0));     // 144 - 8.79
    private static final Point SAMPLE1_CTRL = new Point(76.38, 56.74);                        // 144 - 67.62
    
    private static final Pose INTAKE1_POSE = new Pose(127.55, 69.66, Math.toRadians(0));     // 144 - 16.45
    private static final Point INTAKE1_CTRL = new Point(117.11, 61.55);                       // 144 - 26.89
    
    private static final Pose SAMPLE2_POSE = new Pose(131.92, 61.02, Math.toRadians(35));    // 144 - 12.08, 180-145=35
    private static final Point SAMPLE2_CTRL = new Point(112.11, 62.83);                       // 144 - 31.89
    
    // RedNear continuation positions (Path 5+, mirrored)
    private static final Pose RN_INTAKE2_POSE = new Pose(127.83, 83.46, Math.toRadians(0));  // 144 - 16.17
    private static final Point RN_INTAKE2_CTRL = new Point(97.26, 81.85);                     // 144 - 46.74
    
    private static final Pose RN_SAMPLE2_POSE = new Pose(129.81, 35.31, Math.toRadians(0));  // 144 - 14.19
    private static final Point RN_SAMPLE2_CTRL1 = new Point(80.38, 43.59);                    // 144 - 63.62
    private static final Point RN_SAMPLE2_CTRL2 = new Point(73.67, 33.14);                    // 144 - 70.33
    
    private static final Pose RN_FINAL_POSE = new Pose(128.04, 101.11, Math.toRadians(0));   // 144 - 15.96
    
    // Wait times (ms)
    public static long INTAKE_WAIT_MS = 500;
    public static long SHOOT_WAIT_MS = 1500;
    
    @Override
    public Pose getStartPose() {
        return START_POSE;
    }
    
    private Command shootCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new WaitCommand(SHOOT_WAIT_MS),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
    
    private Command intakeWaitCommand() {
        return new WaitCommand(INTAKE_WAIT_MS);
    }
    
    /**
     * Build path 5: Shoot → Sample 2 (needs to be rebuilt each time)
     */
    private PathChain buildPath5() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                SAMPLE2_CTRL,
                                new Point(SAMPLE2_POSE.getX(), SAMPLE2_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                .build();
    }
    
    /**
     * Build path 6: Sample 2 → Shoot (needs to be rebuilt each time)
     */
    private PathChain buildPath6() {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE2_POSE.getX(), SAMPLE2_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .build();
    }
    
    /**
     * One cycle of path 5-6: Sample 2 collection
     */
    private Command sample2CycleCommand() {
        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, buildPath5()),
                intakeWaitCommand(),
                new AutoDriveCommand(follower, buildPath6()),
                shootCommand()
        );
    }
    
    @Override
    public Command runAutoCommand() {
        // === NEW PATHS (6 paths) ===
        
        // Path 1: Start → Shoot (heading 90° → 0°)
        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(START_POSE.getX(), START_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
        
        // Path 2: Shoot → Sample 1 (curve)
        path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                SAMPLE1_CTRL,
                                new Point(SAMPLE1_POSE.getX(), SAMPLE1_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // Path 3: Sample 1 → Intake 1 (curve)
        path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SAMPLE1_POSE.getX(), SAMPLE1_POSE.getY()),
                                INTAKE1_CTRL,
                                new Point(INTAKE1_POSE.getX(), INTAKE1_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // Path 4: Intake 1 → Shoot (straight)
        path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(INTAKE1_POSE.getX(), INTAKE1_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // Path 5 & 6 are built dynamically via sample2CycleCommand()
        
        // === REDNEAR CONTINUATION (Path 5+) ===
        
        // RN Path 5: Shoot → Intake 2 (curve)
        pathRN5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                RN_INTAKE2_CTRL,
                                new Point(RN_INTAKE2_POSE.getX(), RN_INTAKE2_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // RN Path 6: Intake 2 → Shoot (straight)
        pathRN6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(RN_INTAKE2_POSE.getX(), RN_INTAKE2_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // RN Path 7: Shoot → Sample 3 (double control curve)
        pathRN7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                RN_SAMPLE2_CTRL1,
                                RN_SAMPLE2_CTRL2,
                                new Point(RN_SAMPLE2_POSE.getX(), RN_SAMPLE2_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // RN Path 8: Sample 3 → Shoot (straight)
        pathRN8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(RN_SAMPLE2_POSE.getX(), RN_SAMPLE2_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // RN Path 9: Shoot → Final (straight)
        pathRN9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                new Point(RN_FINAL_POSE.getX(), RN_FINAL_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // Build command sequence
        return new SequentialCommandGroup(
                // === NEW SEQUENCE ===
                // Path 1: Go to shoot position
                new AutoDriveCommand(follower, path1),
                
                // Path 2-3: Get sample 1
                new AutoDriveCommand(follower, path2),
                new AutoDriveCommand(follower, path3),
                intakeWaitCommand(),
                
                // Path 4: Back to shoot
                new AutoDriveCommand(follower, path4),
                shootCommand(),
                
                // Path 5-6: Get sample 2 (angled) - repeat 3 times
                sample2CycleCommand(),  // Cycle 1
                sample2CycleCommand(),  // Cycle 2
                sample2CycleCommand(),  // Cycle 3
                
                // === REDNEAR CONTINUATION ===
                // RN Path 5: Get intake 2
                new AutoDriveCommand(follower, pathRN5),
                intakeWaitCommand(),
                
                // RN Path 6: Back to shoot
                new AutoDriveCommand(follower, pathRN6),
                shootCommand(),
                
                // RN Path 7: Get sample 3 (far bottom)
                new AutoDriveCommand(follower, pathRN7),
                intakeWaitCommand(),
                
                // RN Path 8: Back to shoot
                new AutoDriveCommand(follower, pathRN8),
                shootCommand(),
                
                // RN Path 9: Final position
                new AutoDriveCommand(follower, pathRN9)
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
