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
 * Blue Near Infinite - Extended sample collection auto (Blue Alliance, Near side)
 * 
 * New Sequence (6 paths) + BlueNear Path 5+ continuation
 * 
 * Sequence:
 * 1. Path 1: Start → Shoot position
 * 2. Path 2: → Sample 1 (far curve)
 * 3. Path 3: → Intake position 1 (curve)
 * 4. Path 4: → Shoot position [shoot]
 * 5. Path 5: → Sample 2 (angle 145°)
 * 6. Path 7: → Shoot position [shoot]
 * 
 * Then continue with BlueNear Path 5+:
 * 7. → Intake position 2 (curve)
 * 8. → Shoot position [shoot]
 * 9. → Sample 3 (bottom, double curve)
 * 10. → Shoot position [shoot]
 * 11. → Final position
 */
@Config
@Autonomous(name = "Blue Near Infinite", group = "Autos")
public class BlueNearInfinite extends AutoCommandBase {
    
    // Path declarations
    private PathChain path1, path2, path3, path4;
    private PathChain pathBN5, pathBN6, pathBN7, pathBN8, pathBN9; // BlueNear continuation
    
    // Key positions (Blue side)
    private static final Pose START_POSE = new Pose(25.68, 127.97, Math.toRadians(90));
    private static final Pose SHOOT_POSE = new Pose(45.20, 101.15, Math.toRadians(180));
    
    // New path positions
    private static final Pose SAMPLE1_POSE = new Pose(8.79, 59.30, Math.toRadians(180));
    private static final Point SAMPLE1_CTRL = new Point(67.62, 56.74);
    
    private static final Pose INTAKE1_POSE = new Pose(16.45, 69.66, Math.toRadians(180));
    private static final Point INTAKE1_CTRL = new Point(26.89, 61.55);
    
    private static final Pose SAMPLE2_POSE = new Pose(12.08, 61.02, Math.toRadians(145));
    private static final Point SAMPLE2_CTRL = new Point(31.89, 62.83);
    
    // BlueNear continuation positions (Path 5+)
    private static final Pose BN_INTAKE2_POSE = new Pose(16.17, 83.46, Math.toRadians(180));
    private static final Point BN_INTAKE2_CTRL = new Point(46.74, 81.85);
    
    private static final Pose BN_SAMPLE2_POSE = new Pose(14.19, 35.31, Math.toRadians(180));
    private static final Point BN_SAMPLE2_CTRL1 = new Point(63.62, 43.59);
    private static final Point BN_SAMPLE2_CTRL2 = new Point(70.33, 33.14);
    
    private static final Pose BN_FINAL_POSE = new Pose(15.96, 101.11, Math.toRadians(180));
    
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
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
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
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
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
        
        // Path 1: Start → Shoot (heading 90° → 180°)
        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(START_POSE.getX(), START_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
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
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 5 & 6 are built dynamically via sample2CycleCommand()
        
        // === BLUENEAR CONTINUATION (Path 5+) ===
        
        // BN Path 5: Shoot → Intake 2 (curve)
        pathBN5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                BN_INTAKE2_CTRL,
                                new Point(BN_INTAKE2_POSE.getX(), BN_INTAKE2_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // BN Path 6: Intake 2 → Shoot (straight)
        pathBN6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(BN_INTAKE2_POSE.getX(), BN_INTAKE2_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // BN Path 7: Shoot → Sample 3 (double control curve)
        pathBN7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                BN_SAMPLE2_CTRL1,
                                BN_SAMPLE2_CTRL2,
                                new Point(BN_SAMPLE2_POSE.getX(), BN_SAMPLE2_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // BN Path 8: Sample 3 → Shoot (straight)
        pathBN8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(BN_SAMPLE2_POSE.getX(), BN_SAMPLE2_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // BN Path 9: Shoot → Final (straight)
        pathBN9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                new Point(BN_FINAL_POSE.getX(), BN_FINAL_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                
                // === BLUENEAR CONTINUATION ===
                // BN Path 5: Get intake 2
                new AutoDriveCommand(follower, pathBN5),
                intakeWaitCommand(),
                
                // BN Path 6: Back to shoot
                new AutoDriveCommand(follower, pathBN6),
                shootCommand(),
                
                // BN Path 7: Get sample 3 (far bottom)
                new AutoDriveCommand(follower, pathBN7),
                intakeWaitCommand(),
                
                // BN Path 8: Back to shoot
                new AutoDriveCommand(follower, pathBN8),
                shootCommand(),
                
                // BN Path 9: Final position
                new AutoDriveCommand(follower, pathBN9)
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
