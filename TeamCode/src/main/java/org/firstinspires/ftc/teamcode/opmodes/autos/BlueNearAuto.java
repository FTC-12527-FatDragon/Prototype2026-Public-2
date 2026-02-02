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
 * Blue Near Auto - Sample collection auto (Blue Alliance, Near side)
 * 
 * Sequence:
 * 1. Path 1: Start → Shoot position
 * 2. Path 2: → Sample 1 (far)
 * 3. Path 3: → Intake position 1
 * 4. Path 4: → Shoot position
 * 5. Path 5: → Intake position 2
 * 6. Path 6: → Shoot position
 * 7. Path 7: → Sample 2 (bottom)
 * 8. Path 8: → Shoot position
 * 9. Path 9: → Final intake position
 */
@Config
@Autonomous(name = "Blue Near Auto", group = "Autos")
public class BlueNearAuto extends AutoCommandBase {
    
    // Path declarations
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;
    
    // Key positions (Blue side - left side of field)
    private static final Pose START_POSE = new Pose(25.68, 127.97, Math.toRadians(90));
    private static final Pose SHOOT_POSE = new Pose(45.20, 101.15, Math.toRadians(180));
    
    // Sample/Intake positions
    private static final Pose SAMPLE1_POSE = new Pose(8.79, 59.30, Math.toRadians(180));
    private static final Point SAMPLE1_CTRL = new Point(67.62, 56.74);
    
    private static final Pose INTAKE1_POSE = new Pose(16.45, 69.66, Math.toRadians(180));
    private static final Point INTAKE1_CTRL = new Point(26.89, 61.55);
    
    private static final Pose INTAKE2_POSE = new Pose(16.17, 83.46, Math.toRadians(180));
    private static final Point INTAKE2_CTRL = new Point(46.74, 81.85);
    
    private static final Pose SAMPLE2_POSE = new Pose(14.19, 35.31, Math.toRadians(180));
    private static final Point SAMPLE2_CTRL1 = new Point(63.62, 43.59);
    private static final Point SAMPLE2_CTRL2 = new Point(70.33, 33.14);
    
    private static final Pose FINAL_INTAKE_POSE = new Pose(15.96, 101.11, Math.toRadians(180));
    
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
    
    @Override
    public Command runAutoCommand() {
        // Path 1: Start → Shoot position (heading 90° → 180° via 143.5°)
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
        
        // Path 2: Shoot → Sample 1 (curve with control point)
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
        
        // Path 4: Intake 1 → Shoot (straight line)
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
        
        // Path 5: Shoot → Intake 2 (curve)
        path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                INTAKE2_CTRL,
                                new Point(INTAKE2_POSE.getX(), INTAKE2_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 6: Intake 2 → Shoot (straight line)
        path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(INTAKE2_POSE.getX(), INTAKE2_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 7: Shoot → Sample 2 (double control point curve)
        path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                SAMPLE2_CTRL1,
                                SAMPLE2_CTRL2,
                                new Point(SAMPLE2_POSE.getX(), SAMPLE2_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 8: Sample 2 → Shoot (straight line)
        path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SAMPLE2_POSE.getX(), SAMPLE2_POSE.getY()),
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 9: Shoot → Final intake (straight line)
        path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(SHOOT_POSE.getX(), SHOOT_POSE.getY()),
                                new Point(FINAL_INTAKE_POSE.getX(), FINAL_INTAKE_POSE.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Build command sequence
        return new SequentialCommandGroup(
                // Path 1: Go to shoot position
                new AutoDriveCommand(follower, path1),
                
                // Path 2-3: Get sample 1
                new AutoDriveCommand(follower, path2),
                new AutoDriveCommand(follower, path3),
                intakeWaitCommand(),
                
                // Path 4: Back to shoot
                new AutoDriveCommand(follower, path4),
                shootCommand(),
                
                // Path 5: Get sample from position 2
                new AutoDriveCommand(follower, path5),
                intakeWaitCommand(),
                
                // Path 6: Back to shoot
                new AutoDriveCommand(follower, path6),
                shootCommand(),
                
                // Path 7: Get sample 2 (far bottom)
                new AutoDriveCommand(follower, path7),
                intakeWaitCommand(),
                
                // Path 8: Back to shoot
                new AutoDriveCommand(follower, path8),
                shootCommand(),
                
                // Path 9: Final position
                new AutoDriveCommand(follower, path9)
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
