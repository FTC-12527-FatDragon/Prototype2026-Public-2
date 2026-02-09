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
 * Red Near Infinite - Extended sample collection auto (Red Alliance, Near side)
 * Mirrored from BlueNearInfinite along x=72 axis
 * 
 * Turret starts moving during path to shoot position (saves time).
 * Shooting only starts after turret reaches target angle.
 */
@Config
@Autonomous(name = "Red Near Infinite", group = "Autos")
public class RedNearInfinite extends AutoCommandBase {
    
    private PathChain path1, path2, path3, path4;
    private PathChain pathRN5, pathRN6, pathRN7, pathRN8, pathRN9;
    
    // Key positions (Red side - mirrored at x=72)
    private static final Pose START_POSE = new Pose(118.32, 127.97, Math.toRadians(36.5));
    private static final Pose SHOOT_POSE = new Pose(84.13, 90.57, Math.toRadians(0));
    
    private static final Pose SAMPLE1_POSE = new Pose(125, 58, Math.toRadians(0));
    private static final Pose SAMPLE1_CTRL = new Pose(76.38, 56.74);
    
    private static final Pose INTAKE1_POSE = new Pose(131, 69.66, Math.toRadians(0));
    private static final Pose INTAKE1_CTRL = new Pose(117.11, 61.55);
    
    private static final Pose SAMPLE2_POSE = new Pose(126, 61.02, Math.toRadians(35));
    private static final Pose SAMPLE2_CTRL = new Pose(112.11, 62.83);
    
    private static final Pose RN_INTAKE2_POSE = new Pose(126, 83.46, Math.toRadians(0));
    private static final Pose RN_INTAKE2_CTRL = new Pose(97.26, 81.85);
    
    private static final Pose RN_SAMPLE2_POSE = new Pose(125, 35.31, Math.toRadians(0));
    private static final Pose RN_SAMPLE2_CTRL1 = new Pose(80.38, 43.59);
    private static final Pose RN_SAMPLE2_CTRL2 = new Pose(73.67, 33.14);
    
    private static final Pose RN_FINAL_POSE = new Pose(126.86, 58.25, Math.toRadians(0));
    
    // Wait times (ms)
    public static long SHOOTER_SPINUP_TIMEOUT_MS = 1500;
    public static long TRANSIT_OPEN_MS = 1000;
    public static long TURRET_TIMEOUT_MS = 1000;
    
    // Red: -44° (aim left toward red basket at 144, 140)
    public static double TURRET_SHOOT_ANGLE_DEG = -44.3;
    
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
                // 0. TX auto-aim: if Limelight sees a goal tag, fine-tune turret angle
                new InstantCommand(() -> {
                    if (vision.hasTarget()) {
                        double tx = vision.getTx();
                        double correctedAngle = TURRET_SHOOT_ANGLE_DEG + tx;
                        turret.enableSoftLock(correctedAngle);
                    }
                }),
                new WaitCommand(200),  // Wait for turret to settle on corrected angle
                // 1. Accelerate shooter from idle to MID
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.MID)),
                // 2. Use TransitCommand (same as manual fire) with timeout
                new TransitCommand(transit, shooter)
                        .withTimeout(SHOOTER_SPINUP_TIMEOUT_MS + TRANSIT_OPEN_MS),
                // 3. Return shooter to idle and turret to forward
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new InstantCommand(() -> turret.enableSoftLock(0))
        );
    }
    
    private PathChain buildPath5() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE2_CTRL, SAMPLE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                .build();
    }
    
    private PathChain buildPath6() {
        return follower.pathBuilder()
                .addPath(new BezierLine(SAMPLE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .build();
    }
    
    private Command sample2CycleCommand() {
        return new SequentialCommandGroup(
                new AutoDriveCommand(follower, buildPath5()),
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, buildPath6()),
                shootAfterTurretReady()
        );
    }
    
    @Override
    public Command runAutoCommand() {
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
        
        // Path 4: Intake 1 → Shoot (curve, mirrored control point)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(INTAKE1_POSE, new Pose(90.60, 64.83), SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        pathRN5 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, RN_INTAKE2_CTRL, RN_INTAKE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        pathRN6 = follower.pathBuilder()
                .addPath(new BezierLine(RN_INTAKE2_POSE, SHOOT_POSE))
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
                // Path 1: Go to shoot position first (shooter not running yet)
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path1),
                // Shoot (shootAfterTurretReady handles SLOW→fire→STOP)
                shootAfterTurretReady(),
                
                // Path 2-3: Get sample 1
                new AutoDriveCommand(follower, path2),
                new AutoDriveCommand(follower, path3).setMaxPower(0.8).withTimeout(1300),  // 80% power, 1.3s timeout
                new WaitCommand(700),  // Wait at INTAKE1_POSE for intake
                
                // Path 4: Drive to shoot
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path4),
                shootAfterTurretReady(),
                
                // Sample 2 cycles
                sample2CycleCommand(),
                sample2CycleCommand(),
                sample2CycleCommand(),
                
                new AutoDriveCommand(follower, pathRN5).withTimeout(1300),  // 1.3s timeout
                
                // PathRN6: Drive to shoot
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, pathRN6),
                shootAfterTurretReady(),
                
                new AutoDriveCommand(follower, pathRN7).withTimeout(1300),  // 1.3s timeout
                
                // PathRN8: Drive to shoot
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, pathRN8),
                shootAfterTurretReady(),
                
                // Final position + STOP shooter
                new AutoDriveCommand(follower, pathRN9),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
