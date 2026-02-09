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
 * Blue Near Auto - Sample collection auto (Blue Alliance, Near side)
 * 
 * Turret starts moving during path to shoot position (saves time).
 * Shooting only starts after turret reaches target angle.
 */
@Config
@Autonomous(name = "Blue Near Auto", group = "Autos")
public class BlueNearAuto extends AutoCommandBase {
    
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;
    
    // Key positions (Blue side)
    private static final Pose START_POSE = new Pose(25.68, 127.97, Math.toRadians(143.5));
    private static final Pose SHOOT_POSE = new Pose(59.87, 90.57, Math.toRadians(180));
    
    private static final Pose SAMPLE1_POSE = new Pose(19, 58, Math.toRadians(180));
    private static final Pose SAMPLE1_CTRL = new Pose(67.62, 56.74);
    
    private static final Pose INTAKE1_POSE = new Pose(13, 69.66, Math.toRadians(180));
    private static final Pose INTAKE1_CTRL = new Pose(26.89, 61.55);
    
    private static final Pose INTAKE2_POSE = new Pose(17, 83.46, Math.toRadians(180));
    private static final Pose INTAKE2_CTRL = new Pose(46.74, 81.85);
    
    private static final Pose SAMPLE2_POSE = new Pose(19, 35.31, Math.toRadians(180));
    private static final Pose SAMPLE2_CTRL1 = new Pose(63.62, 43.59);
    private static final Pose SAMPLE2_CTRL2 = new Pose(70.33, 33.14);
    
    private static final Pose FINAL_INTAKE_POSE = new Pose(17.14, 58.25, Math.toRadians(180));
    
    // Wait times (ms)
    public static long SHOOTER_SPINUP_TIMEOUT_MS = 1500;  // Max wait for shooter to reach speed
    public static long TRANSIT_OPEN_MS = 1000;            // Time to keep transit open for ball to exit
    public static long TURRET_TIMEOUT_MS = 1000;
    
    // Blue Near: +44° (aim right toward blue basket at 0, 140)
    public static double TURRET_SHOOT_ANGLE_DEG = 44.3;
    
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
     * 
     * TX auto-aim: If Limelight sees a goal tag (20/24), adjusts turret
     * angle by TX offset for more precise aiming before shooting.
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
                //    TransitCommand handles: speed check, transit open/close, boost
                new TransitCommand(transit, shooter)
                        .withTimeout(SHOOTER_SPINUP_TIMEOUT_MS + TRANSIT_OPEN_MS),
                // 3. Return shooter to idle and turret to forward
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new InstantCommand(() -> turret.enableSoftLock(0))
        );
    }
    
    @Override
    public Command runAutoCommand() {
        // Path 1: Start → Shoot
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), Math.toRadians(180))
                .build();
        
        // Path 2: Shoot → Sample 1
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE1_CTRL, SAMPLE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 3: Sample 1 → Intake 1 (80% power for gentle approach)
        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(SAMPLE1_POSE, INTAKE1_CTRL, INTAKE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 4: Intake 1 → Shoot (curve)
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(INTAKE1_POSE, new Pose(53.40, 64.83), SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 5: Shoot → Intake 2
        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, INTAKE2_CTRL, INTAKE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 6: Intake 2 → Shoot
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 7: Shoot → Sample 2
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE2_CTRL1, SAMPLE2_CTRL2, SAMPLE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 8: Sample 2 → Shoot
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(SAMPLE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 9: Shoot → Final
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, FINAL_INTAKE_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                
                // Path 5: Get intake 2 (1.3s timeout protection)
                new AutoDriveCommand(follower, path5).withTimeout(1300),
                
                // Path 6: Drive to shoot
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path6),
                shootAfterTurretReady(),
                
                // Path 7: Get sample 2 (1.3s timeout protection)
                new AutoDriveCommand(follower, path7).withTimeout(1300),
                
                // Path 8: Drive to shoot
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path8),
                shootAfterTurretReady(),
                
                // Path 9: Final position + STOP shooter
                new AutoDriveCommand(follower, path9),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
