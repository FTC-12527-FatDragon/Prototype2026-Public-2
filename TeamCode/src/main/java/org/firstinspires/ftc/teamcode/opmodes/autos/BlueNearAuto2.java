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
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/**
 * Blue Near Auto 2 - Modified sample collection auto (Blue Alliance, Near side)
 * Mirrored from RedNearAuto2 along x=72 axis
 * 
 * New Sequence (6 paths) + BlueNear Path 6+ continuation
 */
@Config
@Autonomous(name = "Blue Near Auto 2", group = "Autos")
public class BlueNearAuto2 extends AutoCommandBase {
    
    // Path declarations
    private PathChain path1, path2, path3, path4, path5, path6;
    private PathChain pathBN6, pathBN7, pathBN8, pathBN9; // BlueNear continuation
    
    // Key positions (Blue side - mirrored from Red at x=72)
    // Mirror formula: new_x = 144 - old_x, heading 0° → 180°
    private static final Pose START_POSE = new Pose(25.68, 127.97, Math.toRadians(143.5));
    private static final Pose SHOOT_POSE = new Pose(59.87, 90.57, Math.toRadians(180));
    
    // New path positions (mirrored)
    private static final Pose SAMPLE1_POSE = new Pose(19, 58, Math.toRadians(180));
    private static final Pose SAMPLE1_CTRL = new Pose(67.62, 56.74);
    
    private static final Pose INTAKE1_POSE = new Pose(18.3, 69.66, Math.toRadians(180));
    private static final Pose INTAKE1_CTRL = new Pose(26.89, 61.55);
    
    private static final Pose INTAKE2_POSE = new Pose(17, 83.46, Math.toRadians(180));
    private static final Pose INTAKE2_CTRL = new Pose(46.74, 81.85);
    
    private static final Pose NEW_POS = new Pose(18.3, 69.66, Math.toRadians(180));
    private static final Pose NEW_POS_CTRL = new Pose(32.85, 70.64);
    
    // BlueNear continuation positions (mirrored)
    private static final Pose BN_SAMPLE2_POSE = new Pose(17, 35.31, Math.toRadians(180));
    private static final Pose BN_SAMPLE2_CTRL1 = new Pose(63.62, 43.59);
    private static final Pose BN_SAMPLE2_CTRL2 = new Pose(70.33, 33.14);
    
    private static final Pose BN_FINAL_POSE = new Pose(17.14, 58.25, Math.toRadians(180));
    
    // Wait times (ms)
    public static long SHOOTER_SPINUP_TIMEOUT_MS = 1500;
    public static long TRANSIT_OPEN_MS = 1000;
    public static long TURRET_SETTLE_MS = 300;
    
    // Turret angle when at SHOOT_POSE
    // Blue: +44.3° (aim right toward blue basket at 0, 140)
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
     */
    private Command shootCommand() {
        return new SequentialCommandGroup(
                // 1. Set turret angle
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                // 1.5. TX auto-aim: if Limelight sees a goal tag, fine-tune turret angle
                new InstantCommand(() -> {
                    if (vision.hasTarget()) {
                        double tx = vision.getTx();
                        double correctedAngle = TURRET_SHOOT_ANGLE_DEG + tx;
                        turret.enableSoftLock(correctedAngle);
                    }
                }),
                new WaitCommand(200),  // Wait for turret to settle on corrected angle
                // 2. Accelerate shooter from idle to MID
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.MID)),
                // 3. Use TransitCommand (same as manual fire) with timeout
                new TransitCommand(transit, shooter)
                        .withTimeout(SHOOTER_SPINUP_TIMEOUT_MS + TRANSIT_OPEN_MS),
                // 4. Return shooter to idle and turret to forward
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new InstantCommand(() -> turret.enableSoftLock(0))
        );
    }
    
    @Override
    public Command runAutoCommand() {
        // === NEW PATHS (6 paths) ===
        
        // Path 1: Start → Shoot (heading 143.5° → 180°)
        path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), Math.toRadians(180))
                .build();
        
        // Path 2: Shoot → Sample 1 (curve)
        path2 = follower
                .pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE1_CTRL, SAMPLE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 3: Sample 1 → Intake 1 (curve)
        path3 = follower
                .pathBuilder()
                .addPath(new BezierCurve(SAMPLE1_POSE, INTAKE1_CTRL, INTAKE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 4: Intake 1 → Shoot (curve, mirrored control point)
        path4 = follower
                .pathBuilder()
                .addPath(new BezierCurve(INTAKE1_POSE, new Pose(53.40, 64.83), SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 5: Shoot → Intake 2 (curve)
        path5 = follower
                .pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, INTAKE2_CTRL, INTAKE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Path 6: Intake 2 → New Position (curve)
        path6 = follower
                .pathBuilder()
                .addPath(new BezierCurve(INTAKE2_POSE, NEW_POS_CTRL, NEW_POS))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // === BLUENEAR CONTINUATION (Path 6+) ===
        
        // BN Path 6: New Position → Shoot (straight)
        pathBN6 = follower
                .pathBuilder()
                .addPath(new BezierLine(NEW_POS, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // BN Path 7: Shoot → Sample 2 (double control curve)
        pathBN7 = follower
                .pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, BN_SAMPLE2_CTRL1, BN_SAMPLE2_CTRL2, BN_SAMPLE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // BN Path 8: Sample 2 → Shoot (straight)
        pathBN8 = follower
                .pathBuilder()
                .addPath(new BezierLine(BN_SAMPLE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // BN Path 9: Shoot → Final (straight)
        pathBN9 = follower
                .pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, BN_FINAL_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        
        // Build command sequence
        return new SequentialCommandGroup(
                // === NEW SEQUENCE ===
                // Path 1: Go to shoot position first (shooter not running yet)
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path1),
                // Shoot (shootCommand handles MID→fire→STOP)
                shootCommand(),
                
                // Path 2-3: Get sample 1
                new AutoDriveCommand(follower, path2),
                new AutoDriveCommand(follower, path3).setMaxPower(0.8),  // 80% power for gentle intake approach
                new WaitCommand(700),  // Wait at INTAKE1_POSE for intake
                
                // Path 4: Back to shoot
                new AutoDriveCommand(follower, path4),
                shootCommand(),
                
                // Path 5: Get intake 2
                new AutoDriveCommand(follower, path5),
                
                // Path 6: To new position (80% power for gentle approach)
                new AutoDriveCommand(follower, path6).setMaxPower(0.8),
                new WaitCommand(700),  // Wait at NEW_POS for intake
                
                // === BLUENEAR CONTINUATION ===
                // BN Path 6: Back to shoot
                new AutoDriveCommand(follower, pathBN6),
                shootCommand(),
                
                // BN Path 7: Get sample 2
                new AutoDriveCommand(follower, pathBN7),
                
                // BN Path 8: Back to shoot
                new AutoDriveCommand(follower, pathBN8),
                shootCommand(),
                
                // BN Path 9: Final position + STOP shooter
                new AutoDriveCommand(follower, pathBN9),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
