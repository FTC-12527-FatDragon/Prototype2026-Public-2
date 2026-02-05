package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry    .Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.WaitForShooterCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/**
 * Red Near Auto 2 - Modified sample collection auto (Red Alliance, Near side)
 * Mirrored from BlueNearAuto2 along x=72 axis
 * 
 * New Sequence (6 paths) + RedNear Path 6+ continuation
 */
@Config
@Autonomous(name = "Red Near Auto 2", group = "Autos")
public class RedNearAuto2 extends AutoCommandBase {
    
    // Path declarations
    private PathChain path1, path2, path3, path4, path5, path6;
    private PathChain pathRN6, pathRN7, pathRN8, pathRN9; // RedNear continuation
    
    // Key positions (Red side - mirrored at x=72)
    // Mirror formula: new_x = 144 - old_x, heading 180° → 0°
    private static final Pose START_POSE = new Pose(118.32, 127.97, Math.toRadians(36.5));   // 144 - 25.68, mirrored angle
    private static final Pose SHOOT_POSE = new Pose(98.80, 101.15, Math.toRadians(0));       // 144 - 45.20
    
    // New path positions (mirrored)
    private static final Pose SAMPLE1_POSE = new Pose(135.21, 59.30, Math.toRadians(0));     // 144 - 8.79
    private static final Pose SAMPLE1_CTRL = new Pose(76.38, 56.74);                          // 144 - 67.62
    
    private static final Pose INTAKE1_POSE = new Pose(127.55, 69.66, Math.toRadians(0));     // 144 - 16.45
    private static final Pose INTAKE1_CTRL = new Pose(117.11, 61.55);                         // 144 - 26.89
    
    private static final Pose INTAKE2_POSE = new Pose(127.83, 83.46, Math.toRadians(0));     // 144 - 16.17
    private static final Pose INTAKE2_CTRL = new Pose(97.26, 81.85);                          // 144 - 46.74
    
    private static final Pose NEW_POS = new Pose(127.23, 68.97, Math.toRadians(0));          // 144 - 16.77
    private static final Pose NEW_POS_CTRL = new Pose(111.15, 70.64);                         // 144 - 32.85
    
    // RedNear continuation positions (mirrored)
    private static final Pose RN_SAMPLE2_POSE = new Pose(129.81, 35.31, Math.toRadians(0));  // 144 - 14.19
    private static final Pose RN_SAMPLE2_CTRL1 = new Pose(80.38, 43.59);                      // 144 - 63.62
    private static final Pose RN_SAMPLE2_CTRL2 = new Pose(73.67, 33.14);                      // 144 - 70.33
    
    private static final Pose RN_FINAL_POSE = new Pose(128.04, 101.11, Math.toRadians(0));   // 144 - 15.96
    
    // Wait times (ms)
    public static long INTAKE_WAIT_MS = 500;
    public static long SHOOTER_SPINUP_TIMEOUT_MS = 2000;
    public static long TRANSIT_OPEN_MS = 1200;
    public static long TURRET_SETTLE_MS = 300;
    
    // Turret angle when at SHOOT_POSE
    // Red: -43.3° (aim left toward red basket at 140, 140)
    public static double TURRET_SHOOT_ANGLE_DEG = -43.3;
    
    @Override
    public Pose getStartPose() {
        return START_POSE;
    }
    
    /**
     * Shoot command - Shooter is already running in SLOW mode.
     * Uses same firing logic as Solo: waits for shooter speed, triggers boost, then fires.
     */
    private Command shootCommand() {
        return new SequentialCommandGroup(
                // 1. Set turret angle and wait for it to reach
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new WaitCommand(TURRET_SETTLE_MS),
                // 2. Wait for shooter to reach target velocity
                new WaitForShooterCommand(shooter, 2000),
                // 3. Start firing boost timer (same as Solo LT + bumper)
                new InstantCommand(() -> shooter.setTransitFiring(true)),
                // 4. Wait for boost delay (200ms) before opening transit
                new WaitCommand(200),
                // 5. Open transit to release ball (boost now active)
                new InstantCommand(() -> transit.setTransitState(Transit.TransitState.UP)),
                // 6. Wait for ball to exit (boost continues ramping during this time)
                new WaitCommand(TRANSIT_OPEN_MS),
                // 7. Close transit
                new InstantCommand(() -> transit.setTransitState(Transit.TransitState.DOWN)),
                // 8. End firing boost
                new InstantCommand(() -> shooter.setTransitFiring(false)),
                // 9. Return turret to forward (shooter keeps running)
                new InstantCommand(() -> turret.enableSoftLock(0))
        );
    }
    
    private Command intakeWaitCommand() {
        return new WaitCommand(INTAKE_WAIT_MS);
    }
    
    @Override
    public Command runAutoCommand() {
        // === NEW PATHS (6 paths) ===
        
        // Path 1: Start → Shoot (heading 90° → 0°)
        path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), Math.toRadians(0))
                .build();
        
        // Path 2: Shoot → Sample 1 (curve)
        path2 = follower
                .pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, SAMPLE1_CTRL, SAMPLE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // Path 3: Sample 1 → Intake 1 (curve)
        path3 = follower
                .pathBuilder()
                .addPath(new BezierCurve(SAMPLE1_POSE, INTAKE1_CTRL, INTAKE1_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // Path 4: Intake 1 → Shoot (curve, mirrored control point)
        path4 = follower
                .pathBuilder()
                .addPath(new BezierCurve(INTAKE1_POSE, new Pose(90.60, 64.83), SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // Path 5: Shoot → Intake 2 (curve)
        path5 = follower
                .pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, INTAKE2_CTRL, INTAKE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // Path 6: Intake 2 → New Position (curve)
        path6 = follower
                .pathBuilder()
                .addPath(new BezierCurve(INTAKE2_POSE, NEW_POS_CTRL, NEW_POS))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // === REDNEAR CONTINUATION (Path 6+) ===
        
        // RN Path 6: New Position → Shoot (straight)
        pathRN6 = follower
                .pathBuilder()
                .addPath(new BezierLine(NEW_POS, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // RN Path 7: Shoot → Sample 2 (double control curve)
        pathRN7 = follower
                .pathBuilder()
                .addPath(new BezierCurve(SHOOT_POSE, RN_SAMPLE2_CTRL1, RN_SAMPLE2_CTRL2, RN_SAMPLE2_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // RN Path 8: Sample 2 → Shoot (straight)
        pathRN8 = follower
                .pathBuilder()
                .addPath(new BezierLine(RN_SAMPLE2_POSE, SHOOT_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // RN Path 9: Shoot → Final (straight)
        pathRN9 = follower
                .pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, RN_FINAL_POSE))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        
        // Build command sequence
        return new SequentialCommandGroup(
                // START: Keep shooter running at SLOW throughout entire auto
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                
                // === NEW SEQUENCE ===
                // Path 1: Go to shoot + FIRST SHOT (preloaded ball)
                new InstantCommand(() -> turret.enableSoftLock(TURRET_SHOOT_ANGLE_DEG)),
                new AutoDriveCommand(follower, path1),
                shootCommand(),
                
                // Path 2-3: Get sample 1
                new AutoDriveCommand(follower, path2),
                new AutoDriveCommand(follower, path3),
                intakeWaitCommand(),
                
                // Path 4: Back to shoot
                new AutoDriveCommand(follower, path4),
                shootCommand(),
                
                // Path 5: Get intake 2
                new AutoDriveCommand(follower, path5),
                intakeWaitCommand(),
                
                // Path 6: To new position
                new AutoDriveCommand(follower, path6),
                intakeWaitCommand(),
                
                // === REDNEAR CONTINUATION ===
                // RN Path 6: Back to shoot
                new AutoDriveCommand(follower, pathRN6),
                shootCommand(),
                
                // RN Path 7: Get sample 2
                new AutoDriveCommand(follower, pathRN7),
                intakeWaitCommand(),
                
                // RN Path 8: Back to shoot
                new AutoDriveCommand(follower, pathRN8),
                shootCommand(),
                
                // RN Path 9: Final position + STOP shooter
                new AutoDriveCommand(follower, pathRN9),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
