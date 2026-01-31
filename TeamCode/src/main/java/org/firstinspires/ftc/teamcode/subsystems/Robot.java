package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivePinpoint;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/**
 * Robot Hardware Container for TeleOp.
 * Initializes and holds references to all robot subsystems including MecanumDrivePinpoint.
 * 
 * NOTE: This class is intended for TeleOp only!
 * For Autonomous, use AutoCommandBase instead, which:
 * - Uses Follower (Pedro Pathing) instead of MecanumDrivePinpoint
 * - Does NOT reset Pinpoint odometry to avoid conflicts with Follower
 * 
 * @see org.firstinspires.ftc.teamcode.opmodes.autos.AutoCommandBase
 */
public class Robot {
    public final MecanumDrivePinpoint drive;
    public final Shooter shooter;
    public final Transit transit;
    public final Intake intake;
    public final Vision vision;
    public final Turret turret;

    /**
     * Constructor for Robot.
     * Initializes all subsystems using the provided HardwareMap.
     *
     * @param hardwareMap The hardware map from the OpMode.
     */
    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrivePinpoint(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap);
        
        // VISION/LIMELIGHT DISABLED
        vision = null;
        // vision = new Vision(hardwareMap);
        
        // TURRET DISABLED
        turret = null;
        // turret = new Turret(hardwareMap);
    }
}

