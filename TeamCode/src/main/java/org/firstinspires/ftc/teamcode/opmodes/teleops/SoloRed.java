package org.firstinspires.ftc.teamcode.opmodes.teleops;

import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

/**
 * SoloRed — Inertial auto-aim to RED basket (140, 140).
 * All logic lives in {@link SoloBase}.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Solo Red", group = "TeleOp")
public class SoloRed extends SoloBase {

    @Override protected Turret.Alliance getAlliance() { return Turret.Alliance.RED; }
    @Override protected double getGoalX()            { return TurretConstants.redGoalX; }
    @Override protected double getGoalY()            { return TurretConstants.redGoalY; }
    @Override protected String getGoalColor()        { return "#FF0000"; }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
