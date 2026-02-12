package org.firstinspires.ftc.teamcode.opmodes.teleops;

import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

/**
 * SoloBlue — Inertial auto-aim to BLUE basket (4, 140).
 * All logic lives in {@link SoloBase}.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Solo Blue", group = "TeleOp")
public class SoloBlue extends SoloBase {

    @Override protected Turret.Alliance getAlliance() { return Turret.Alliance.BLUE; }
    @Override protected double getGoalX()            { return TurretConstants.blueGoalX; }
    @Override protected double getGoalY()            { return TurretConstants.blueGoalY; }
    @Override protected String getGoalColor()        { return "#0000FF"; }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
