package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit.TransitState;
import org.firstinspires.ftc.teamcode.utils.Util;

/**
 * Command to control transit in autonomous with position and velocity checks.
 */
public class AutoTransitCommand extends CommandBase {
    private final Transit transit;
    private final Shooter shooter;
    private final Follower follower;
    private final Pose targetPose;
    private final Intake intake;

    // Tolerances for position check
    private static final double POSITION_TOLERANCE = 10.0; // inches
    private static final double HEADING_TOLERANCE = 20.0;  // degrees

    public AutoTransitCommand(Transit transit, Shooter shooter, Follower follower, Pose targetPose, Intake intake) {
        this.transit = transit;
        this.shooter = shooter;
        this.follower = follower;
        this.targetPose = targetPose;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        transit.setTransitState(TransitState.DOWN);
    }

    @Override
    public void execute() {
        boolean atPosition = isAtTargetPosition();
        boolean shooterReady = shooter.isShooterAtSetPoint();

        // Control intake fast shooting based on position
        intake.setFastShooting(atPosition);

        if (atPosition && shooterReady) {
            transit.setTransitState(TransitState.UP);
        } else {
            transit.setTransitState(TransitState.DOWN);
        }
    }

    private boolean isAtTargetPosition() {
        Pose currentPose = follower.getPose();

        double dx = currentPose.getX() - targetPose.getX();
        double dy = currentPose.getY() - targetPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);

        double currentHeading = Math.toDegrees(currentPose.getHeading());
        double targetHeading = Math.toDegrees(targetPose.getHeading());
        double headingError = Math.abs(Util.normalizeAngleDegrees(currentHeading - targetHeading));

        return distance <= POSITION_TOLERANCE && headingError <= HEADING_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(TransitState.DOWN);
        intake.setFastShooting(false);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until timeout
    }
}


// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
