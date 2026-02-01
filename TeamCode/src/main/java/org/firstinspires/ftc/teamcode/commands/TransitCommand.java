package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/**
 * Command to control the Transit servo based on Shooter velocity.
 * "Smart Fire": Only pushes the ring when shooter is at target velocity.
 */
public class TransitCommand extends CommandBase {
    private final Transit transit;
    private final Shooter shooter;

    public TransitCommand(Transit transit, Shooter shooter) {
        this.transit = transit;
        this.shooter = shooter;
    }

    /**
     * Called repeatedly while the command is scheduled (usually while Trigger is held).
     * Sets transit to UP when shooter reaches target velocity.
     */
    @Override
    public void execute() {
        if (shooter.isShooterAtSetPoint()) {
            transit.setTransitState(Transit.TransitState.UP);
        }
    }

    @Override
    public boolean isFinished() {
        return false;  // Runs until timeout
    }

    /**
     * Called when the command ends (Trigger released).
     * Resets transit servo to DOWN.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.DOWN);
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
