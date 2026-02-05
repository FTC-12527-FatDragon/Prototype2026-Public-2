package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Command that waits until the shooter reaches its target velocity.
 * Used in autonomous to ensure shooter is ready before firing.
 */
public class WaitForShooterCommand extends CommandBase {
    private final Shooter shooter;
    private final long timeoutMs;
    private long startTime;
    
    /**
     * Creates a new WaitForShooterCommand.
     * @param shooter The shooter subsystem.
     * @param timeoutMs Maximum time to wait in milliseconds.
     */
    public WaitForShooterCommand(Shooter shooter, long timeoutMs) {
        this.shooter = shooter;
        this.timeoutMs = timeoutMs;
    }
    
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }
    
    @Override
    public boolean isFinished() {
        // Finish if shooter is at setpoint OR timeout exceeded
        boolean atSetpoint = shooter.isShooterAtSetPoint();
        boolean timedOut = (System.currentTimeMillis() - startTime) > timeoutMs;
        return atSetpoint || timedOut;
    }
}
