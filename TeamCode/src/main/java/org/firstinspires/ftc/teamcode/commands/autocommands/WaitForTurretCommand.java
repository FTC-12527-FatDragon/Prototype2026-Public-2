package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

/**
 * Command that waits for turret to reach target angle.
 * Finishes when turret.isOnTarget() returns true or timeout is reached.
 */
public class WaitForTurretCommand extends CommandBase {
    private final Turret turret;
    private final double timeoutMs;
    private final ElapsedTime timer;
    
    /**
     * Constructor with default 1 second timeout.
     * @param turret The turret subsystem
     */
    public WaitForTurretCommand(Turret turret) {
        this(turret, 1000);
    }
    
    /**
     * Constructor with custom timeout.
     * @param turret The turret subsystem
     * @param timeoutMs Timeout in milliseconds
     */
    public WaitForTurretCommand(Turret turret, double timeoutMs) {
        this.turret = turret;
        this.timeoutMs = timeoutMs;
        this.timer = new ElapsedTime();
    }
    
    @Override
    public void initialize() {
        timer.reset();
    }
    
    @Override
    public void execute() {
        // Turret.periodic() handles the actual movement
        // This command just waits
    }
    
    @Override
    public boolean isFinished() {
        // Finish if turret is at target OR timeout reached
        return turret.isOnTarget() || timer.milliseconds() >= timeoutMs;
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
