package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Command for Autonomous driving using Pedro Pathing.
 * Follows a specified PathChain until completion.
 * Aligned with Prototype2026-Public implementation.
 */
public class AutoDriveCommand extends CommandBase {
    private Follower follower;
    private PathChain pathChain;
    private double waitTime;
    private double maxPower = 1.0;  // Default: full power
    private final ElapsedTime timer;

    /**
     * Constructor for AutoDriveCommand with default 30s timeout.
     *
     * @param follower The Pedro Pathing Follower instance.
     * @param pathChain The PathChain to follow.
     */
    public AutoDriveCommand(Follower follower, PathChain pathChain) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.waitTime = 30 * 1000;
        this.timer = new ElapsedTime();
    }

    /**
     * Constructor for AutoDriveCommand with custom timeout.
     *
     * @param follower The Pedro Pathing Follower instance.
     * @param pathChain The PathChain to follow.
     * @param waitTime Timeout in milliseconds.
     */
    public AutoDriveCommand(Follower follower, PathChain pathChain, double waitTime) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.waitTime = waitTime;
        this.timer = new ElapsedTime();
    }

    /**
     * Constructor for AutoDriveCommand with custom max power and default timeout.
     *
     * @param follower The Pedro Pathing Follower instance.
     * @param pathChain The PathChain to follow.
     * @param maxPower Maximum motor power (0.0 to 1.0).
     * @param waitTime Timeout in milliseconds.
     */
    public AutoDriveCommand(Follower follower, PathChain pathChain, double maxPower, double waitTime) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.maxPower = maxPower;
        this.waitTime = waitTime;
        this.timer = new ElapsedTime();
    }

    /**
     * Sets the maximum power for this path.
     * @param maxPower Maximum motor power (0.0 to 1.0).
     * @return this command for chaining.
     */
    public AutoDriveCommand setMaxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    /**
     * Start following the path.
     */
    @Override
    public void initialize() {
        timer.reset();
        follower.setMaxPower(maxPower);
        follower.followPath(pathChain);
    }

    /**
     * Update the follower loop.
     */
    @Override
    public void execute() {
        follower.update();
    }

    /**
     * Stop following when command ends.
     * Resets max power to 1.0.
     */
    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
        follower.setMaxPower(1.0);  // Reset to full power for next path
    }

    /**
     * Command finishes when the follower is no longer busy OR timeout reached.
     */
    @Override
    public boolean isFinished() {
        return !follower.isBusy() || timer.milliseconds() >= waitTime;
    }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
