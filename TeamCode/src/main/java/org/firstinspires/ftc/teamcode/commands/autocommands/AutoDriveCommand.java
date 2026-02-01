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
     * Start following the path.
     */
    @Override
    public void initialize() {
        timer.reset();
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
     */
    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
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
