package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

/**
 * Command to hold robot position using Follower's holdPoint.
 */
public class HoldPositionCommand extends CommandBase {
    private final Follower follower;

    public HoldPositionCommand(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void initialize() {
        follower.holdPoint(follower.getPose());
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until externally cancelled
    }
}


// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
