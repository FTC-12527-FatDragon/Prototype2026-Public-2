package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/**
 * Command for auto-aligning to a goal AprilTag (20 or 24) in autonomous.
 * Uses Limelight's tx to calculate target heading, then uses Follower to turn.
 * Finishes when aligned (tx < threshold) or after timeout.
 */
public class AutoAlignCommand extends CommandBase {
    private final Follower follower;
    private final Vision vision;
    
    // Alignment parameters
    private static final double TX_THRESHOLD = 1.0; // Degrees - considered aligned when tx < this
    
    private int alignedFrameCount = 0;
    private static final int REQUIRED_ALIGNED_FRAMES = 3;  // Need 3 consecutive aligned frames
    
    /**
     * Creates an AutoAlignCommand.
     * @param follower The PedroPathing Follower instance.
     * @param vision The Vision subsystem.
     */
    public AutoAlignCommand(Follower follower, Vision vision) {
        this.follower = follower;
        this.vision = vision;
        // No addRequirements - Follower is not a Subsystem
    }
    
    @Override
    public void initialize() {
        alignedFrameCount = 0;
    }
    
    @Override
    public void execute() {
        int tagId = vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        if (!isGoalTag) {
            // No goal tag in view, hold current position
            Pose currentPose = follower.getPose();
            follower.holdPoint(currentPose);
            follower.update();
            alignedFrameCount = 0;  // Reset counter
            return;
        }
        
        double tx = vision.getTx();
        
        // Check if aligned
        if (Math.abs(tx) < TX_THRESHOLD) {
            alignedFrameCount++;
            // Hold current position when aligned
            Pose currentPose = follower.getPose();
            follower.holdPoint(currentPose);
        } else {
            alignedFrameCount = 0;  // Reset counter if not aligned
            
            // Calculate target heading based on tx
            // tx > 0 means target is to the right, so we need to turn right (decrease heading)
            Pose currentPose = follower.getPose();
            double targetHeading = currentPose.getHeading() - Math.toRadians(tx);
            
            // Create target pose with same X/Y but new heading
            Pose targetPose = new Pose(currentPose.getX(), currentPose.getY(), targetHeading);
            follower.holdPoint(targetPose);
        }
        
        follower.update();
    }
    
    @Override
    public boolean isFinished() {
        // Finish when aligned for enough frames
        return alignedFrameCount >= REQUIRED_ALIGNED_FRAMES;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Follower will continue to hold last position
    }
}


// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
