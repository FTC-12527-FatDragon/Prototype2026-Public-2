package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Vision subsystem for AprilTag detection using Limelight3A.
 * Detects AprilTags and determines alliance color based on tag ID.
 * 
 * FTC DECODE 2025-2026 AprilTag Layout:
 * - Blue alliance goal: ID 20
 * - Red alliance goal: ID 24
 * - Obelisk (not for navigation): ID 21, 22, 23
 */
public class Vision extends SubsystemBase {
    
    private final Limelight3A limelight;
    
    /**
     * Alliance color enumeration.
     */
    public enum Alliance {
        RED,
        BLUE,
        UNKNOWN
    }
    
    // FTC DECODE 2025-2026 AprilTag IDs:
    // Blue alliance goal: 20
    // Red alliance goal: 24
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID = 24;
    
    // Minimum target area threshold to filter noise
    // Lowered to 0.001 to avoid filtering out distant tags
    private static final double MIN_TARGET_AREA = 0.0000;
    
    /**
     * Constructor for Vision subsystem.
     * Initializes Limelight and starts polling.
     *
     * @param hardwareMap The hardware map from the OpMode.
     */
    public Vision(final HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);  // Set polling rate to 50Hz
        limelight.pipelineSwitch(0);  // Switch to AprilTag pipeline (pipeline 0)
        limelight.start();            // Start polling for data
    }
    
    /**
     * Gets the ID of the currently detected AprilTag.
     * ONLY returns tag 20 (blue goal) or 24 (red goal), ignores all others.
     *
     * @return The AprilTag ID (20 or 24), or -1 if no goal tag is detected.
     */
    public int getDetectedTagId() {
        FiducialResult goalTag = getFirstGoalTag();
        if (goalTag == null) {
            return -1;
        }
        return goalTag.getFiducialId();
    }
    
    /**
     * Gets the first detected goal tag (20 or 24) from results.
     * Ignores all other tags (21, 22, 23, etc.).
     *
     * @return FiducialResult for tag 20 or 24, or null if no goal tag found.
     */
    private FiducialResult getFirstGoalTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        
        List<FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            return null;
        }
        
        // Search for first goal tag (20 or 24)
        for (FiducialResult tag : fiducialResults) {
            int tagId = tag.getFiducialId();
            // Only accept goal tags (20 = blue, 24 = red)
            if (tagId == BLUE_GOAL_TAG_ID || tagId == RED_GOAL_TAG_ID) {
                // Filter by minimum target area
                if (tag.getTargetArea() >= MIN_TARGET_AREA) {
                    return tag;
                }
            }
        }
        
        return null;  // No goal tag found
    }
    
    /**
     * Determines the alliance color based on the detected AprilTag ID.
     *
     * @return Alliance.RED, Alliance.BLUE, or Alliance.UNKNOWN if no valid tag is detected.
     */
    public Alliance getDetectedAlliance() {
        int tagId = getDetectedTagId();
        
        if (tagId == -1) {
            return Alliance.UNKNOWN;
        }
        
        // Blue alliance goal: ID 20
        if (tagId == BLUE_GOAL_TAG_ID) {
            return Alliance.BLUE;
        }
        
        // Red alliance goal: ID 24
        if (tagId == RED_GOAL_TAG_ID) {
            return Alliance.RED;
        }
        
        // Other IDs (e.g., obelisk tags 21-23) - unknown
        return Alliance.UNKNOWN;
    }
    
    /**
     * Checks if Limelight currently sees any AprilTag.
     *
     * @return True if an AprilTag is detected, false otherwise.
     */
    public boolean hasTarget() {
        return getDetectedTagId() != -1;
    }
    
    /**
     * Checks if the detected tag belongs to a specific alliance.
     *
     * @param alliance The alliance to check for.
     * @return True if the detected tag belongs to the specified alliance.
     */
    public boolean isAllianceTag(Alliance alliance) {
        return getDetectedAlliance() == alliance;
    }
    
    /**
     * Gets the robot's position in field space from AprilTag detection.
     * ONLY uses goal tags (20 or 24), ignores all other tags.
     *
     * @return Pose3D of the robot in field space (meters, degrees), or null if no goal tag detected.
     */
    public Pose3D getRobotPose() {
        FiducialResult goalTag = getFirstGoalTag();
        if (goalTag == null) {
            return null;
        }
        return goalTag.getRobotPoseFieldSpace();
    }
    
    /**
     * Stops the Limelight polling.
     * Should be called when the OpMode ends.
     */
    public void stop() {
        limelight.stop();
    }
    
    // ==================== DEBUG METHODS ====================
    
    /**
     * Gets Limelight status information for debugging.
     * @return LLStatus object with temperature, CPU usage, FPS, etc.
     */
    public LLStatus getStatus() {
        return limelight.getStatus();
    }
    
    /**
     * Checks if Limelight is connected and running.
     * @return True if Limelight is connected.
     */
    public boolean isConnected() {
        LLStatus status = limelight.getStatus();
        return status != null && status.getFps() > 0;
    }
    
    /**
     * Gets the current pipeline index.
     * @return Pipeline index (0-9).
     */
    public int getPipelineIndex() {
        LLStatus status = limelight.getStatus();
        return status != null ? (int) status.getPipelineIndex() : -1;
    }
    
    /**
     * Gets the current FPS.
     * @return Frames per second.
     */
    public double getFps() {
        LLStatus status = limelight.getStatus();
        return status != null ? status.getFps() : 0;
    }
    
    /**
     * Checks if the latest result is valid.
     * @return True if result is valid.
     */
    public boolean isResultValid() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }
    
    /**
     * Gets the number of goal tags (20 or 24) detected.
     * @return Number of goal tags detected (0, 1, or 2).
     */
    public int getNumTagsDetected() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return 0;
        }
        
        int count = 0;
        for (FiducialResult tag : result.getFiducialResults()) {
            int tagId = tag.getFiducialId();
            if (tagId == BLUE_GOAL_TAG_ID || tagId == RED_GOAL_TAG_ID) {
                count++;
            }
        }
        return count;
    }
    
    /**
     * Switches to a different pipeline.
     * @param pipelineIndex Pipeline index (0-9).
     */
    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }
    
    /**
     * Gets the raw tag ID (only goal tags 20/24).
     * @return Tag ID (20 or 24), or -1 if no goal tag.
     */
    public int getRawTagId() {
        // Same as getDetectedTagId - only return goal tags
        return getDetectedTagId();
    }
    
    /**
     * Gets the target area of the first detected goal tag (20 or 24).
     * @return Target area or -1 if no goal tag.
     */
    public double getTagArea() {
        FiducialResult goalTag = getFirstGoalTag();
        if (goalTag == null) {
            return -1;
        }
        return goalTag.getTargetArea();
    }
    
    /**
     * Gets tx (horizontal offset) for goal tags only.
     * @return tx value or 0 if no goal tag detected.
     */
    public double getTx() {
        // Only return TX if we see a goal tag (20 or 24)
        FiducialResult goalTag = getFirstGoalTag();
        if (goalTag == null) {
            return 0;
        }
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0;
    }
    
    /**
     * Gets ty (vertical offset) for goal tags only.
     * @return ty value or 0 if no goal tag detected.
     */
    public double getTy() {
        // Only return TY if we see a goal tag (20 or 24)
        FiducialResult goalTag = getFirstGoalTag();
        if (goalTag == null) {
            return 0;
        }
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTy();
        }
        return 0;
    }
    
    /**
     * Gets the distance from robot to the detected goal AprilTag in inches.
     * ONLY uses goal tags (20 or 24).
     * 
     * @return Distance in inches, or -1 if no goal tag detected.
     */
    public double getDistanceToTag() {
        FiducialResult goalTag = getFirstGoalTag();
        if (goalTag == null) {
            return -1;
        }
        
        // Get robot pose relative to target (in meters)
        Pose3D robotPoseTargetSpace = goalTag.getRobotPoseTargetSpace();
        if (robotPoseTargetSpace == null) {
            return -1;
        }
        
        // Get position components (in meters)
        double x = robotPoseTargetSpace.getPosition().x;
        double y = robotPoseTargetSpace.getPosition().y;
        double z = robotPoseTargetSpace.getPosition().z;
        
        // Calculate 3D distance and convert from meters to inches
        double distanceMeters = Math.sqrt(x * x + y * y + z * z);
        double distanceInches = distanceMeters * 39.3701;  // 1 meter = 39.3701 inches
        
        return distanceInches;
    }
    
    /**
     * Gets the raw robot pose for goal tags only (20 or 24).
     * @return Pose3D or null if no goal tag.
     */
    public Pose3D getRawRobotPose() {
        FiducialResult goalTag = getFirstGoalTag();
        if (goalTag == null) {
            return null;
        }
        return goalTag.getRobotPoseFieldSpace();
    }
    
    /**
     * Debug method to check why getRobotPose returns null.
     * @return Debug string explaining the state.
     */
    public String getDebugState() {
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            return "result=NULL";
        }
        if (!result.isValid()) {
            return "result.isValid=FALSE";
        }
        List<FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults == null) {
            return "fiducialResults=NULL";
        }
        if (fiducialResults.isEmpty()) {
            return "fiducialResults=EMPTY";
        }
        
        // Check for goal tags
        FiducialResult goalTag = getFirstGoalTag();
        if (goalTag == null) {
            // List what tags were seen
            StringBuilder sb = new StringBuilder("No goal tag (20/24). Seen: ");
            for (FiducialResult tag : fiducialResults) {
                sb.append(tag.getFiducialId()).append(" ");
            }
            return sb.toString();
        }
        
        Pose3D pose = goalTag.getRobotPoseFieldSpace();
        if (pose == null) {
            return "getRobotPoseFieldSpace=NULL for tag " + goalTag.getFiducialId();
        }
        return "OK: goal tag " + goalTag.getFiducialId() + " detected";
    }
}


// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
