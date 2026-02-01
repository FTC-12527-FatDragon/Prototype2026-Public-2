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
     *
     * @return The AprilTag ID, or -1 if no tag is detected.
     */
    public int getDetectedTagId() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1;
        }
        
        List<FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            return -1;
        }
        
        // Get the first detected tag (largest/closest)
        FiducialResult firstTag = fiducialResults.get(0);
        
        // Filter by minimum target area to reduce noise
        if (firstTag.getTargetArea() < MIN_TARGET_AREA) {
            return -1;
        }
        
        return firstTag.getFiducialId();
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
     * Uses the first detected AprilTag to calculate robot pose.
     *
     * @return Pose3D of the robot in field space (meters, degrees), or null if no valid tag detected.
     */
    public Pose3D getRobotPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        
        List<FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            return null;
        }
        
        FiducialResult firstTag = fiducialResults.get(0);
        
        // Filter by minimum target area to reduce noise
        if (firstTag.getTargetArea() < MIN_TARGET_AREA) {
            return null;
        }
        
        // Return robot pose in field space
        return firstTag.getRobotPoseFieldSpace();
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
     * Gets the number of fiducials (AprilTags) detected.
     * @return Number of tags detected.
     */
    public int getNumTagsDetected() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return 0;
        }
        return result.getFiducialResults().size();
    }
    
    /**
     * Switches to a different pipeline.
     * @param pipelineIndex Pipeline index (0-9).
     */
    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }
    
    /**
     * Gets the raw tag ID without area filtering (for debugging).
     * @return Tag ID or -1 if no tag.
     */
    public int getRawTagId() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1;
        }
        List<FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            return -1;
        }
        return fiducialResults.get(0).getFiducialId();
    }
    
    /**
     * Gets the target area of the first detected tag (for debugging).
     * @return Target area or -1 if no tag.
     */
    public double getTagArea() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1;
        }
        List<FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            return -1;
        }
        return fiducialResults.get(0).getTargetArea();
    }
    
    /**
     * Gets tx (horizontal offset) for debugging.
     * @return tx value or 0 if no result.
     */
    public double getTx() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0;
    }
    
    /**
     * Gets ty (vertical offset) for debugging.
     * @return ty value or 0 if no result.
     */
    public double getTy() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTy();
        }
        return 0;
    }
    
    /**
     * Gets the distance from robot to the detected AprilTag in inches.
     * Uses the robot pose in target space to calculate distance.
     * 
     * @return Distance in inches, or -1 if no valid tag detected.
     */
    public double getDistanceToTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1;
        }
        
        List<FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            return -1;
        }
        
        FiducialResult firstTag = fiducialResults.get(0);
        
        // Get robot pose relative to target (in meters)
        Pose3D robotPoseTargetSpace = firstTag.getRobotPoseTargetSpace();
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
     * Gets the raw robot pose WITHOUT any filtering (for debugging).
     * @return Pose3D or null if no fiducial results.
     */
    public Pose3D getRawRobotPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            return null;
        }
        // Don't check isValid() - try to get data anyway
        List<FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults == null || fiducialResults.isEmpty()) {
            return null;
        }
        // No area filtering - return raw pose
        return fiducialResults.get(0).getRobotPoseFieldSpace();
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
        FiducialResult first = fiducialResults.get(0);
        double area = first.getTargetArea();
        if (area < MIN_TARGET_AREA) {
            return "area=" + area + " < " + MIN_TARGET_AREA;
        }
        Pose3D pose = first.getRobotPoseFieldSpace();
        if (pose == null) {
            return "getRobotPoseFieldSpace=NULL";
        }
        return "OK: pose available";
    }
}


// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
