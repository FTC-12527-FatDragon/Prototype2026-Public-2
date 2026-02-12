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
 * Vision subsystem — AprilTag detection via Limelight 3A.
 *
 * <h3>FTC DECODE 2025-2026 Tag Layout</h3>
 * <ul>
 *   <li>Blue goal — ID 20</li>
 *   <li>Red goal  — ID 24</li>
 *   <li>Obelisks (ignored) — IDs 21, 22, 23</li>
 * </ul>
 *
 * <p>All public getters return data for <b>goal tags only</b> (20 / 24).
 * A per-frame cache ({@link #refreshFrame()}) avoids redundant Limelight queries.</p>
 */
public class Vision extends SubsystemBase {

    // ── Constants ───────────────────────────────────────────────
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID  = 24;
    private static final double MIN_TARGET_AREA = 0.0;

    // ── Hardware ────────────────────────────────────────────────
    private final Limelight3A limelight;

    // ── Per-frame cache ─────────────────────────────────────────
    private long           cacheTimestamp = -1;
    private FiducialResult cachedGoalTag  = null;

    public enum Alliance { RED, BLUE, UNKNOWN }

    // ═════════════════════════════════════════════════════════════
    //  Construction
    // ═════════════════════════════════════════════════════════════

    public Vision(final HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // ═════════════════════════════════════════════════════════════
    //  Frame-level Cache
    // ═════════════════════════════════════════════════════════════

    /**
     * Refresh the per-frame cache.  Cheap to call multiple times per loop —
     * the Limelight is only queried once per unique timestamp.
     */
    private void refreshFrame() {
        long now = System.currentTimeMillis();
        if (now == cacheTimestamp) return;          // already refreshed this ms
        cacheTimestamp = now;

        LLResult result = limelight.getLatestResult();
        cachedGoalTag = null;

        if (result == null || !result.isValid()) return;

        List<FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        for (FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            if ((id == BLUE_GOAL_TAG_ID || id == RED_GOAL_TAG_ID)
                    && tag.getTargetArea() >= MIN_TARGET_AREA) {
                cachedGoalTag = tag;
                return;
            }
        }
    }

    /** Cached first goal tag for the current frame (null if none). */
    private FiducialResult goalTag() {
        refreshFrame();
        return cachedGoalTag;
    }

    // ═════════════════════════════════════════════════════════════
    //  Primary API
    // ═════════════════════════════════════════════════════════════

    /** Detected goal-tag ID (20 or 24), or −1. */
    public int getDetectedTagId() {
        FiducialResult tag = goalTag();
        return tag != null ? tag.getFiducialId() : -1;
    }

    /** Alliance colour of the detected tag. */
    public Alliance getDetectedAlliance() {
        int id = getDetectedTagId();
        if (id == BLUE_GOAL_TAG_ID) return Alliance.BLUE;
        if (id == RED_GOAL_TAG_ID)  return Alliance.RED;
        return Alliance.UNKNOWN;
    }

    /** True if a goal tag is currently visible. */
    public boolean hasTarget() { return getDetectedTagId() != -1; }

    /** Check whether the visible tag belongs to a given alliance. */
    public boolean isAllianceTag(Alliance a) { return getDetectedAlliance() == a; }

    /** Robot pose in field space (metres, degrees), or null. */
    public Pose3D getRobotPose() {
        FiducialResult tag = goalTag();
        return tag != null ? tag.getRobotPoseFieldSpace() : null;
    }

    // ═════════════════════════════════════════════════════════════
    //  TX / TY / Distance
    // ═════════════════════════════════════════════════════════════

    /** Horizontal offset to goal tag (degrees). 0 if no tag. */
    public double getTx() {
        FiducialResult tag = goalTag();
        return tag != null ? tag.getTargetXDegrees() : 0;
    }

    /** Vertical offset to goal tag (degrees). 0 if no tag. */
    public double getTy() {
        FiducialResult tag = goalTag();
        return tag != null ? tag.getTargetYDegrees() : 0;
    }

    /** 3D distance to goal tag in inches, or −1. */
    public double getDistanceToTag() {
        FiducialResult tag = goalTag();
        if (tag == null) return -1;
        Pose3D pose = tag.getRobotPoseTargetSpace();
        if (pose == null) return -1;
        double x = pose.getPosition().x, y = pose.getPosition().y, z = pose.getPosition().z;
        return Math.sqrt(x * x + y * y + z * z) * 39.3701;
    }

    /** Target area of goal tag, or −1. */
    public double getTagArea() {
        FiducialResult tag = goalTag();
        return tag != null ? tag.getTargetArea() : -1;
    }

    // ═════════════════════════════════════════════════════════════
    //  Diagnostics
    // ═════════════════════════════════════════════════════════════

    public LLStatus getStatus()       { return limelight.getStatus(); }
    public boolean  isConnected()     { LLStatus s = getStatus(); return s != null && s.getFps() > 0; }
    public int      getPipelineIndex(){ LLStatus s = getStatus(); return s != null ? (int) s.getPipelineIndex() : -1; }
    public double   getFps()          { LLStatus s = getStatus(); return s != null ? s.getFps() : 0; }
    public void     setPipeline(int i){ limelight.pipelineSwitch(i); }

    public boolean isResultValid() {
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid();
    }

    public int getNumTagsDetected() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0;
        int n = 0;
        for (FiducialResult tag : r.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id == BLUE_GOAL_TAG_ID || id == RED_GOAL_TAG_ID) n++;
        }
        return n;
    }

    public String getDebugState() {
        LLResult result = limelight.getLatestResult();
        if (result == null)      return "result=NULL";
        if (!result.isValid())   return "result.isValid=FALSE";
        List<FiducialResult> tags = result.getFiducialResults();
        if (tags == null)        return "fiducialResults=NULL";
        if (tags.isEmpty())      return "fiducialResults=EMPTY";

        FiducialResult tag = goalTag();
        if (tag == null) {
            StringBuilder sb = new StringBuilder("No goal tag (20/24). Seen: ");
            for (FiducialResult t : tags) sb.append(t.getFiducialId()).append(' ');
            return sb.toString();
        }
        return "OK: goal tag " + tag.getFiducialId() + " detected";
    }

    public void stop() { limelight.stop(); }
}

// Special thanks to PeterLu for contributions to this code. All code and interpretation rights belong to PeterLu.
