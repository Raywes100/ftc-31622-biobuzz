package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Vision subsystem. Owns the Limelight and answers one question for the motion/firing code:
 * where is our CELL relative to the robot right now?
 *
 * Contract (see VISION_ICD.md):
 *   - Call init() in the OpMode's init, start() before the loop, update() once per loop, stop() at the end.
 *   - getTarget() returns the latest Target. Check target.valid before using any numbers.
 *   - Frame: robot-centric, origin at robot center on the floor. x forward (in), y left (in), z up (in).
 *     bearingDeg is positive to the LEFT (CCW, same sign as heading). rangeIn is horizontal distance.
 *   - A Target older than MAX_AGE_MS is reported valid = false.
 *
 * Nothing in here moves the robot.
 */
public class TargetVision {

    public enum Alliance { RED, BLUE }

    /** Immutable snapshot handed to the controls code. */
    public static class Target {
        public final boolean valid;
        public final double xIn, yIn, zIn;        // target position in robot frame
        public final double bearingDeg;          // atan2(y, x), +left
        public final double rangeIn;             // sqrt(x^2 + y^2)
        public final int tagId;                  // tag used for the solve
        public final int tagCount;               // our-alliance tags seen this frame
        public final long ageMs;                 // time since the camera frame

        Target(boolean valid, double x, double y, double z, int tagId, int tagCount, long ageMs) {
            this.valid = valid; this.xIn = x; this.yIn = y; this.zIn = z;
            this.bearingDeg = Math.toDegrees(Math.atan2(y, x));
            this.rangeIn = Math.hypot(x, y);
            this.tagId = tagId; this.tagCount = tagCount; this.ageMs = ageMs;
        }
        static final Target NONE = new Target(false, 0, 0, 0, -1, 0, Long.MAX_VALUE);
    }

    // ---- Configuration: measure these on the real robot --------------------------------------
    /** Camera lens position in robot frame, inches. x forward, y left, z up from floor. */
    public static double CAM_X_IN = 6.0, CAM_Y_IN = 0.0, CAM_Z_IN = 12.0;
    /** Camera pitch above horizontal, degrees (positive = looking up). Yaw relative to robot forward, +left. */
    public static double CAM_PITCH_DEG = 30.0, CAM_YAW_DEG = 0.0;
    public static int PIPELINE_TAGS = 1;
    public static long MAX_AGE_MS = 150;

    // Red CELL tags 0-7. Blue audience-side 38-41; add the other blue cluster from manual Fig 9-17.
    static final int[] RED_TAGS = {0, 1, 2, 3, 4, 5, 6, 7};
    static final int[] BLUE_TAGS = {38, 39, 40, 41};

    // ---- State --------------------------------------------------------------------------------
    private Limelight3A limelight;
    private Alliance alliance = Alliance.RED;
    private volatile Target target = Target.NONE;
    private LLResult lastResult;

    public void init(HardwareMap hw, Alliance a) {
        limelight = hw.get(Limelight3A.class, "limelight");
        alliance = a;
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE_TAGS);
    }

    public void start() { limelight.start(); }
    public void stop()  { limelight.stop(); }
    public void setAlliance(Alliance a) { alliance = a; }
    public Alliance getAlliance() { return alliance; }
    public Target getTarget() { return target; }
    public LLResult getRawResult() { return lastResult; }

    /** Call once per control loop. Cheap: one poll of the Limelight's cached result. */
    public void update() {
        LLResult r = limelight.getLatestResult();
        lastResult = r;
        if (r == null || !r.isValid() || r.getPipelineIndex() != PIPELINE_TAGS || r.getStaleness() > MAX_AGE_MS) {
            target = Target.NONE;
            return;
        }
        List<FiducialResult> tags = r.getFiducialResults();
        FiducialResult best = null;
        int ours = 0;
        for (FiducialResult t : tags) {
            if (!isOurs(t.getFiducialId())) continue;
            ours++;
            // Prefer the largest tag: closest and best-conditioned pose solve.
            if (best == null || t.getTargetArea() > best.getTargetArea()) best = t;
        }
        if (best == null) { target = Target.NONE; return; }

        Pose3D p = best.getTargetPoseCameraSpace();
        if (p == null) { target = Target.NONE; return; }

        // Limelight camera space: +x right, +y down, +z forward, meters.
        double cxR = p.getPosition().x * 39.37;
        double cyD = p.getPosition().y * 39.37;
        double czF = p.getPosition().z * 39.37;

        // Camera-aligned frame: forward, left, up.
        double f = czF, l = -cxR, u = -cyD;

        // Undo camera pitch (rotation about the left axis).
        double cp = Math.cos(Math.toRadians(CAM_PITCH_DEG)), sp = Math.sin(Math.toRadians(CAM_PITCH_DEG));
        double f1 = f * cp - u * sp;
        double u1 = f * sp + u * cp;

        // Undo camera yaw (rotation about the up axis).
        double cy = Math.cos(Math.toRadians(CAM_YAW_DEG)), sy = Math.sin(Math.toRadians(CAM_YAW_DEG));
        double x = f1 * cy - l * sy + CAM_X_IN;
        double y = f1 * sy + l * cy + CAM_Y_IN;
        double z = u1 + CAM_Z_IN;

        target = new Target(true, x, y, z, best.getFiducialId(), ours, r.getStaleness());
    }

    private boolean isOurs(int id) {
        for (int t : (alliance == Alliance.RED ? RED_TAGS : BLUE_TAGS)) if (t == id) return true;
        return false;
    }
}
