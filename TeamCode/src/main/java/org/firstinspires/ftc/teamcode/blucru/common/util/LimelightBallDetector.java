package org.firstinspires.ftc.teamcode.blucru.common.util;

import android.util.Log;

import com.seattlesolvers.solverslib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

import java.util.ArrayList;
import java.util.List;

/**
 * Uses a Limelight neural network pipeline to detect balls on the field,
 * clusters nearby detections to find the densest clump, and outputs
 * the field-relative X position of that clump for autonomous path building.
 */
public class LimelightBallDetector implements BluSubsystem, Subsystem {

    // ===============================
    // CONFIGURATION
    // ===============================

    /** Limelight pipeline index for the neural network ball detector. */
    private static final int BALL_DETECTOR_PIPELINE = 2;

    /**
     * Clustering threshold in degrees. Two detections within this angular
     * distance of each other are considered part of the same cluster.
     */
    private static final double CLUSTER_THRESHOLD_DEG = 8.0;

    /**
     * Minimum confidence to accept a detection (0-1). Adjust based on your
     * model's behavior.
     */
    private static final double MIN_CONFIDENCE = 0.5;

    /**
     * Exponential moving average smoothing factor (0-1).
     * Lower = smoother but more lag. Higher = more responsive but jittery.
     * 0.3 is a good balance for flickering NN detections.
     */
    private static final double EMA_ALPHA = 0.3;

    /**
     * Number of frames to persist the last valid clump position when
     * detection drops out. Prevents flickering from killing the signal.
     */
    private static final int PERSISTENCE_FRAMES = 10;

    /**
     * Camera X offset from robot center in inches.
     * Positive = forward (toward the front of the robot).
     */
    private double cameraXOffsetInches = 11.2992;

    /**
     * Camera Y offset from robot center in inches.
     * Positive = left (following standard FTC coordinate convention).
     */
    private double cameraYOffsetInches = 6.49606;

    /**
     * Camera Z offset (height) above the ground in inches.
     * This is the vertical distance from the ground to the camera lens.
     */
    private double cameraZHeightInches = 13.0315;

    /**
     * Camera mounting angle in degrees below horizontal (positive = looking down).
     * If your camera is pointing straight down, this would be 90.
     */
    private double cameraMountAngleDeg = 15;

    // ===============================
    // STATE
    // ===============================
    private Limelight3A limelight;
    private boolean hasValidClump = false;

    /** Raw (unsmoothed) clump centroid in camera-frame degrees */
    private double clumpTxDeg = 0;
    private double clumpTyDeg = 0;

    /** Smoothed (EMA) clump centroid in camera-frame degrees */
    private double smoothedTxDeg = 0;
    private double smoothedTyDeg = 0;
    private boolean hasSmoothedData = false;

    /** Smoothed clump centroid converted to field-relative X position (inches) */
    private double clumpFieldX = 0;

    /** Number of balls in the biggest cluster */
    private int clumpBallCount = 0;

    /** Total number of balls detected this frame */
    private int totalBallsDetected = 0;

    /** Frames since last valid detection (for persistence) */
    private int framesSinceLastDetection = 0;

    /**
     * Raw list of detected ball positions (tx, ty, area) from this frame for
     * telemetry.
     */
    private final List<double[]> rawDetections = new ArrayList<>();

    /**
     * Per-ball distances from the robot center (inches), computed each frame.
     * Index matches rawDetections.
     */
    private final List<Double> ballDistances = new ArrayList<>();

    // ===============================
    // CONSTRUCTOR
    // ===============================

    /**
     * Creates a new LimelightBallDetector using the default "limelight" hardware
     * name.
     * Call setCameraParameters() after construction to configure camera geometry.
     */
    public LimelightBallDetector() {
        limelight = Globals.hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(BALL_DETECTOR_PIPELINE);
    }

    /**
     * Configure the camera's physical mounting position and angle.
     * All offsets are relative to the robot's center point on the ground.
     *
     * @param xOffsetInches forward offset from robot center (positive = front of
     *                      robot)
     * @param yOffsetInches lateral offset from robot center (positive = left side)
     * @param zHeightInches height of the camera lens above the ground
     * @param mountAngleDeg angle below horizontal that the camera points
     *                      (e.g. 45 for 45° down, 90 for straight down)
     */
    public void setCameraParameters(double xOffsetInches, double yOffsetInches,
            double zHeightInches, double mountAngleDeg) {
        this.cameraXOffsetInches = xOffsetInches;
        this.cameraYOffsetInches = yOffsetInches;
        this.cameraZHeightInches = zHeightInches;
        this.cameraMountAngleDeg = mountAngleDeg;
    }

    // ===============================
    // LIFECYCLE (BluSubsystem)
    // ===============================

    @Override
    public void init() {
        // Pipeline is already switched in constructor
    }

    @Override
    public void read() {
        rawDetections.clear();
        ballDistances.clear();
        totalBallsDetected = 0;
        clumpBallCount = 0;

        boolean gotDetectionThisFrame = false;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            // Collect valid detections
            List<double[]> ballPositions = new ArrayList<>();

            if (detections != null && !detections.isEmpty()) {
                // NN pipeline populates detector results list
                for (LLResultTypes.DetectorResult dr : detections) {
                    double tx = dr.getTargetXDegrees();
                    double ty = dr.getTargetYDegrees();
                    double area = dr.getTargetArea();

                    ballPositions.add(new double[] { tx, ty, area });
                    rawDetections.add(new double[] { tx, ty, area });
                    ballDistances.add(computeBallDistanceFromRobot(tx, ty));
                }
            } else if (result.getTa() > 0) {
                // Fallback: some pipelines only populate top-level tx/ty/ta
                double tx = result.getTx();
                double ty = result.getTy();
                double area = result.getTa();

                ballPositions.add(new double[] { tx, ty, area });
                rawDetections.add(new double[] { tx, ty, area });
                ballDistances.add(computeBallDistanceFromRobot(tx, ty));
            }

            totalBallsDetected = ballPositions.size();

            if (totalBallsDetected > 0) {
                // ------- Clustering -------
                List<List<double[]>> clusters = new ArrayList<>();

                for (double[] ball : ballPositions) {
                    boolean added = false;
                    for (List<double[]> cluster : clusters) {
                        double[] rep = cluster.get(0);
                        double dist = Math.sqrt(
                                Math.pow(ball[0] - rep[0], 2) + Math.pow(ball[1] - rep[1], 2));
                        if (dist < CLUSTER_THRESHOLD_DEG) {
                            cluster.add(ball);
                            added = true;
                            break;
                        }
                    }
                    if (!added) {
                        List<double[]> newCluster = new ArrayList<>();
                        newCluster.add(ball);
                        clusters.add(newCluster);
                    }
                }

                // Find the largest cluster
                List<double[]> biggestCluster = null;
                for (List<double[]> cluster : clusters) {
                    if (biggestCluster == null || cluster.size() > biggestCluster.size()) {
                        biggestCluster = cluster;
                    }
                }

                if (biggestCluster != null && !biggestCluster.isEmpty()) {
                    clumpBallCount = biggestCluster.size();

                    // Compute raw center of the biggest cluster
                    double sumTx = 0, sumTy = 0;
                    for (double[] ball : biggestCluster) {
                        sumTx += ball[0];
                        sumTy += ball[1];
                    }
                    clumpTxDeg = sumTx / clumpBallCount;
                    clumpTyDeg = sumTy / clumpBallCount;

                    // ------- EMA Smoothing -------
                    if (!hasSmoothedData) {
                        // First valid reading: seed the EMA
                        smoothedTxDeg = clumpTxDeg;
                        smoothedTyDeg = clumpTyDeg;
                        hasSmoothedData = true;
                    } else {
                        smoothedTxDeg = EMA_ALPHA * clumpTxDeg + (1.0 - EMA_ALPHA) * smoothedTxDeg;
                        smoothedTyDeg = EMA_ALPHA * clumpTyDeg + (1.0 - EMA_ALPHA) * smoothedTyDeg;
                    }

                    framesSinceLastDetection = 0;
                    gotDetectionThisFrame = true;
                }
            }
        }

        // ------- Persistence: hold last smoothed value through flicker -------
        if (!gotDetectionThisFrame) {
            framesSinceLastDetection++;
        }

        // We have a valid clump if we got one this frame OR we're within
        // the persistence window of the last valid detection
        hasValidClump = hasSmoothedData && framesSinceLastDetection <= PERSISTENCE_FRAMES;

        if (hasValidClump) {
            // Use smoothed values for field position computation
            computeFieldPosition();
        }

        Log.d("BallDetector", String.format(
                "det=%d clump=%d raw=(%.1f,%.1f) smooth=(%.1f,%.1f) fieldX=%.1f persist=%d valid=%b",
                totalBallsDetected, clumpBallCount, clumpTxDeg, clumpTyDeg,
                smoothedTxDeg, smoothedTyDeg, clumpFieldX,
                framesSinceLastDetection, hasValidClump));
    }

    /**
     * Computes the straight-line distance (inches) from the robot center
     * to a single ball detected at the given camera angles.
     */
    private double computeBallDistanceFromRobot(double txDeg, double tyDeg) {
        double tyRad = Math.toRadians(cameraMountAngleDeg + tyDeg);
        double txRad = Math.toRadians(txDeg);

        // Ground distance from under the camera to the ball
        double camForward = cameraZHeightInches / Math.tan(tyRad);
        double camLateral = camForward * Math.tan(txRad);

        // Shift by camera offset to get distance from robot center
        double robotX = cameraXOffsetInches + camForward;
        double robotY = cameraYOffsetInches + camLateral;

        return Math.sqrt(robotX * robotX + robotY * robotY);
    }

    /**
     * Converts the clump centroid (in camera degree-space) to a field-relative X
     * coordinate, accounting for the camera's XYZ offset from the robot center.
     *
     * Steps:
     * 1. Use ty + mount angle to compute how far forward (from the camera) the
     * ball is on the ground. Use tx to compute lateral offset from camera axis.
     * 2. Add the camera's XY offset on the robot to get the ball position
     * relative to the robot center (in robot-frame coordinates).
     * 3. Rotate by the robot's heading and add the robot's field pose to get
     * the ball's field-relative position.
     */
    private void computeFieldPosition() {
        Pose2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos();
        if (robotPose == null) {
            // Fallback: use tx directly as a relative offset (won't be field-accurate)
            clumpFieldX = clumpTxDeg;
            return;
        }

        double heading = robotPose.getH(); // Robot heading in radians

        // --- Step 1: Ball position relative to the camera (using smoothed values) ---
        double tyRad = Math.toRadians(cameraMountAngleDeg + smoothedTyDeg);
        double txRad = Math.toRadians(smoothedTxDeg);

        // Ground distance forward from directly under the camera to the ball
        double camForwardDist = cameraZHeightInches / Math.tan(tyRad);

        // Lateral offset from the camera's forward axis
        double camLateralDist = camForwardDist * Math.tan(txRad);

        // --- Step 2: Ball position relative to the robot center ---
        // Camera is at (cameraXOffset, cameraYOffset) in robot-frame coords.
        // Ball relative to camera is (camForwardDist, camLateralDist).
        // Ball relative to robot center:
        double ballRobotX = cameraXOffsetInches + camForwardDist;
        double ballRobotY = cameraYOffsetInches + camLateralDist;

        // --- Step 3: Rotate to field frame and add robot position ---
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double fieldDeltaX = ballRobotX * cosH - ballRobotY * sinH;
        double fieldDeltaY = ballRobotX * sinH + ballRobotY * cosH;

        clumpFieldX = robotPose.getX() + fieldDeltaX;
    }

    @Override
    public void write() {
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("[Ball] Total Detected", totalBallsDetected);
        telemetry.addData("[Ball] Clump Size", clumpBallCount);
        telemetry.addData("[Ball] Has Valid Clump", hasValidClump);
        telemetry.addData("[Ball] Persistence", framesSinceLastDetection);
        if (hasValidClump) {
            telemetry.addData("[Ball] Raw TX/TY",
                    String.format("%.1f / %.1f", clumpTxDeg, clumpTyDeg));
            telemetry.addData("[Ball] Smooth TX/TY",
                    String.format("%.1f / %.1f", smoothedTxDeg, smoothedTyDeg));
            telemetry.addData("[Ball] Clump Field X", String.format("%.1f", clumpFieldX));
        }
        for (int i = 0; i < rawDetections.size(); i++) {
            double[] d = rawDetections.get(i);
            double dist = (i < ballDistances.size()) ? ballDistances.get(i) : -1;
            telemetry.addData("[Ball] Det " + i,
                    String.format("tx=%.1f ty=%.1f area=%.2f dist=%.1fin",
                            d[0], d[1], d[2], dist));
        }
    }

    @Override
    public void reset() {
        hasValidClump = false;
        hasSmoothedData = false;
        framesSinceLastDetection = 0;
        rawDetections.clear();
        ballDistances.clear();
    }

    // ===============================
    // PUBLIC API
    // ===============================

    /** Returns true if a valid clump of balls was detected this frame. */
    public boolean hasValidClump() {
        return hasValidClump;
    }

    /**
     * Returns the field-relative X coordinate of the ball clump centroid (inches).
     */
    public double getClumpFieldX() {
        return clumpFieldX;
    }

    /**
     * Returns the smoothed clump centroid horizontal angle from camera center
     * (degrees).
     */
    public double getClumpTxDeg() {
        return smoothedTxDeg;
    }

    /**
     * Returns the smoothed clump centroid vertical angle from camera center
     * (degrees).
     */
    public double getClumpTyDeg() {
        return smoothedTyDeg;
    }

    /** Returns the raw (unsmoothed) clump TX this frame. */
    public double getRawClumpTxDeg() {
        return clumpTxDeg;
    }

    /** Returns the raw (unsmoothed) clump TY this frame. */
    public double getRawClumpTyDeg() {
        return clumpTyDeg;
    }

    /**
     * Returns the distance (inches) from the robot center to the smoothed
     * clump centroid position on the ground.
     */
    public double getClumpDistanceFromRobot() {
        return computeBallDistanceFromRobot(smoothedTxDeg, smoothedTyDeg);
    }

    /**
     * Returns the distance (inches) from the robot center to a specific
     * detected ball by index. Returns -1 if index is out of range.
     */
    public double getBallDistance(int index) {
        if (index >= 0 && index < ballDistances.size()) {
            return ballDistances.get(index);
        }
        return -1;
    }

    /** Returns the number of balls in the biggest detected cluster. */
    public int getClumpBallCount() {
        return clumpBallCount;
    }

    /** Returns the total number of balls detected this frame. */
    public int getTotalBallsDetected() {
        return totalBallsDetected;
    }

    /** The pipeline index used by this detector. */
    public int getPipelineIndex() {
        return BALL_DETECTOR_PIPELINE;
    }

    /** Switches the limelight to this detector's pipeline (useful if shared). */
    public void activate() {
        limelight.pipelineSwitch(BALL_DETECTOR_PIPELINE);
    }

    /**
     * Call if you want to release the limelight for another pipeline
     *
     */
    public void stop() {
        limelight.stop();
    }
}
