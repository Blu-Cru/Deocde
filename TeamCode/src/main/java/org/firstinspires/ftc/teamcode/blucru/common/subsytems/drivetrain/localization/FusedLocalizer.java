package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

/**
 * Fused localizer combining Pinpoint odometry with Limelight MegaTag2 vision
 * using a Kalman Filter for optimal pose estimation
 *
 * Architecture:
 * - Prediction: Uses Pinpoint odometry deltas (high frequency, drifts over time)
 * - Correction: Uses Limelight AprilTag detection (low frequency, absolute position)
 * - Fusion: EKF blends both sources based on their respective uncertainties
 *
 * References:
 * - Limelight MegaTag2: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
 * - Sensor Fusion: https://www.mdpi.com/1424-8220/11/9/8339
 */
@Config
public class FusedLocalizer implements RobotLocalizer {

    private Pinpoint pinpoint;
    private LimelightLocalizer limelight;
    private KalmanFilter kf;

    // Previous odometry reading for delta calculation
    private Pose2d lastOdometryPose;

    // Statistics
    private int updateCount = 0;
    private int visionUpdateCount = 0;

    // Configuration
    public static boolean USE_VISION_CORRECTION = true;
    public static boolean ENABLE_TELEMETRY = true;

    /**
     * Creates a fused localizer with Pinpoint and Limelight
     * @param hardwareMap Hardware map for device access
     * @param pinpointName Name of Pinpoint device in config
     * @param limelightName Name of Limelight device in config
     */
    public FusedLocalizer(HardwareMap hardwareMap, String pinpointName, String limelightName) {
        // Initialize sensors
        pinpoint = new Pinpoint(pinpointName);
        limelight = new LimelightLocalizer(hardwareMap, limelightName);

        // Initialize Kalman Filter with starting pose
        Pose2d startPose = pinpoint.getPose();
        kf = new KalmanFilter(startPose.getX(), startPose.getY(), startPose.getH());

        lastOdometryPose = startPose;
    }

    @Override
    public void read() {
        updateCount++;

        // Read sensors
        pinpoint.read();
        Pose2d currentOdometryPose = pinpoint.getPose();

        // Update Limelight with current heading for MegaTag2 fusion
        limelight.updateRobotOrientation(currentOdometryPose.getH());
        limelight.read();

        // ========== PREDICTION STEP ==========
        // Calculate odometry delta
        double dx = currentOdometryPose.getX() - lastOdometryPose.getX();
        double dy = currentOdometryPose.getY() - lastOdometryPose.getY();
        double dHeading = normalizeAngle(currentOdometryPose.getH() - lastOdometryPose.getH());

        // Predict state using odometry
        kf.predict(dx, dy, dHeading);

        // ========== UPDATE STEP ==========
        // If we have valid vision data, use it to correct the estimate
        if (USE_VISION_CORRECTION && limelight.hasValidData()) {
            Pose2d visionPose = limelight.getPose();
            kf.update(visionPose.getX(), visionPose.getY(), visionPose.getH());
            visionUpdateCount++;
        }

        // Update last odometry pose for next delta
        lastOdometryPose = currentOdometryPose;
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(kf.getX(), kf.getY(), kf.getHeading());
    }

    @Override
    public double getX() {
        return kf.getX();
    }

    @Override
    public double getY() {
        return kf.getY();
    }

    @Override
    public double getHeading() {
        return kf.getHeading();
    }

    @Override
    public void setPosition(Pose2d pose) {
        // Reset all sensors and Kalman Filter to new pose
        pinpoint.setPosition(pose);
        limelight.setPosition(pose);
        kf.reset(pose.getX(), pose.getY(), pose.getH());
        lastOdometryPose = pose;
    }

    @Override
    public void setPosition(double x, double y, double h) {
        setPosition(new Pose2d(x, y, h));
    }

    @Override
    public Pose2d getVel() {
        // Use Pinpoint velocity (EKF doesn't estimate velocity)
        return pinpoint.getVel();
    }

    @Override
    public void setHeading(double heading) {
        Pose2d current = getPose();
        setPosition(current.getX(), current.getY(), heading);
    }

    @Override
    public void setOffset(double x, double y, double h) {
        pinpoint.setOffset(x, y, h);
        limelight.setOffset(x, y, h);
    }

    /**
     * Get the raw Pinpoint odometry pose (before fusion)
     */
    public Pose2d getRawOdometryPose() {
        return pinpoint.getPose();
    }

    /**
     * Get the raw Limelight vision pose (when available)
     */
    public Pose2d getRawVisionPose() {
        return limelight.getPose();
    }

    /**
     * Get position uncertainty from Kalman Filter
     */
    public double getPositionUncertainty() {
        return kf.getPositionUncertainty();
    }

    /**
     * Get heading uncertainty from Kalman Filter
     */
    public double getHeadingUncertainty() {
        return kf.getHeadingUncertainty();
    }

    /**
     * Get Kalman gain values (for debugging)
     * Values close to 1 = trusting vision more
     * Values close to 0 = trusting odometry more
     */
    public double getKalmanGainX() {
        return kf.getKalmanGainX();
    }

    public double getKalmanGainY() {
        return kf.getKalmanGainY();
    }

    public double getKalmanGainHeading() {
        return kf.getKalmanGainHeading();
    }

    /**
     * Check if vision correction is currently active
     */
    public boolean isVisionActive() {
        return limelight.hasValidData();
    }

    /**
     * Get percentage of updates that used vision correction
     */
    public double getVisionUpdateRate() {
        if (updateCount == 0) return 0;
        return 100.0 * visionUpdateCount / updateCount;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        if (!ENABLE_TELEMETRY) return;

        telemetry.addLine("========== FUSED LOCALIZER ==========");

        // Fused estimate
        telemetry.addData("Fused X", "%.2f in", getX());
        telemetry.addData("Fused Y", "%.2f in", getY());
        telemetry.addData("Fused Heading", "%.2f deg", Math.toDegrees(getHeading()));
        telemetry.addData("Position Uncertainty", "%.2f in", getPositionUncertainty());
        telemetry.addData("Heading Uncertainty", "%.2f deg", Math.toDegrees(getHeadingUncertainty()));

        telemetry.addLine();

        // Raw sources
        Pose2d odometry = getRawOdometryPose();
        telemetry.addData("Odometry X", "%.2f in", odometry.getX());
        telemetry.addData("Odometry Y", "%.2f in", odometry.getY());
        telemetry.addData("Odometry Heading", "%.2f deg", Math.toDegrees(odometry.getH()));

        telemetry.addLine();

        // Vision status
        telemetry.addData("Vision Active", isVisionActive());
        if (isVisionActive()) {
            Pose2d vision = getRawVisionPose();
            telemetry.addData("Vision X", "%.2f in", vision.getX());
            telemetry.addData("Vision Y", "%.2f in", vision.getY());
            telemetry.addData("Vision Heading", "%.2f deg", Math.toDegrees(vision.getH()));
        }
        telemetry.addData("Vision Update Rate", "%.1f%%", getVisionUpdateRate());
        telemetry.addData("Total Updates", updateCount);
        telemetry.addData("Vision Updates", visionUpdateCount);

        telemetry.addLine();

        // Kalman gain (debugging)
        telemetry.addData("Kalman Gain X", "%.2f", getKalmanGainX());
        telemetry.addData("Kalman Gain Y", "%.2f", getKalmanGainY());
        telemetry.addData("Kalman Gain H", "%.2f", getKalmanGainHeading());

        telemetry.addLine();

        // Individual sensor telemetry
        limelight.telemetry(telemetry);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
