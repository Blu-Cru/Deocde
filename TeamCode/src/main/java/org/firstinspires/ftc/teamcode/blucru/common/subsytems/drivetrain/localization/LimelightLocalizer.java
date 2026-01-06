package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

/**
 * Limelight 3A wrapper for MegaTag2 AprilTag localization
 *
 * Documentation:
 * - https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
 * - https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
 */
@Config
public class LimelightLocalizer implements RobotLocalizer {

    private Limelight3A limelight;
    private LLResult result;

    // Offset from robot center to Limelight (inches)
    public static double LIMELIGHT_X_OFFSET = 0.0;  // Forward/backward from center
    public static double LIMELIGHT_Y_OFFSET = 0.0;  // Left/right from center

    // Data validity
    private boolean hasValidData = false;
    private double lastValidX = 0;
    private double lastValidY = 0;
    private double lastValidHeading = 0;
    private long lastUpdateTime = 0;

    // Quality threshold
    public static double MIN_TAG_AREA = 0.1;  // Minimum tag area to trust measurement
    public static long MAX_AGE_MS = 500;      // Maximum age of measurement to consider valid

    public LimelightLocalizer(HardwareMap hardwareMap, String limelightName) {
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        limelight.pipelineSwitch(0);  // Switch to MegaTag2 pipeline
        limelight.start();
    }

    @Override
    public void read() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();

            if (botpose_mt2 != null) {
                // Convert from meters to inches and apply offset
                double visionX = botpose_mt2.getPosition().x * 39.3701;  // meters to inches
                double visionY = botpose_mt2.getPosition().y * 39.3701;

                // Get heading from pose (yaw)
                double visionHeading = Math.toRadians(botpose_mt2.getOrientation().getYaw());

                // Apply camera offset (transform from camera pose to robot center pose)
                double cos = Math.cos(visionHeading);
                double sin = Math.sin(visionHeading);
                lastValidX = visionX - (LIMELIGHT_X_OFFSET * cos - LIMELIGHT_Y_OFFSET * sin);
                lastValidY = visionY - (LIMELIGHT_X_OFFSET * sin + LIMELIGHT_Y_OFFSET * cos);
                lastValidHeading = visionHeading;

                lastUpdateTime = System.currentTimeMillis();
                hasValidData = true;
            } else {
                hasValidData = false;
            }
        } else {
            hasValidData = false;
        }
    }

    /**
     * Update robot orientation for MegaTag2 fusion
     * Call this before read() to provide IMU data to Limelight
     */
    public void updateRobotOrientation(double headingRadians) {
        limelight.updateRobotOrientation(Math.toDegrees(headingRadians));
    }

    /**
     * Check if we have recent valid vision data
     */
    public boolean hasValidData() {
        long age = System.currentTimeMillis() - lastUpdateTime;
        return hasValidData && age < MAX_AGE_MS;
    }

    /**
     * Get the number of AprilTags currently detected
     */
    public int getNumTagsDetected() {
        if (result != null && result.isValid()) {
            return (int) result.getTa();  // Tag area as proxy for detection
        }
        return 0;
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(lastValidX, lastValidY, lastValidHeading);
    }

    @Override
    public double getX() {
        return lastValidX;
    }

    @Override
    public double getY() {
        return lastValidY;
    }

    @Override
    public double getHeading() {
        return lastValidHeading;
    }

    @Override
    public void setPosition(Pose2d pose) {
        // Limelight doesn't support setting position - it only provides measurements
        lastValidX = pose.getX();
        lastValidY = pose.getY();
        lastValidHeading = pose.getH();
    }

    @Override
    public void setPosition(double x, double y, double h) {
        lastValidX = x;
        lastValidY = y;
        lastValidHeading = h;
    }

    @Override
    public Pose2d getVel() {
        // Limelight doesn't provide velocity
        return new Pose2d(0, 0, 0);
    }

    @Override
    public void setHeading(double heading) {
        lastValidHeading = heading;
    }

    @Override
    public void setOffset(double x, double y, double h) {
        LIMELIGHT_X_OFFSET = x;
        LIMELIGHT_Y_OFFSET = y;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Limelight Valid", hasValidData());
        telemetry.addData("Limelight X", "%.2f", lastValidX);
        telemetry.addData("Limelight Y", "%.2f", lastValidY);
        telemetry.addData("Limelight Heading", "%.2f deg", Math.toDegrees(lastValidHeading));
        telemetry.addData("Tags Detected", getNumTagsDetected());

        long age = System.currentTimeMillis() - lastUpdateTime;
        telemetry.addData("Measurement Age", age + " ms");
    }
}
