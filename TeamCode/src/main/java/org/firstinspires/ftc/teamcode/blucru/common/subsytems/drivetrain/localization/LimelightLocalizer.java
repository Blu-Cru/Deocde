package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.KalmanFilter;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

/**
 * LimelightLocalizer using MegaTag 2 (MT2) for improved localization accuracy.
 * 
 * MegaTag 2 Requirements:
 * 1. AprilTag pipeline must have "Full 3D" enabled in Limelight web interface
 * 2. Camera pose (position/orientation relative to robot center) configured in web interface
 * 3. Field map (.fmap file) uploaded to Limelight with AprilTag locations
 * 4. IMU data must be provided to Limelight via updateRobotOrientation()
 */
public class LimelightLocalizer implements RobotLocalizer {
    Limelight3A limelight;
    RobotLocalizer imuLocalizer; // For providing heading data to MegaTag 2
    Pose2d pose;
    Pose2d vel;
    long lastTime;

    // Kalman Filters for smoothing
    KalmanFilter xFilter;
    KalmanFilter yFilter;
    KalmanFilter hFilter;

    // Filter Parameters (Tune these!)
    // Q: Process noise covariance (trust in model/prediction). Higher = faster response, less smoothing.
    // R: Measurement noise covariance (trust in sensor). Higher = smoother (less noise), slower response.
    // Recommended starting values: Q=0.1, R=0.3
    public static double Q_X = 0.1, R_X = 0.3;
    public static double Q_Y = 0.1, R_Y = 0.3;
    public static double Q_H = 0.1, R_H = 0.1;

    /**
     * Constructor for LimelightLocalizer with MegaTag 2 support
     * @param name Hardware map name for the Limelight
     * @param imuLocalizer Another localizer (e.g., Pinpoint) that provides IMU heading data
     */
    public LimelightLocalizer(String name, RobotLocalizer imuLocalizer) {
        limelight = Globals.hwMap.get(Limelight3A.class, name);
        this.imuLocalizer = imuLocalizer;
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0); // Assuming pipeline 0 is set up for AprilTags/MegaTag

        pose = new Pose2d(0, 0, 0);
        vel = new Pose2d(0, 0, 0);
        lastTime = System.nanoTime();

        // Initialize Kalman Filters
        xFilter = new KalmanFilter(Q_X, R_X);
        yFilter = new KalmanFilter(Q_Y, R_Y);
        hFilter = new KalmanFilter(Q_H, R_H);
    }
    
    /**
     * Legacy constructor for backward compatibility (uses standard localization)
     * @param name Hardware map name for the Limelight
     */
    public LimelightLocalizer(String name) {
        this(name, null);
    }

    @Override
    public void read() {
        // 1. Always read from Pinpoint/IMU first (Continuous Odometry)
        if (imuLocalizer != null) {
            imuLocalizer.read();
        }
        
        LLResult result = limelight.getLatestResult();
        
        // 2. Check for Vision Data (Drift Correction)
        if (result != null && result.isValid()) {
            Pose3D botPose3D;
            
            // MegaTag 2 Fusion (if IMU available) or Standard
            if (imuLocalizer != null) {
                // Provide heading for MT2
                limelight.updateRobotOrientation(Math.toDegrees(imuLocalizer.getHeading()));
                botPose3D = result.getBotpose_MT2();
            } else {
                botPose3D = result.getBotpose();
            }
            
            if (botPose3D != null) {
                // Convert to inches
                double xInches = botPose3D.getPosition().x * 39.3701;
                double yInches = botPose3D.getPosition().y * 39.3701;
                double heading = botPose3D.getOrientation().getYaw(AngleUnit.RADIANS);

                // Filter the Vision Data
                double filteredX = xFilter.update(xInches);
                double filteredY = yFilter.update(yInches);
                double filteredH = hFilter.update(heading);

                // Create new corrected pose
                Pose2d correctedPose = new Pose2d(filteredX, filteredY, filteredH);

                // Update our output pose
                pose = correctedPose;
                
                // CRITICAL: Sync Pinpoint with Vision Correction
                // This resets the odometry to match the known field position
                if (imuLocalizer != null) {
                    imuLocalizer.setPosition(correctedPose);
                }
            }
        } 
        // 3. NO Vision Data (Fallback to Odometry)
        else {
            if (imuLocalizer != null) {
                // Use Pinpoint's dead reckoning
                pose = imuLocalizer.getPose();
                
                // Keep filters synced so they don't "jump" when vision returns
                xFilter.setEstimate(pose.getX());
                yFilter.setEstimate(pose.getY());
                hFilter.setEstimate(pose.getH());
            }
            // If no IMU/Pinpoint and no Vision, pose remains frozen (dangerous but unavoidable)
        }

        // Calculate velocity (simple derivative)
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        if (dt > 0) {
            // ... (velocity calc simplified for brevity if needed, or keep existing)
             double vx = (pose.getX() - xFilter.getEstimate()) / dt; // roughly
             // Better: we can get velocity from Pinpoint directly if available
             if (imuLocalizer != null) {
                 vel = imuLocalizer.getVel();
             }
        }
        lastTime = currentTime;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public double getX() {
        return pose.getX();
    }

    @Override
    public double getY() {
        return pose.getY();
    }

    @Override
    public double getHeading() {
        return pose.getH();
    }

    @Override
    public void setOffset(double x, double y, double h) {
        // Limelight handles its own offsets mostly, but we could offset our result
        // For now, implementing as empty or storing offset locally if needed.
        // Interface requires it.
    }

    @Override
    public void setPosition(double x, double y, double h) {
        // Hard to "set" position on Vision sensor without updating the map or offset.
        // For simple localization, we might override the current logic, but vision
        // overwrites it on next frame.
        // overwrites it on next frame.
        pose = new Pose2d(x, y, h);
        xFilter.setEstimate(x);
        yFilter.setEstimate(y);
        hFilter.setEstimate(h);
    }

    @Override
    public void setPosition(Pose2d pose) {
        this.pose = pose;
        xFilter.setEstimate(pose.getX());
        yFilter.setEstimate(pose.getY());
        hFilter.setEstimate(pose.getH());
    }

    @Override
    public Pose2d getVel() {
        return vel;
    }

    @Override
    public void setHeading(double heading) {
        // pose = new Pose2d(pose.getX(), pose.getY(), heading);
        // Note: Vision will overwrite this.
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        String mode = (imuLocalizer != null) ? "MegaTag 2 (MT2)" : "MegaTag 1";
        telemetry.addData("Limelight Mode", mode);
        telemetry.addData("Limelight Pose", "X: %.2f, Y: %.2f, H: %.2f", pose.getX(), pose.getY(),
                Math.toDegrees(pose.getH()));
        telemetry.addData("Limelight Vel", "X: %.2f, Y: %.2f, H: %.2f", vel.getX(), vel.getY(),
                Math.toDegrees(vel.getH()));
    }
}
