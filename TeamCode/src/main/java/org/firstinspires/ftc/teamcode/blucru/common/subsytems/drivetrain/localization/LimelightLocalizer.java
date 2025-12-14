package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
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
        // Update robot orientation for MegaTag 2 if IMU localizer is available
        if (imuLocalizer != null) {
            // Update IMU data to ensure it's fresh
            imuLocalizer.read();
            
            // Provide current heading to Limelight for MegaTag 2 fusion
            // MegaTag 2 uses this IMU data to eliminate pose ambiguity
            double currentHeading = imuLocalizer.getHeading();
            limelight.updateRobotOrientation(Math.toDegrees(currentHeading));
        }
        
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose3D;
            
            // Use MegaTag 2 if IMU localizer is available, otherwise use standard localization
            if (imuLocalizer != null) {
                // MegaTag 2: More accurate, eliminates ambiguity by fusing IMU + AprilTags
                botPose3D = result.getBotpose_MT2();
            } else {
                // Standard MegaTag 1: Basic AprilTag localization
                botPose3D = result.getBotpose();
            }
            
            if (botPose3D != null) {
                // Convert Pose3D to Pose2d
                // Limelight returns position in meters, convert to inches for consistency
                // with other localizers (Pinpoint uses inches)
                double xInches = botPose3D.getPosition().x * 39.3701;
                double yInches = botPose3D.getPosition().y * 39.3701;
                
                // Get heading in radians
                double heading = botPose3D.getOrientation().getYaw(AngleUnit.RADIANS);

                // Update pose
                Pose2d newPose = new Pose2d(xInches, yInches, heading);

                // Calculate velocity
                long currentTime = System.nanoTime();
                double dt = (currentTime - lastTime) / 1e9;
                if (dt > 0) {
                    double vx = (newPose.getX() - pose.getX()) / dt;
                    double vy = (newPose.getY() - pose.getY()) / dt;
                    double vh = (newPose.getH() - pose.getH()) / dt;
                    vel = new Pose2d(vx, vy, vh);
                }

                pose = newPose;
                lastTime = currentTime;
            }
        }
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
        pose = new Pose2d(x, y, h);
    }

    @Override
    public void setPosition(Pose2d pose) {
        this.pose = pose;
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
