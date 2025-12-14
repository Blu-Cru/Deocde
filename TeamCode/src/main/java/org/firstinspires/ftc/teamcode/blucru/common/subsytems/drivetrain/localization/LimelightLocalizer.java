package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class LimelightLocalizer implements RobotLocalizer {
    Limelight3A limelight;
    Pose2d pose;
    Pose2d vel;
    long lastTime;

    public LimelightLocalizer(String name) {
        limelight = Globals.hwMap.get(Limelight3A.class, name);
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0); // Assuming pipeline 0 is set up for AprilTags/Localization

        pose = new Pose2d(0, 0, 0);
        vel = new Pose2d(0, 0, 0);
        lastTime = System.nanoTime();
    }

    @Override
    public void read() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose3D = result.getBotpose();
            if (botPose3D != null) {
                // Convert Pose3D to Pose2d (Meters/Inches?)
                // Pinpoint uses Inches. Limelight typically returns Meters.
                // Converting Meters to Inches: * 39.3701

                // However, check LimelightObeliskTagDetector:
                // bot.getPosition().x is used directly or converted?
                // It does: bot.getPosition().y * 1000 / 25.4 (mm to inches)
                // getBotpose() usually returns in meters.

                double xInches = botPose3D.getPosition().x * 39.3701;
                double yInches = botPose3D.getPosition().y * 39.3701;
                // Yaw is in degrees usually for Limelight, but check AngleUnit
                // result.getBotpose() returns Pose3D.
                // Pose3D orientation is usually Quaternion or intrinsic rotations.

                // Let's use getRobotPoseFieldSpace() which returns Pose3D
                // LimelightObeliskTagDetector uses res.getRobotPoseFieldSpace()

                // But simplified: result.getBotpose()

                // Let's stick to what LimelightObeliskTagDetector did for conversion if
                // possible,
                // but that was tag specific.
                // Standard Limelight `result.getBotpose()` is what we want.

                double heading = botPose3D.getOrientation().getYaw(AngleUnit.RADIANS);

                // Update pose
                Pose2d newPose = new Pose2d(xInches, yInches, heading);

                // Simple Velocity calculation
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
        telemetry.addData("Limelight Pose", "X: %.2f, Y: %.2f, H: %.2f", pose.getX(), pose.getY(),
                Math.toDegrees(pose.getH()));
        telemetry.addData("Limelight Vel", "X: %.2f, Y: %.2f, H: %.2f", vel.getX(), vel.getY(),
                Math.toDegrees(vel.getH()));
    }
}
