package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

/**
 * Displays real-time telemetry of:
 * - Each detected ball's angle, area, and distance from robot center
 * - The smoothed cluster centroid and its distance from the robot
 * - Raw vs smoothed values so you can see the EMA working
 * - Persistence counter showing how the detector holds through flicker
 */
@TeleOp(name = "Limelight Ball Detector Test", group = "Test")
public class LimelightBallDetectorTest extends BluLinearOpMode {

    @Override
    public void initialize() {
        addSixWheel();
        addBallDetector();

        // Configure camera mounting geometry (adjust these to match your robot!)
        // xOffset = forward from robot center (positive = front), inches
        // yOffset = lateral from robot center (positive = left), inches
        // zHeight = camera lens height above ground, inches
        // mountAngle = degrees below horizontal (90 = straight down)
        ballDetector.setCameraParameters(
                11.3, // xOffset: camera forward from center
                6.5, // yOffset: camera left from center
                13.0, // zHeight: camera above ground
                15.0 // mountAngle: 15° below horizontal
        );

        sixWheel.reset();
    }

    @Override
    public void onStart() {
        sixWheel.setPosition(new Pose2d(0, 0, 0));
    }

    @Override
    public void periodic() {
        ballDetector.read();

        // ---- Header ----
        telemetry.addLine("=== LIMELIGHT BALL DETECTOR ===");
        telemetry.addData("Pipeline", ballDetector.getPipelineIndex());
        telemetry.addLine("");

        // ---- Detection summary ----
        telemetry.addData("Total Balls Detected", ballDetector.getTotalBallsDetected());
        telemetry.addData("Has Valid Clump", ballDetector.hasValidClump());
        telemetry.addLine("");

        // ---- Smoothed clump info ----
        if (ballDetector.hasValidClump()) {
            telemetry.addLine("--- CLUMP (smoothed) ---");
            telemetry.addData("Ball Count", ballDetector.getClumpBallCount());
            telemetry.addData("Smooth TX / TY (deg)",
                    String.format("%.2f / %.2f",
                            ballDetector.getClumpTxDeg(),
                            ballDetector.getClumpTyDeg()));
            telemetry.addData("Clump Robot X (in)",
                    String.format("%.2f (forward)", ballDetector.getClumpRobotX()));
            telemetry.addData("Clump Robot Y (in)",
                    String.format("%.2f (left)", ballDetector.getClumpRobotY()));
            telemetry.addData("Clump Dist from Robot (in)",
                    String.format("%.2f", ballDetector.getClumpDistanceFromRobot()));
            telemetry.addData("Clump Field X (in)",
                    String.format("%.2f", ballDetector.getClumpFieldX()));
        }

        // ---- Individual ball detections with distances ----
        telemetry.addLine("");
        ballDetector.telemetry(telemetry);
    }
}
