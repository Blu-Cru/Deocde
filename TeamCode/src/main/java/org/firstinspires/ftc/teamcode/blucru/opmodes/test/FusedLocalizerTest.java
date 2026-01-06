package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.FusedLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

/**
 * Test opmode for verifying sensor fusion setup
 *
 * Instructions:
 * 1. Place robot at known starting position (e.g., 0, 0, 0)
 * 2. Run this opmode
 * 3. Drive robot using gamepad
 * 4. Observe telemetry:
 *    - Compare Fused vs Odometry vs Vision estimates
 *    - Check "Vision Active" status
 *    - Monitor position uncertainty
 * 5. Drive near AprilTags and observe corrections
 * 6. Tune EKF parameters via FTC Dashboard if needed
 *
 * Success criteria:
 * - Vision Active = true when tags visible
 * - Position uncertainty <2 inches with tags visible
 * - Smooth corrections (no jumping)
 * - Vision update rate >20% in tag-rich areas
 */
@TeleOp(name = "Fused Localizer Test", group = "Test")
public class FusedLocalizerTest extends OpMode {

    private FusedLocalizer localizer;

    @Override
    public void init() {
        // Setup telemetry
        Globals.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize fused localizer
        localizer = new FusedLocalizer(
                hardwareMap,
                "pinpoint",   // Pinpoint device name
                "limelight"   // Limelight device name
        );

        // Set starting position (adjust as needed)
        localizer.setPosition(new Pose2d(0, 0, 0));

        telemetry.addLine("Fused Localizer Test Ready");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick: Drive");
        telemetry.addLine("  Right Stick X: Turn");
        telemetry.addLine("  A: Reset position to (0, 0, 0)");
        telemetry.addLine("  B: Toggle vision correction");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update localizer
        localizer.read();

        // Simple drive controls (for testing)
        double forward = -gamepad1.left_stick_y * 0.5;
        double turn = gamepad1.right_stick_x * 0.3;

        // Apply drive (if motors connected)
        // drivetrain.drive(forward, turn);

        // Reset position
        if (gamepad1.a) {
            localizer.setPosition(new Pose2d(0, 0, 0));
            telemetry.addLine("Position reset!");
        }

        // Toggle vision
        if (gamepad1.b) {
            FusedLocalizer.USE_VISION_CORRECTION = !FusedLocalizer.USE_VISION_CORRECTION;
        }

        // Display comprehensive telemetry
        displayTelemetry();

        telemetry.update();
    }

    private void displayTelemetry() {
        telemetry.addLine("========== SENSOR FUSION TEST ==========");
        telemetry.addLine();

        // Current fused estimate
        Pose2d fused = localizer.getPose();
        telemetry.addLine("FUSED ESTIMATE:");
        telemetry.addData("  X", "%.2f inches", fused.getX());
        telemetry.addData("  Y", "%.2f inches", fused.getY());
        telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(fused.getH()));
        telemetry.addData("  Position Uncertainty", "%.2f inches", localizer.getPositionUncertainty());
        telemetry.addData("  Heading Uncertainty", "%.2f degrees", Math.toDegrees(localizer.getHeadingUncertainty()));
        telemetry.addLine();

        // Odometry (before fusion)
        Pose2d odometry = localizer.getRawOdometryPose();
        telemetry.addLine("ODOMETRY (Raw):");
        telemetry.addData("  X", "%.2f inches", odometry.getX());
        telemetry.addData("  Y", "%.2f inches", odometry.getY());
        telemetry.addData("  Heading", "%.1f degrees", Math.toDegrees(odometry.getH()));
        telemetry.addLine();

        // Vision status
        telemetry.addLine("VISION STATUS:");
        telemetry.addData("  Active", localizer.isVisionActive() ? "YES" : "NO");
        telemetry.addData("  Update Rate", "%.1f%%", localizer.getVisionUpdateRate());

        if (localizer.isVisionActive()) {
            Pose2d vision = localizer.getRawVisionPose();
            telemetry.addData("  Vision X", "%.2f inches", vision.getX());
            telemetry.addData("  Vision Y", "%.2f inches", vision.getY());
            telemetry.addData("  Vision Heading", "%.1f degrees", Math.toDegrees(vision.getH()));

            // Show difference between odometry and vision
            double errorX = Math.abs(odometry.getX() - vision.getX());
            double errorY = Math.abs(odometry.getY() - vision.getY());
            double errorHeading = Math.abs(Math.toDegrees(odometry.getH() - vision.getH()));

            telemetry.addLine();
            telemetry.addLine("ODOMETRY ERROR:");
            telemetry.addData("  X Error", "%.2f inches", errorX);
            telemetry.addData("  Y Error", "%.2f inches", errorY);
            telemetry.addData("  Heading Error", "%.1f degrees", errorHeading);
        }

        telemetry.addLine();

        // Configuration
        telemetry.addLine("CONFIGURATION:");
        telemetry.addData("  Vision Correction", FusedLocalizer.USE_VISION_CORRECTION ? "ON" : "OFF");
        telemetry.addLine();

        // Instructions
        telemetry.addLine("CONTROLS:");
        telemetry.addData("  A", "Reset position");
        telemetry.addData("  B", "Toggle vision (currently " + (FusedLocalizer.USE_VISION_CORRECTION ? "ON" : "OFF") + ")");

        // Health check
        telemetry.addLine();
        telemetry.addLine("HEALTH CHECK:");
        if (localizer.isVisionActive()) {
            if (localizer.getPositionUncertainty() < 2.0) {
                telemetry.addLine("  ✓ Low uncertainty - good fusion!");
            } else {
                telemetry.addLine("  ⚠ High uncertainty - adjust EKF params");
            }

            if (localizer.getVisionUpdateRate() > 20) {
                telemetry.addLine("  ✓ Good vision update rate");
            } else {
                telemetry.addLine("  ⚠ Low vision updates - check tags visible");
            }
        } else {
            telemetry.addLine("  ⚠ No vision - drive near AprilTags");
        }
    }
}
