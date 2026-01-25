package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.SixWheelDrive;

@TeleOp(name = "PID Tuning OpMode", group = "Tuner")
public class PathingTuningOpMode extends BluLinearOpMode {

    enum TuningMode {
        LINEAR,
        ROTATIONAL
    }

    TuningMode mode = TuningMode.LINEAR;
    boolean manualDriveEnabled = false;

    @Override
    public void initialize() {
        addSixWheel();
        enableDash();
        telemetry.addLine("PID Tuning OpMode");
        telemetry.addLine("Controls:");
        telemetry.addLine("  X: Toggle Mode (Linear <-> Rotational)");
        telemetry.addLine("  A: Execute Test Move");
        telemetry.addLine("  B: Reset Position to (0,0,0)");
        telemetry.addLine("  Y: Toggle Manual Drive");
    }

    @Override
    public void periodic() {
        // Toggle Manual Drive
        if (driver1.pressedY()) {
            manualDriveEnabled = !manualDriveEnabled;
            if (manualDriveEnabled) {
                sixWheel.switchToIdle(); // Stop any PID movement
            }
        }

        // Manual Drive Mode
        if (manualDriveEnabled) {
            double forward = -gamepad1.left_stick_y; // Forward/backward
            double turn = gamepad1.right_stick_x; // Turn left/right
            sixWheel.drive(forward * 0.6, turn * 0.5); // Reduced power for control
            return; // Skip other controls in manual mode
        }

        // Toggle Mode
        if (driver1.pressedX()) {
            if (mode == TuningMode.LINEAR)
                mode = TuningMode.ROTATIONAL;
            else
                mode = TuningMode.LINEAR;
        }

        // Test Move
        if (driver1.pressedA()) {
            if (mode == TuningMode.LINEAR) {
                // Move forward 48 inches
                Point2d[] path = { new Point2d(48, 0) };
                sixWheel.followPathNaive(path);
            } else {
                // Smoother arc path instead of harsh 90 deg turn
                Point2d[] path = {
                        new Point2d(24, 12),
                        new Point2d(36, 36),
                        new Point2d(0, 48)
                };
                sixWheel.followPathNaive(path);
            }
        }

        // Reset position
        if (driver1.pressedB()) {
            sixWheel.switchToIdle();
            sixWheel.setPosition(new Pose2d(0, 0, 0));
        }

        // Update PID constants from dashboard
        sixWheel.updatePIDConstants();
    }

    @Override
    public void telemetry() {
        telemetry.addLine("=== PID TUNING ===");
        telemetry.addData("Mode", mode);
        telemetry.addData("Manual Drive", manualDriveEnabled ? "ENABLED" : "OFF");
        telemetry.addLine("");

        // Real-time error telemetry
        telemetry.addData("Dist Error", String.format("%.2f in", sixWheel.getLastDistance()));
        telemetry.addData("Angle Error", String.format("%.2f deg", sixWheel.getLastAngleError()));
        telemetry.addLine("");

        if (manualDriveEnabled) {
            telemetry.addLine("Use sticks to drive back to start");
            telemetry.addLine("Press Y to exit manual mode");
            telemetry.addLine("Press B to reset odometry to (0,0,0)");
            return;
        }

        if (mode == TuningMode.LINEAR) {
            telemetry.addLine("--- LINEAR MODE (kP_dist) ---");
            telemetry.addData("kP_dist", String.format("%.4f", SixWheelDrive.kP_dist));
            telemetry.addData("kD_dist", String.format("%.4f", SixWheelDrive.kD_dist));
            telemetry.addLine("");
            telemetry.addLine("Press A: Drive 48\" forward");
            telemetry.addLine("");
            telemetry.addLine("TUNING GUIDE:");
            telemetry.addLine("Too SLOW/doesn't reach 48\"? -> INCREASE kP_dist");
            telemetry.addLine("OVERSHOOTS past 48\"? -> DECREASE kP_dist");
            telemetry.addLine("OSCILLATES back/forth? -> DECREASE kP_dist or INCREASE kD_dist");
        } else {
            telemetry.addLine("--- ROTATIONAL MODE (kP_angle) ---");
            telemetry.addData("kP_angle", String.format("%.4f", SixWheelDrive.kP_angle));
            telemetry.addData("kD_angle", String.format("%.4f", SixWheelDrive.kD_angle));
            telemetry.addLine("");
            telemetry.addLine("Press A: Smooth Arc to (0,48)");
            telemetry.addLine("");
            telemetry.addLine("TUNING GUIDE:");
            telemetry.addLine("Turns too SLOW? -> INCREASE kP_angle");
            telemetry.addLine("OVERSHOOTS heading? -> DECREASE kP_angle");
            telemetry.addLine("WIGGLES/oscillates? -> DECREASE kP_angle or INCREASE kD_angle");
        }

        telemetry.addLine("");
        telemetry.addLine("Controls: A=Test | B=Reset | X=Mode | Y=Manual");
    }
}
