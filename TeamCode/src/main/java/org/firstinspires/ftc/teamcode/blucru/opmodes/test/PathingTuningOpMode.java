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

    @Override
    public void initialize() {
        addSixWheel();
        enableDash();
        telemetry.addLine("PID Tuning OpMode");
        telemetry.addLine("Controls:");
        telemetry.addLine("  X: Toggle Mode (Linear <-> Rotational)");
        telemetry.addLine("  A: Execute Test Move");
        telemetry.addLine("  B: Reset Position to (0,0,0)");
    }

    @Override
    public void periodic() {
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
                // Rotate 90 degrees by moving to point (0, 48)
                Point2d[] path = { new Point2d(0, 48) };
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
        telemetry.addData("Mode", mode);
        if (mode == TuningMode.LINEAR) {
            telemetry.addData("kP_dist", SixWheelDrive.kP_dist);
            telemetry.addData("kD_dist", SixWheelDrive.kD_dist);
        } else {
            telemetry.addData("kP_angle", SixWheelDrive.kP_angle);
            telemetry.addData("kD_angle", SixWheelDrive.kD_angle);
        }
    }
}
