package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.SixWheelDrive;

@TeleOp(name = "Rotational PID Tuning", group = "Tuner")
public class RotationalPIDTuningOpMode extends BluLinearOpMode {

    @Override
    public void initialize() {
        addSixWheel();
        enableDash();
        telemetry.addLine("Rotational PID Tuning OpMode");
        telemetry.addLine("Use Dashboard to tune SixWheelDrive.kP_angle and kD_angle");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A: Rotate 90 Degrees");
        telemetry.addLine("  B: Reset Position to (0,0,0)");
    }

    @Override
    public void periodic() {
        // Test rotational movement - move to point that requires 90 degree turn
        if (driver1.pressedA()) {
            Point2d[] path = { new Point2d(0, 48) };
            sixWheel.followPathNaive(path);
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
        telemetry.addData("kP_angle", SixWheelDrive.kP_angle);
        telemetry.addData("kD_angle", SixWheelDrive.kD_angle);
    }
}
