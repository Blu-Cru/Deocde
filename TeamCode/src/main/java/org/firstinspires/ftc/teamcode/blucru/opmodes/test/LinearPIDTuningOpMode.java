package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.SixWheelDrive;

@TeleOp(name = "Linear PID Tuning", group = "Tuner")
public class LinearPIDTuningOpMode extends BluLinearOpMode {

    @Override
    public void initialize() {
        addSixWheel();
        enableDash();
        telemetry.addLine("Linear PID Tuning OpMode");
        telemetry.addLine("Use Dashboard to tune SixWheelDrive.kP_dist and kD_dist");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A: Move Forward 48\"");
        telemetry.addLine("  B: Reset Position to (0,0,0)");
    }

    @Override
    public void periodic() {
        // Test linear movement
        if (driver1.pressedA()) {
            Point2d[] path = { new Point2d(48, 0) };
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
        telemetry.addData("kP_dist", SixWheelDrive.kP_dist);
        telemetry.addData("kD_dist", SixWheelDrive.kD_dist);
    }
}
