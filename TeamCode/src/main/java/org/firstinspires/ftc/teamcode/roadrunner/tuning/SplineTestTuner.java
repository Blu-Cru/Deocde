package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;

@TeleOp(name = "Spline Test Tuner", group = "tuning")
public final class SplineTestTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize TankDrive
        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Setup Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            // =================================================================
            // 1. MANUAL DRIVE (Arcade Style)
            // =================================================================
            // Left Stick Y = Forward/Back, Right Stick X = Turn
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            0.0),
                    -gamepad1.right_stick_x));

            drive.updatePoseEstimate();

            // =================================================================
            // 2. PARAMETER TUNING
            // =================================================================
            // D-Pad Up/Down: ramseteBBar (Convergence)
            // D-Pad Left/Right: ramseteZeta (Damping)

            // Simple debouncing logic could be added, but holding is fine for coarse tuning
            if (gamepad1.dpad_up) {
                TankDrive.PARAMS.ramseteBBar += 0.01;
            } else if (gamepad1.dpad_down) {
                TankDrive.PARAMS.ramseteBBar -= 0.01;
            }

            if (gamepad1.dpad_right) {
                TankDrive.PARAMS.ramseteZeta += 0.005;
            } else if (gamepad1.dpad_left) {
                TankDrive.PARAMS.ramseteZeta -= 0.005;
            }

            // Safety Clamps
            if (TankDrive.PARAMS.ramseteBBar < 0)
                TankDrive.PARAMS.ramseteBBar = 0;
            if (TankDrive.PARAMS.ramseteZeta < 0)
                TankDrive.PARAMS.ramseteZeta = 0;
            if (TankDrive.PARAMS.ramseteZeta > 1)
                TankDrive.PARAMS.ramseteZeta = 1;

            // =================================================================
            // 3. RUN PATH
            // =================================================================
            if (gamepad1.a) {
                // Stop manual drive
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

                // Reset Pose to 0,0,0 if user wants (optional, but good for repeatability)
                drive.localizer.setPose(new Pose2d(0, 0, 0));

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(0, 0, 0))
                                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                                .splineTo(new Vector2d(0, 60), Math.PI)
                                .build());
            }

            // =================================================================
            // 4. TELEMETRY
            // =================================================================
            telemetry.addLine("CONTROLS:");
            telemetry.addLine("  sticks: Manual Drive");
            telemetry.addLine("  A: Run Spline Path (Resets Pose to 0,0,0!)");
            telemetry.addLine("  D-Pad Up/Down: Tune BBar (Convergence)");
            telemetry.addLine("  D-Pad L/R: Tune Zeta (Damping)");
            telemetry.addLine("--------------------------------");
            telemetry.addData("ramseteBBar (Aggressiveness)", "%.3f", TankDrive.PARAMS.ramseteBBar);
            telemetry.addData("ramseteZeta (Damping)", "%.3f", TankDrive.PARAMS.ramseteZeta);
            telemetry.addData("X", drive.localizer.getPose().position.x);
            telemetry.addData("Y", drive.localizer.getPose().position.y);
            telemetry.addData("Heading", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.update();
        }
    }
}
