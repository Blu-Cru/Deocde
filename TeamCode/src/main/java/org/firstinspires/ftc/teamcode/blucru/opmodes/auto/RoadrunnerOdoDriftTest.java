package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;

@Autonomous(name = "Odo drift test", group = "auto")
public class RoadrunnerOdoDriftTest extends BluLinearOpMode {
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;
    protected Alliance alliance = Alliance.RED;

    private boolean pathRunning = false;
    private boolean manualMode = true;

    @Override
    public void initialize() {
        Globals.setAlliance(alliance);
        robot.clear();

        startPose = new Pose2d(-45, 30, Math.toRadians(0));
        drive = new TankDrive(hardwareMap, startPose);
    }

    private Action buildPath() {
        return drive.actionBuilder(drive.localizer.getPose())
                .setReversed(false)
                .lineToX(40)
                .waitSeconds(1)
                .setReversed(true)
                .lineToX(-45)
                .waitSeconds(1)
                .setReversed(false)
                .lineToX(40)
                .waitSeconds(1)
                .setReversed(true)
                .lineToX(-45)
                .waitSeconds(1)
                .setReversed(false)
                .lineToX(40)
                .waitSeconds(1)
                .setReversed(true)
                .lineToX(-45)
                .build();
    }

    @Override
    public void onStart() {
        drive.lazyImu.get().resetYaw();
        drive.localizer.setPose(Globals.mapRRPose2d(startPose));

        TelemetryPacket packet = new TelemetryPacket();
        com.acmerobotics.dashboard.FtcDashboard dash = com.acmerobotics.dashboard.FtcDashboard.getInstance();

        while (opModeIsActive() && !isStopRequested()) {

            // =================================================================
            // PARAMETER TUNING (D-Pad)
            // =================================================================
            if (gamepad1.dpad_up) {
                TankDrive.PARAMS.ramseteBBar += 0.02;
            } else if (gamepad1.dpad_down) {
                TankDrive.PARAMS.ramseteBBar -= 0.02;
            }
            if (gamepad1.dpad_right) {
                TankDrive.PARAMS.ramseteZeta += 0.01;
            } else if (gamepad1.dpad_left) {
                TankDrive.PARAMS.ramseteZeta -= 0.01;
            }
            // Clamp values
            if (TankDrive.PARAMS.ramseteBBar < 0)
                TankDrive.PARAMS.ramseteBBar = 0;
            if (TankDrive.PARAMS.ramseteZeta < 0)
                TankDrive.PARAMS.ramseteZeta = 0;
            if (TankDrive.PARAMS.ramseteZeta > 1)
                TankDrive.PARAMS.ramseteZeta = 1;

            // =================================================================
            // MODE CONTROL
            // =================================================================
            // A = Start/Restart Path
            if (gamepad1.a && !pathRunning) {
                drive.localizer.setPose(new Pose2d(-45, 30, 0)); // Reset to start
                path = buildPath();
                pathRunning = true;
                manualMode = false;
            }
            // B = Cancel Path / Enter Manual Mode
            if (gamepad1.b) {
                pathRunning = false;
                manualMode = true;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }
            // X = Reset Pose to Start
            if (gamepad1.x) {
                drive.localizer.setPose(new Pose2d(-45, 30, 0));
            }

            // =================================================================
            // PATH RUNNING OR MANUAL DRIVE
            // =================================================================
            if (pathRunning && path != null) {
                if (!path.run(packet)) {
                    pathRunning = false;
                    manualMode = true;
                }
            } else if (manualMode) {
                // Manual tank drive
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(-gamepad1.left_stick_y, 0.0),
                        -gamepad1.right_stick_x));
            }

            drive.updatePoseEstimate();

            // =================================================================
            // TELEMETRY & DRAWING
            // =================================================================
            double headingDeg = Math.toDegrees(drive.localizer.getPose().heading.toDouble());

            if (drive.poseHistory.size() > 1) {
                Canvas c = packet.fieldOverlay();
                c.setStroke("#3F51B5");
                Drawing.drawRobot(c, drive.localizer.getPose());

                c.setStrokeWidth(1);
                c.setStroke("#3F51B5");
                double[] xPoints = new double[drive.poseHistory.size()];
                double[] yPoints = new double[drive.poseHistory.size()];
                int idx = 0;
                for (Pose2d pose : drive.poseHistory) {
                    xPoints[idx] = pose.position.x;
                    yPoints[idx] = pose.position.y;
                    idx++;
                }
                c.strokePolyline(xPoints, yPoints);
            }

            dash.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();

            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("A: Run Path | B: Stop/Manual | X: Reset Pose");
            telemetry.addLine("D-Pad Up/Down: ramseteBBar | D-Pad L/R: ramseteZeta");
            telemetry.addLine("Sticks: Manual Drive (when not running path)");
            telemetry.addLine("=== TUNING ===");
            telemetry.addData("ramseteBBar", "%.3f", TankDrive.PARAMS.ramseteBBar);
            telemetry.addData("ramseteZeta", "%.3f", TankDrive.PARAMS.ramseteZeta);
            telemetry.addLine("=== STATUS ===");
            telemetry.addData("Mode", pathRunning ? "PATH RUNNING" : "MANUAL");
            telemetry.addData("X", "%.2f", drive.localizer.getPose().position.x);
            telemetry.addData("Y", "%.2f", drive.localizer.getPose().position.y);
            telemetry.addData("Heading (deg)", "%.1f", headingDeg);
            telemetry.update();

            idle();
        }
    }

    @Override
    public void periodic() {
    }

    @Override
    public void telemetry() {
    }
}
