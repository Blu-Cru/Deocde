package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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

    @Override
    public void initialize() {
        Globals.setAlliance(alliance);
        robot.clear();

        startPose = new Pose2d(-45, 30, Math.toRadians(0));
        drive = new TankDrive(hardwareMap, startPose);

        path = drive.actionBuilder(startPose)
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
        while (opModeIsActive() && !isStopRequested() && path.run(packet)) {

            double headingDeg = Math.toDegrees(drive.localizer.getPose().heading.toDouble());

            // Draw field visualization (only if pose history has data)
            if (drive.poseHistory.size() > 1) {
                Canvas c = packet.fieldOverlay();

                // Draw current robot position (blue circle with heading line)
                c.setStroke("#3F51B5");
                Drawing.drawRobot(c, drive.localizer.getPose());

                // Draw pose history (blue line trail)
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

            telemetry.addData("Heading (deg)", headingDeg);
            telemetry.addData("Path Running", "True");
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
