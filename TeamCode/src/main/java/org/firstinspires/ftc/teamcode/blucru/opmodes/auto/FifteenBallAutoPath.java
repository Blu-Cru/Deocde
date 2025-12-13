package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.blucru.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;


@Autonomous(name = "15 Ball Auto Path", group = "auto")
public class FifteenBallAutoPath extends BluLinearOpMode {
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;
    public AccelConstraint SLOW_ACCEL = new ProfileAccelConstraint(-20,20);
    

    // dashboard handle
    private FtcDashboard dashboard;

    @Override
    public void initialize() {
        // send all telemetry to DS *and* Dashboard
        enableDash();                      // <- from BluLinearOpMode

        dashboard = FtcDashboard.getInstance();

        Command pickupBalls = new SequentialCommandGroup(
                new IntakeCommand(),
                new WaitCommand(500),
                new TransferCommand(true)
        );

        startPose = new Pose2d(-45, 52, Math.toRadians(127));

        // TankDrive should already be using your Pinpoint localizer internally
        drive = new TankDrive(hardwareMap, startPose);

        path = drive.actionBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-35, 43), Math.toRadians(160+180))

                .waitSeconds(2) // SHOOT PRELOAD

                .setReversed(true)
//                        .lineToX(-32)
                .splineTo(new Vector2d(-15, 47), Math.toRadians(0))  // PICKUP FIRST SET
                .waitSeconds(2)
//                        .turnTo(Math.toRadians(200))
                .setReversed(false)
                .turnTo(Math.toRadians(200))

                .splineTo(new Vector2d(-35, 43), Math.toRadians(135))
                .waitSeconds(2) // SHOOT FIRST SET

                .setReversed(true)
                .splineTo(new Vector2d(0, 47), Math.toRadians(0))
                .splineTo(new Vector2d(10, 47), Math.toRadians(0))  // PICKUP SECOND SET
                .waitSeconds(2)
                .setReversed(false)
                .splineTo(new Vector2d(-35, 43), Math.toRadians(140))
                .waitSeconds(2) // SHOOT SECOND SET

                .setReversed(true)
                .splineTo(new Vector2d(2, 53), Math.toRadians(90))

                .splineTo(new Vector2d(2, 56), Math.toRadians(90),
                        new TranslationalVelConstraint(10.0)) // OPEN GATE
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(new Vector2d(-7, 45), Math.toRadians(180))

                .setReversed(true)
                .splineTo(new Vector2d(30, 47), Math.toRadians(0))
                .splineTo(new Vector2d(35, 47), Math.toRadians(0))  // PICKUP THIRD SET
                .waitSeconds(2)
//                .turnTo(Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(53, 13), Math.toRadians(-20))
                .waitSeconds(2) // SHOOT THIRD SET
                .turnTo(Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(53,40), Math.toRadians(90))
                .splineTo(new Vector2d(53, 47), Math.toRadians(90), new TranslationalVelConstraint(5.0))   // PICKUP FOURTH SET
                .waitSeconds(2)

                .setReversed(false)
                .splineTo(new Vector2d(52.5, 13), Math.toRadians(270))
                .turnTo(Math.toRadians(160))


                .waitSeconds(2)
                .build();
    }

    @Override
    public void onStart() {
        Actions.runBlocking(path);
    }

    @Override
    public void periodic() {
        // ==== 1) GET POSE FROM ROAD RUNNER (Pinpoint localizer feeds this) ====
        Pose2d pose = drive.localizer.getPose();  // com.acmerobotics.roadrunner.Pose2d

        double x = pose.position.x;
        double y = pose.position.y;
        double headingRad = pose.heading.toDouble();
        double headingDeg = Math.toDegrees(headingRad);

        // ---- Driver Station + Dashboard numeric telemetry ----
        telemetry.addData("RR x", x);
        telemetry.addData("RR y", y);
        telemetry.addData("RR heading (deg)", headingDeg);
        telemetry.update();

        // ==== 2) FIELD OVERLAY ON FTC DASHBOARD ====
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", x);
        packet.put("y", y);
        packet.put("headingDeg", headingDeg);

        Canvas field = packet.fieldOverlay();

        // Draw robot as a small circle
        field.strokeCircle(x, y, 2);

        // Draw a short heading line (direction arrow)
        double arrowLen = 6;
        double endX = x + arrowLen * Math.cos(headingRad);
        double endY = y + arrowLen * Math.sin(headingRad);
        field.strokeLine(x, y, endX, endY);

        dashboard.sendTelemetryPacket(packet);
    }
}

