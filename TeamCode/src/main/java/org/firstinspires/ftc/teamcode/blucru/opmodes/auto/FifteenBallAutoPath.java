package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.blucru.common.commands.FtclibCommandAction;
import org.firstinspires.ftc.teamcode.blucru.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ShootBallsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;


@Autonomous(name = "15 Ball Auto Path", group = "auto")
public class FifteenBallAutoPath extends BluLinearOpMode {
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;

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
                new TransferCommand()
        );

        startPose = new Pose2d(-45, 52, Math.toRadians(307));

        // TankDrive should already be using your Pinpoint localizer internally
        drive = new TankDrive(hardwareMap, startPose);

        path = drive.actionBuilder(startPose)
                .lineToX(-30)
                .waitSeconds(2)//SHOOT PRELOAD
                .turnTo(Math.toRadians(70))
                .splineTo(new Vector2d(-20, 47), Math.toRadians(0))
                .splineTo(new Vector2d(-15, 47), Math.toRadians(0))  //PICKUP FIRST SET
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(-30, 40), Math.toRadians(225))
                .waitSeconds(2)//SHOOT FIRST SET
                .setReversed(false)
                .splineTo(new Vector2d(0, 47), Math.toRadians(0))
                .splineTo(new Vector2d(15, 47), Math.toRadians(0))  //PICKUP SECOND SET
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(-16, 25), Math.toRadians(225))
                .waitSeconds(2)//SHOOT SECOND SET
                .setReversed(false)
                .splineTo(new Vector2d(-2, 57), Math.toRadians(90))
                .waitSeconds(2)//OPEN GATE
                .setReversed(true)
                .splineTo(new Vector2d(-7,45), Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(30, 47), Math.toRadians(0))
                .splineTo(new Vector2d(35, 47), Math.toRadians(0))  //PICKUP THIRD SET
                .splineTo(new Vector2d(43, 10), Math.toRadians(0))
                .waitSeconds(2)//SHOOT THIRD SET
                .splineTo(new Vector2d(60, 50), Math.toRadians(90))
                .splineTo(new Vector2d(60, 62), Math.toRadians(90))//PICKUP FOURTH SET
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(60, 20), Math.toRadians(270))
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

