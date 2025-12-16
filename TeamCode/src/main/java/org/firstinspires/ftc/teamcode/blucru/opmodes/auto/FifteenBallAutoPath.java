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
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCloseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FtclibCommandAction;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
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

        path = drive.actionBuilder(Globals.mapRRPose2d(startPose))
                .setReversed(true)
                .splineTo(Globals.mapRRVector(new Vector2d(-32, 42)), Globals.mapAngle(Math.toRadians(135+180)))
                .waitSeconds(0.4)
                //.lineToX(-44)
                .waitSeconds(1.2) // SHOOT PRELOAD
                .turnTo(Globals.mapAngle(Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(Globals.mapRRVector(new Vector2d(-20, 48)), Globals.mapAngle(Math.toRadians(0)))  // PICKUP FIRST SET
                .splineTo(Globals.mapRRVector(new Vector2d(-15, 48)), Globals.mapAngle(Math.toRadians(0)))  // PICKUP FIRST SET
                .waitSeconds(0.3)
                .setReversed(false)
                .turnTo(Globals.mapAngle(Math.toRadians(200)))
                .splineTo(Globals.mapRRVector(new Vector2d(-28, 38)), Globals.mapAngle(Math.toRadians(120)))
                .waitSeconds(1.2)

                .setReversed(true)
                .splineTo(Globals.mapRRVector(new Vector2d(5, 46)), Globals.mapAngle(Math.toRadians(0)), new TranslationalVelConstraint(20))
                .lineToX(12.5)  // PICKUP SECOND SET
                .waitSeconds(0.1)
                .setReversed(false)

                .splineTo(Globals.mapRRVector(new Vector2d(-28, 38)), Globals.mapAngle(Math.toRadians(130)))
                .waitSeconds(1.2) // SHOOT SECOND SET

                .setReversed(true)
                .splineTo(Globals.mapRRVector(new Vector2d(2, 53)), Globals.mapAngle(Math.toRadians(90)), new TranslationalVelConstraint(20))

                .splineTo(Globals.mapRRVector(new Vector2d(2, 55)), Globals.mapAngle(Math.toRadians(90))) // OPEN GATE
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(Globals.mapRRVector(new Vector2d(-4, 45)), Globals.mapAngle(Math.toRadians(180)))
                .setReversed(true)

                .splineTo(Globals.mapRRVector(new Vector2d(28, 46)), Globals.mapAngle(Math.toRadians(0)), new TranslationalVelConstraint(30))
                .splineTo(Globals.mapRRVector(new Vector2d(37, 46)), Globals.mapAngle(Math.toRadians(0)))  // PICKUP THIRD SET
                .waitSeconds(0.1)
                .setReversed(false)
                .splineTo(Globals.mapRRVector(new Vector2d(-28, 38)), Globals.mapAngle(Math.toRadians(180)))
                .turnTo(Globals.mapAngle(Math.toRadians(180)))
                .build();
    }

    @Override
    public void onStart() {
        Actions.runBlocking(path);
    }

    @Override
    public void periodic() {
        // ==== 1) GET POSE FROM ROAD RUNNER (Pinpoint localizer feeds this) ====
        drive.localizer.update();
        Pose2d pose = drive.localizer.getPose();  // com.acmerobotics.roadrunner.Pose2d

        double x = pose.position.x;
        double y = pose.position.y;
        double headingRad = pose.heading.toDouble();
        double headingDeg = Math.toDegrees(headingRad);

        // ---- Driver Station + Dashboard numeric telemetry ----
        telemetry.addData("RR x", x);
        telemetry.addData("RR y", y);
        telemetry.addData("RR heading (deg)", headingDeg);
/**
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

        dashboard.sendTelemetryPacket(packet);*/
    }
}

