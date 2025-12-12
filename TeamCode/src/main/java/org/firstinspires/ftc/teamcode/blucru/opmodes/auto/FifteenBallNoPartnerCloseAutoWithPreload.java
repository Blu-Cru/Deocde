package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

// your own commands:
import org.firstinspires.ftc.teamcode.blucru.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ShootBallsCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.FtclibCommandAction;


@Autonomous(name = "15 Ball Close Auto With Preload No Partner", group = "auto")
public class FifteenBallNoPartnerCloseAutoWithPreload extends BluLinearOpMode {
    // TODO: Add trajectory sequence when rr package is configured
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;

    @Override
    public void initialize() {
        addDrivetrain();   // optional, if you still use your drivetrain subsystem
        Command pickupBalls = new SequentialCommandGroup(
                new IntakeCommand(),
                new WaitCommand(500),
                new TransferCommand()
        );

        startPose = new Pose2d(-45, 52, Math.toRadians(307));

        drive = new TankDrive(hardwareMap, startPose);

        path = drive.actionBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-30, 48), Math.toRadians(160+180))
                .stopAndAdd(new FtclibCommandAction(new ShootBallsCommand()))
                .waitSeconds(2) // SHOOT PRELOAD

                .setReversed(true)
//                        .lineToX(-32)
                .stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(
                        new IntakeCommand(),
                        new WaitCommand(2000),
                        new TransferCommand()
                )))
                .splineTo(new Vector2d(-15, 47), Math.toRadians(0))  // PICKUP FIRST SET
                .waitSeconds(2)
                .setReversed(false)
                .splineTo(new Vector2d(-30, 48), Math.toRadians(160))
                .stopAndAdd(new FtclibCommandAction(new ShootBallsCommand()))
                .waitSeconds(2) // SHOOT FIRST SET

                .setReversed(true)
                .splineTo(new Vector2d(0, 47), Math.toRadians(0))
                .stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(
                        new IntakeCommand(),
                        new WaitCommand(2000),
                        new TransferCommand()
                )))
                .splineTo(new Vector2d(10, 47), Math.toRadians(0))  // PICKUP SECOND SET
                .waitSeconds(2)
                .setReversed(false)
                .splineTo(new Vector2d(-30, 48), Math.toRadians(160))
                    .stopAndAdd(new FtclibCommandAction(new ShootBallsCommand()))
                .waitSeconds(2) // SHOOT SECOND SET

                .setReversed(true)
                .splineTo(new Vector2d(-2, 57), Math.toRadians(90))
                .waitSeconds(2) // OPEN GATE

                .setReversed(false)
                .splineTo(new Vector2d(-7, 45), Math.toRadians(180))

                .setReversed(true)
                .splineTo(new Vector2d(30, 47), Math.toRadians(0))
                .stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(
                        new IntakeCommand(),
                        new WaitCommand(2000),
                        new TransferCommand()
                )))
                .splineTo(new Vector2d(35, 47), Math.toRadians(0))  // PICKUP THIRD SET
                .waitSeconds(2)
                .turnTo(Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(46, 13), Math.toRadians(0))
                    .stopAndAdd(new FtclibCommandAction(new ShootBallsCommand()))

                .waitSeconds(2) // SHOOT THIRD SET

                .turnTo(Math.toRadians(-120))
                .setReversed(true)
                .stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(
                        new IntakeCommand(),
                        new WaitCommand(3000),
                        new TransferCommand()
                )))
                .splineTo(new Vector2d(57, 62), Math.toRadians(90))   // PICKUP FOURTH SET
                .waitSeconds(2)

                .setReversed(false)
                .splineTo(new Vector2d(43, 13), Math.toRadians(180))//SHOOT FOURTH SET
                .stopAndAdd(new FtclibCommandAction(new ShootBallsCommand()))
                .waitSeconds(2)
                
                .build();
    }

    @Override
    public void onStart() {
        // BluLinearOpMode already did waitForStart() for you,
        // so this is the tutorial's "waitForStart(); Actions.runBlocking(path);"
        Actions.runBlocking(path);
    }

    @Override
    public void periodic() {
        // If this auto is *just* the RR path, you can leave this empty.
        // If you want extra telemetry or non-RR logic during the match,
        // put it here.
    }
}
