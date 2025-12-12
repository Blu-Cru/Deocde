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
                .lineToX(-35)
                .stopAndAdd(new FtclibCommandAction(new ShootBallsCommand()))
                .waitSeconds(1)//SHOOT PRELOAD
                .splineTo(new Vector2d(-20, 47), Math.toRadians(0))
                .stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(
                        new IntakeCommand(),
                        new WaitCommand(1500),
                        new TransferCommand()
                )))
                .splineTo(new Vector2d(-15, 47), Math.toRadians(0))  //PICKUP FIRST SET
                .setReversed(true)
                .splineTo(new Vector2d(-30, 40), Math.toRadians(225))
                .waitSeconds(1)//SHOOT FIRST SET
                .stopAndAdd(new FtclibCommandAction(new ShootBallsCommand()))
                .setReversed(false)
                .splineTo(new Vector2d(0, 47), Math.toRadians(0))
                .stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(
                        new IntakeCommand(),
                        new WaitCommand(1500),
                        new TransferCommand()
                )))
                .splineTo(new Vector2d(15, 47), Math.toRadians(0))  //PICKUP SECOND SET
                .waitSeconds(0)
                .setReversed(true)
                .splineTo(new Vector2d(-16, 25), Math.toRadians(225))
                .stopAndAdd(new FtclibCommandAction(new ShootBallsCommand()))
                .waitSeconds(1)//SHOOT SECOND SET
                .setReversed(false)
                .splineTo(new Vector2d(-2, 57), Math.toRadians(90))
                .waitSeconds(2)//OPEN GATE
                .setReversed(true)
                .splineTo(new Vector2d(-7,45), Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(30, 47), Math.toRadians(0))
                .stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(
                        new IntakeCommand(),
                        new WaitCommand(2500),
                        new TransferCommand()
                )))
                .splineTo(new Vector2d(35, 47), Math.toRadians(0))  //PICKUP THIRD SET
                .splineTo(new Vector2d(43, 10), Math.toRadians(0))
                .stopAndAdd(new FtclibCommandAction(new ShootBallsCommand()))
                .waitSeconds(1)//SHOOT THIRD SET
                .splineTo(new Vector2d(60, 50), Math.toRadians(90))
                .stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(
                        new IntakeCommand(),
                        new WaitCommand(2500),
                        new TransferCommand()
                )))
                .splineTo(new Vector2d(60, 62), Math.toRadians(90))//PICKUP FOURTH SET
                .waitSeconds(0.5)
                .setReversed(true)
                .splineTo(new Vector2d(60, 20), Math.toRadians(270))
                .stopAndAdd(new FtclibCommandAction(new ShootBallsCommand()))




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
