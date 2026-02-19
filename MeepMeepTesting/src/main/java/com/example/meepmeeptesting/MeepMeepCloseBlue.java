package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCloseBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-50, -52, Math.toRadians(180-127+180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(startPose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-33, -40), -Math.toRadians(0))
//.afterTime(0.1, new FtclibCommandAction(
//        new SequentialCommandGroup(
//                new LeftTransferUpCommand(),
//                new WaitCommand(200),
//                new MiddleTransferUpCommand(),
//                new WaitCommand(200),
//                new RightTransferUpCommand(),
//                new WaitCommand(500),
//                new CenterTurretCommand(),
//                new WaitCommand(500),
//                new AllTransferDownCommand(),
//                new IntakeStartCommand(),
//                new ElevatorDownCommand()
//        ), false
//))
                        .waitSeconds(1.6)

                        .setReversed(true)
                        .splineTo(new Vector2d(-18, -45), -Math.toRadians(0))
                        .waitSeconds(1)
//.stopAndAdd(new FtclibCommandAction(
//        new SequentialCommandGroup(
//                new ElevatorUpCommand(),
//                new WaitCommand(200),
//                new IntakeStopCommand(),
//                new ElevatorMiddleCommand(),
//                new AllTransferMiddleCommand(),
//                new WaitCommand(200),
//                new TurnTurretToPosCommand(-75)
//        ), false
//))

                        .setReversed(false)
                        .splineTo(new Vector2d(-27, -37), -Math.toRadians(210))
                        .waitSeconds(0.3)
//.stopAndAdd(new FtclibCommandAction(
//        new SequentialCommandGroup(
//                new LeftTransferUpCommand(),
//                new WaitCommand(100),
//                new MiddleTransferUpCommand(),
//                new WaitCommand(100),
//                new RightTransferUpCommand(),
//                new WaitCommand(500),
//                new CenterTurretCommand(),
//                new WaitCommand(500),
//                new AllTransferDownCommand(),
//                new IntakeStartCommand(),
//                new ElevatorDownCommand()
//        ), false
//))
                        .waitSeconds(1.5)

                        .setReversed(true)
                        .splineTo(new Vector2d(5, -38), -Math.toRadians(0))
                        .waitSeconds(2)

//.stopAndAdd(new FtclibCommandAction(
//        new SequentialCommandGroup(
//                new ElevatorUpCommand(),
//                new WaitCommand(300),
//                new ElevatorMiddleCommand(),
//                new WaitCommand(100),
//                new AllTransferMiddleCommand(),
//                new WaitCommand(300),
//                new IntakeStopCommand(),
//                new TurnTurretToPosCommand(-90)
//        ), false
//))

                        .setReversed(false)
                        .setTangent(-Math.toRadians(180))
                        .splineTo(new Vector2d(-19, -32), -Math.toRadians(190))
//.stopAndAdd(new FtclibCommandAction(
//        new SequentialCommandGroup(
//                new LeftTransferUpCommand(),
//                new WaitCommand(100),
//                new MiddleTransferUpCommand(),
//                new WaitCommand(100),
//                new RightTransferUpCommand(),
//                new WaitCommand(500),
//                new CenterTurretCommand(),
//                new WaitCommand(500),
//                new AllTransferDownCommand(),
//                new IntakeStartCommand(),
//                new ElevatorDownCommand()
//        ), false
//))
                        .waitSeconds(2)

                        .setReversed(true)
                        .turnTo(-Math.toRadians(180))
                        .setReversed(true)
                        .splineTo(new Vector2d(2, -45), -Math.toRadians(90))
                        .splineTo(new Vector2d(2, -58), -Math.toRadians(90))
                        .waitSeconds(1.5)

                        .setReversed(false)
                        .splineTo(new Vector2d(-7, -45), -Math.toRadians(180))

                        .setReversed(true)
                        .splineTo(new Vector2d(25, -45), -Math.toRadians(0))
                        .waitSeconds(0.5)
//.stopAndAdd(new FtclibCommandAction(
//        new SequentialCommandGroup(
//                new WaitCommand(500),
//                new ElevatorUpCommand(),
//                new WaitCommand(300),
//                new ElevatorMiddleCommand(),
//                new WaitCommand(100),
//                new AllTransferMiddleCommand(),
//                new WaitCommand(300),
//                new IntakeStopCommand(),
//                new WaitCommand(500),
//                new TurnTurretToPosCommand(-85)
//        )
//))
                        .waitSeconds(1.2)

                        .setReversed(false)
                        .splineTo(new Vector2d(-20, -37), -Math.toRadians(180))
//.stopAndAdd(new FtclibCommandAction(
//        new SequentialCommandGroup(
//                new LeftTransferUpCommand(),
//                new WaitCommand(200),
//                new MiddleTransferUpCommand(),
//                new WaitCommand(200),
//                new RightTransferUpCommand(),
//                new WaitCommand(500),
//                new CenterTurretCommand(),
//                new WaitCommand(500),
//                new AllTransferDownCommand(),
//                new IntakeStartCommand(),
//                new ElevatorDownCommand()
//        ), false
//))
                        .waitSeconds(5)
                        .setReversed(true)
                        .splineTo(new Vector2d(0, -37), Math.toRadians(0))

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}