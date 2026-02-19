package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepFarBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(60, -15, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(startPose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(

                myBot.getDrive().actionBuilder(startPose)
                        .setReversed(false)
                        .splineTo(new Vector2d(52, -10), Math.toRadians(-200))
                        .waitSeconds(1)
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new LeftTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new MiddleTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new RightTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new AllTransferMiddleCommand(),
//                                        new WaitCommand(200),
//                                        new IdleShooterCommand(),
//                                        new CenterTurretCommand(),
//                                        new IntakeStartCommand(),
//                                        new WaitCommand(400),
//                                        new AllTransferDownCommand()
//                                )
//                        ))
                        .waitSeconds(2) // SHOOT PRELOAD

                        .turnTo(Math.toRadians(-270))
                        .setReversed(true)
                        .splineTo(new Vector2d(55, -46), Math.toRadians(-80)) // INTAKE FIRST SET
                        .waitSeconds(2)
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new ElevatorUpCommand(),
//                                        new WaitCommand(200),
//                                        new IntakeSpitCommand(),
//                                        new ElevatorMiddleCommand(),
//                                        new ShootWithVelocityCommand(1580),
//                                        new AllTransferMiddleCommand(),
//                                        new WaitCommand(700),
//                                        new IntakeStopCommand(),
//                                        new TurnTurretToPosCommand(-56)
//                                ), false
//                        ))
                        .turnTo(Math.toRadians(-260))
                        .setReversed(false)
                        .lineToY(-14)
                        .turnTo(Math.toRadians(-220))
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new LeftTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new MiddleTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new RightTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new AllTransferMiddleCommand(),
//                                        new WaitCommand(200),
//                                        new IdleShooterCommand(),
//                                        new ElevatorDownCommand(),
//                                        new ElevatorDownCommand(),
//                                        new CenterTurretCommand(),
//                                        new IntakeStartCommand(),
//                                        new WaitCommand(400),
//                                        new AllTransferDownCommand()
//                                )
//                        ))
                        .waitSeconds(2) // SHOOT FIRST SET

                        .turnTo(Math.toRadians(-270))
                        .setReversed(true)
                        .lineToY(-23)
                        .turnTo(Math.toRadians(0))
                        .lineToX(32)
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new ElevatorUpCommand(),
//                                        new WaitCommand(200),
//                                        new IntakeSpitCommand(),
//                                        new ElevatorMiddleCommand(),
//                                        new ShootWithVelocityCommand(1580),
//                                        new WaitCommand(800),
//                                        new AllTransferMiddleCommand(),
//                                        new WaitCommand(300),
//                                        new IntakeStopCommand(),
//                                        new TurnTurretToPosCommand(-55)
//                                ), false
//                        ))
                        .waitSeconds(2)

                        .turnTo(Math.toRadians(45))
                        .setReversed(false)
                        .splineTo(new Vector2d(45, -10), Math.toRadians(-270))
                        .turnTo(Math.toRadians(-220))
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new LeftTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new MiddleTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new RightTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new AllTransferMiddleCommand(),
//                                        new WaitCommand(200),
//                                        new IdleShooterCommand(),
//                                        new CenterTurretCommand(),
//                                        new IntakeStartCommand(),
//                                        new WaitCommand(400),
//                                        new AllTransferDownCommand()
//                                )
//                        ))
                        .waitSeconds(2) // SHOOT SECOND SET

                        .turnTo(Math.toRadians(-270))
                        .setReversed(true)
                        .splineTo(new Vector2d(45, -46), Math.toRadians(-90))
                        .waitSeconds(1) // INTAKE THIRD SET
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new ElevatorUpCommand(),
//                                        new WaitCommand(200),
//                                        new IntakeSpitCommand(),
//                                        new ElevatorMiddleCommand(),
//                                        new ShootWithVelocityCommand(1580),
//                                        new WaitCommand(800),
//                                        new AllTransferMiddleCommand(),
//                                        new WaitCommand(300),
//                                        new IntakeStopCommand(),
//                                        new TurnTurretToPosCommand(-60)
//                                ), false
//                        ))
                        .waitSeconds(2)

                        .setReversed(false)
                        .splineTo(new Vector2d(48, -10), Math.toRadians(-270)) // SHOOT THIRD SET
                        .turnTo(Math.toRadians(-220))
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new LeftTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new MiddleTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new RightTransferUpCommand(),
//                                        new WaitCommand(300),
//                                        new AllTransferMiddleCommand(),
//                                        new WaitCommand(200),
//                                        new IdleShooterCommand(),
//                                        new CenterTurretCommand(),
//                                        new IntakeStartCommand(),
//                                        new WaitCommand(400),
//                                        new AllTransferDownCommand()
//                                )
//                        ))
                        .waitSeconds(2)    .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}