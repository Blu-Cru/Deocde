package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-45, 52, Math.toRadians(127));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(startPose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-33, 40), Math.toRadians(0))
/* .afterTime(0.1, new FtclibCommandAction(
                new SequentialCommandGroup(
                        new LeftTransferUpCommand(),
                        new WaitCommand(100),
                        new MiddleTransferUpCommand(),
                        new WaitCommand(100),
                        new RightTransferUpCommand(),
                        new WaitCommand(500),
                        new CenterTurretCommand(),
                        new WaitCommand(500),
                        new AllTransferDownCommand(),
                        new IntakeStartCommand(),
                        new ElevatorDownCommand()
                ), false
        )) */
                        .waitSeconds(2) // SHOOT PRELOAD
                        .setReversed(true)
                        .splineTo(new Vector2d(-18, 45), Math.toRadians(0))
                        // PICKUP FIRST SET
                        .waitSeconds(2)
/* .stopAndAdd(new FtclibCommandAction(
                new SequentialCommandGroup(
                        new ElevatorUpCommand(),
                        new WaitCommand(300),
                        new ElevatorMiddleCommand(),
                        new WaitCommand(100),
                        new AllTransferMiddleCommand(),
                        new WaitCommand(300),
                        new IntakeStopCommand(),
                        new TurnTurretToPosCommand(60)
                )
        )) */
                        .setReversed(false)
                        .splineTo(new Vector2d(-30, 40), Math.toRadians(210))
                        //SHOOT FIRST SET
/* .stopAndAdd(new FtclibCommandAction(
                new SequentialCommandGroup(
                        new LeftTransferUpCommand(),
                        new WaitCommand(100),
                        new MiddleTransferUpCommand(),
                        new WaitCommand(100),
                        new RightTransferUpCommand(),
                        new WaitCommand(500),
                        new CenterTurretCommand(),
                        new WaitCommand(500),
                        new AllTransferDownCommand(),
                        new IntakeStartCommand(),
                        new ElevatorDownCommand()
                ), false
        )) */
                        .waitSeconds(2)

                        .setReversed(true)
                        .splineTo(new Vector2d(5, 40), Math.toRadians(0))
                        //PICKUP SECOND SET
/* .stopAndAdd(new FtclibCommandAction(
                new SequentialCommandGroup(
                        new ElevatorUpCommand(),
                        new WaitCommand(300),
                        new ElevatorMiddleCommand(),
                        new WaitCommand(100),
                        new AllTransferMiddleCommand(),
                        new WaitCommand(300),
                        new IntakeStopCommand(),
                        new TurnTurretToPosCommand(60)
                )
        )) */
                        .waitSeconds(2)
                        .setReversed(false)
                        .setTangent(Math.toRadians(180))
                        .splineTo(new Vector2d(-25, 40), Math.toRadians(210))
/* .stopAndAdd(new FtclibCommandAction(
                new SequentialCommandGroup(
                        new LeftTransferUpCommand(),
                        new WaitCommand(100),
                        new MiddleTransferUpCommand(),
                        new WaitCommand(100),
                        new RightTransferUpCommand(),
                        new WaitCommand(500),
                        new CenterTurretCommand(),
                        new WaitCommand(500),
                        new AllTransferDownCommand(),
                        new IntakeStartCommand(),
                        new ElevatorDownCommand()
                ), false
        )) */
                        .waitSeconds(2) // SHOOT SECOND SET

                        .setReversed(true)
                        .splineTo(new Vector2d(-2, 50), Math.toRadians(90))

                        .splineTo(new Vector2d(-2, 56), Math.toRadians(90)) // OPEN GATE
                        .waitSeconds(1)
                        .setReversed(false)
                        .splineTo(new Vector2d(-7, 45), Math.toRadians(180))

                        .setReversed(true)
                        .splineTo(new Vector2d(30, 45), Math.toRadians(0))   // PICKUP THIRD SET
/* .stopAndAdd(new FtclibCommandAction(
                new SequentialCommandGroup(
                        new ElevatorUpCommand(),
                        new WaitCommand(300),
                        new ElevatorMiddleCommand(),
                        new WaitCommand(100),
                        new AllTransferMiddleCommand(),
                        new WaitCommand(300),
                        new IntakeStopCommand(),
                        new TurnTurretToPosCommand(60)
                )
        )) */
                        .waitSeconds(2)
                        .setReversed(false)
                        .splineTo(new Vector2d(-30, 40), Math.toRadians(210))//SHOOT THIRD SET
/* .stopAndAdd(new FtclibCommandAction(
                new SequentialCommandGroup(
                        new LeftTransferUpCommand(),
                        new WaitCommand(100),
                        new MiddleTransferUpCommand(),
                        new WaitCommand(100),
                        new RightTransferUpCommand(),
                        new WaitCommand(500),
                        new CenterTurretCommand(),
                        new WaitCommand(500),
                        new AllTransferDownCommand(),
                        new IntakeStartCommand(),
                        new ElevatorDownCommand()
                ), false
        )) */
                        .waitSeconds(2) // SHOOT THIRD SET
                        .setReversed(true)
                        .splineTo(new Vector2d(53,40), Math.toRadians(90))
                        .splineTo(new Vector2d(53, 45), Math.toRadians(90))// PICKUP FOURTH SET
/* .stopAndAdd(new FtclibCommandAction(
                new SequentialCommandGroup(
                        new ElevatorUpCommand(),
                        new IntakeStopCommand(),
                        new WaitCommand(300),
                        new ElevatorMiddleCommand(),
                        new WaitCommand(100),
                        new AllTransferMiddleCommand(),
                        new WaitCommand(300),
                        new TurnTurretToPosCommand(30)
                )
        )) */
                        .waitSeconds(2)

                        .setReversed(false)
                        .splineTo(new Vector2d(52.5, 13), Math.toRadians(270))
                        .turnTo(Math.toRadians(160))
/* .stopAndAdd(
                new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LockOnGoalCommand(),
                                new AutoAimCommand(),
                                new AutonomousShootCommand()
                        )
                )) */
                        .waitSeconds(2)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}