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
                        .lineToX(-44)
//                        .stopAndAdd(new FtclibCommandAction(new ShootWithVelocityCommand(850)))
                        .splineTo(new Vector2d(-28, 38), Math.toRadians(150+180))
//                        .afterTime(0.1, new FtclibCommandAction(new CenterTurretCommand()))
//                        .stopAndAdd(new FtclibCommandAction(new AutonomousShootCloseCommand()))//SHOOT PRELOAD
                        .waitSeconds(5) // SHOOT PRELOAD
                        .turnTo(Math.toRadians(-90))
                        .setReversed(true)
//                        .afterTime(0.1, new FtclibCommandAction(new SequentialCommandGroup(new IntakeStartCommand(), new ElevatorDownCommand(), new CenterTurretCommand())))
                        .splineTo(new Vector2d(-20, 47), Math.toRadians(0))  // PICKUP FIRST SET
                        .splineTo(new Vector2d(-15, 47), Math.toRadians(0))  // PICKUP FIRST SET
                        .waitSeconds(2)
//                        .stopAndAdd(new FtclibCommandAction(new AutonomousTransferCommand()))
                        .setReversed(false)
                        .turnTo(Math.toRadians(200))

                        .splineTo(new Vector2d(-28, 38), Math.toRadians(135))
                        .waitSeconds(2) // SHOOT FIRST SET
//                        .stopAndAdd(new FtclibCommandAction(new AutonomousShootCloseCommand()))//SHOOT PRELOAD

                        .setReversed(true)
                        .splineTo(new Vector2d(0, 47), Math.toRadians(0))
//                        .stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(
//                                new IntakeStartCommand(),
//                                new ElevatorDownCommand(),
//                                new CenterTurretCommand()
//                        )))
                        .splineTo(new Vector2d(10, 47), Math.toRadians(0))  // PICKUP SECOND SET
                        .waitSeconds(2)
                        .setReversed(false)
                        .splineTo(new Vector2d(-28, 38), Math.toRadians(140))
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


                        .setReversed(true)

                        .splineTo(new Vector2d(-35, 43), Math.toRadians(160+180))
//                        .afterTime(0.1, new FtclibCommandAction(new TurnTurretToPosCommand(0)))
//                        .stopAndAdd(new FtclibCommandAction(new AutonomousShootCommand()))//SHOOT PRELOAD
                        .waitSeconds(5)


                        .setReversed(true)
                        .splineTo(new Vector2d(-20, 47), Math.toRadians(0))
//                        .afterTime(0.1, new FtclibCommandAction(new IntakeStartCommand()))
                        .splineTo(new Vector2d(-15, 47), Math.toRadians(0))  // PICKUP FIRST SET
//                        .stopAndAdd(new FtclibCommandAction(new AutonomousTransferCommand()))
                        .turnTo(Math.toRadians(200))
                        .setReversed(false)

                        .splineTo(new Vector2d(-35, 43), Math.toRadians(135))
//                        .stopAndAdd(new FtclibCommandAction(new AutoAimCommand()))
                        .waitSeconds(5) // SHOOT FIRST SET
                        .setReversed(true)
                        .splineTo(new Vector2d(0, 47), Math.toRadians(0))
                        .splineTo(new Vector2d(10, 47), Math.toRadians(0))  // PICKUP SECOND SET
                        .waitSeconds(2)
                        .setReversed(false)
                        .splineTo(new Vector2d(-35, 43), Math.toRadians(140))
                        .waitSeconds(2) // SHOOT SECOND SET

                        .setReversed(true)
                        .splineTo(new Vector2d(2, 50), Math.toRadians(90))

                        .splineTo(new Vector2d(2, 56), Math.toRadians(90),
                                new TranslationalVelConstraint(5.0)) // OPEN GATE
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
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}