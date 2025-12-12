package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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
                        // baseline flipped: start driving "backwards" relative to old front
                        .setReversed(true)
                        .splineTo(new Vector2d(-30, 48), Math.toRadians(160+180))

                        .waitSeconds(2) // SHOOT PRELOAD

                        .setReversed(true)
//                        .lineToX(-32)
                        .splineTo(new Vector2d(-15, 47), Math.toRadians(0))  // PICKUP FIRST SET
                        .waitSeconds(2)
//                        .turnTo(Math.toRadians(200))
                        .setReversed(false)
                        .splineTo(new Vector2d(-30, 48), Math.toRadians(160))
                        .waitSeconds(2) // SHOOT FIRST SET

                        .setReversed(true)
                        .splineTo(new Vector2d(0, 47), Math.toRadians(0))
                        .splineTo(new Vector2d(10, 47), Math.toRadians(0))  // PICKUP SECOND SET
                        .waitSeconds(2)
                        .setReversed(false)
                        .splineTo(new Vector2d(-30, 48), Math.toRadians(160))
                        .waitSeconds(2) // SHOOT SECOND SET

                        .setReversed(true)
                        .splineTo(new Vector2d(-2, 57), Math.toRadians(90))
                        .waitSeconds(2) // OPEN GATE

                        .setReversed(false)
                        .splineTo(new Vector2d(-7, 45), Math.toRadians(180))

                        .setReversed(true)
                        .splineTo(new Vector2d(30, 47), Math.toRadians(0))
                        .splineTo(new Vector2d(35, 47), Math.toRadians(0))  // PICKUP THIRD SET
                        .waitSeconds(2)
                        .turnTo(Math.toRadians(90))
                        .setReversed(true)
                        .splineTo(new Vector2d(46, 13), Math.toRadians(0))
                        .waitSeconds(2) // SHOOT THIRD SET

                        .turnTo(Math.toRadians(-120))
                        .setReversed(true)

                        .splineTo(new Vector2d(57, 62), Math.toRadians(90))   // PICKUP FOURTH SET
                        .waitSeconds(2)

                        .setReversed(false)
                        .splineTo(new Vector2d(43, 13), Math.toRadians(180))//SHOOT FOURTH SET
                        .waitSeconds(2)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}