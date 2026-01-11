package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepFarRed {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(63, 20, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(startPose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)
                        .setReversed(false)
                        .splineTo(new Vector2d(48, 10), Math.toRadians(220))
                        .waitSeconds(2)//SHOOT PRELOAD
                        .turnTo(Math.toRadians(270))
                        .setReversed(true)
                        .splineTo(new Vector2d(48, 47), Math.toRadians(90))//INTAKE FIRST SET
                        .waitSeconds(1)
                        .setReversed(false)
                        .lineToY(10)
                        .turnTo(Math.toRadians(220))
                        .waitSeconds(2)//SHOOT FIRST SET
                        .turnTo(Math.toRadians(270))
                        .setReversed(true)
                        .splineTo(new Vector2d(35, 37), Math.toRadians(180))
                        //INTAKE SECOND SET
                        .setReversed(false)
                        .splineTo(new Vector2d(48, 10), Math.toRadians(270))
                        .turnTo(Math.toRadians(220))
                        .waitSeconds(2)//SHOOT SECOND SET
                        .turnTo(Math.toRadians(270))

                        .setReversed(true)
                        .splineTo(new Vector2d(50, 47), Math.toRadians(90))
                        .waitSeconds(1)//INTAKE THIRD SET
                        .setReversed(false)
                        .splineTo(new Vector2d(48, 10), Math.toRadians(270)) //SHOOT THIRD SET
                        .turnTo(Math.toRadians(220))

                        .waitSeconds(2)
//
//                        //INTAKE FOURTH SET
//                        .turnTo(Math.toRadians(90))
//
//                        .setReversed(true)
//                        .splineTo(new Vector2d(50, 47), Math.toRadians(90))
//                        .waitSeconds(1)
//                        //SHOOT FOURTH SET
//                        .setReversed(false)
//                        .splineTo(new Vector2d(48, 10), Math.toRadians(270))
//                        .turnTo(Math.toRadians(220))

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}