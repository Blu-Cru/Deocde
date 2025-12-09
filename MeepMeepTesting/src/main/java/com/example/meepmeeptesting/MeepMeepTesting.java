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
        Pose2d startPose = new Pose2d(66, 22, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(startPose)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)
                        .waitSeconds(1)
                        .lineToX(56)
                        .turnTo(Math.toRadians(160))//turn to shoot
                        //SHOOT PRELOAD
                        .waitSeconds(1)
                        //PICKUP FIRST SET OF BALLS
                        .turnTo(Math.toRadians(90))
                        .lineToY(46)
                        .turnTo(Math.toRadians(180))
                        .lineToX(35)
                        .turnTo(Math.toRadians(135))
                        .lineToX(60)
                        .turnTo(Math.toRadians(160))
                        //SHOOT FIRST SET
                        .waitSeconds(1)
                        //PICKUP SECOND SET OF BALLS
                        .turnTo(Math.toRadians(142)) //turning to setup for pickup second set

                        .lineToY(46)
                        .turnTo(Math.toRadians(180))
                        .lineToX(10)
                        .turnTo(Math.toRadians(150))
                        .lineToX(56)
                        .turnTo(Math.toRadians(160))//turn to shoot
                        //SHOOT SECOND SET
                        .waitSeconds(1)
                        //PICKUP BALLS ON WALL
                        .turnTo(Math.toRadians(85))
                        .lineToY(65)
                        .waitSeconds(1)
                        .lineToY(22)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}