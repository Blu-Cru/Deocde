package org.firstinspires.ftc.teamcode.blucru.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

import java.util.Arrays;

@TeleOp(group = "test")
public class LLObeliskTagTest extends BluLinearOpMode {

    public void initialize(){
        addLLTagDetector();
    }

    public void periodic(){
        telemetry.addData("Pattern", Arrays.toString(llTagDetector.getPattern()));
        Pose2d llBotPose = llTagDetector.getLLBotPose();
        telemetry.addData("LL BotPose", "X: " + llBotPose.getX() + ",Y: " + llBotPose.getY() + ",H: " + llBotPose.getH());
    }

}
