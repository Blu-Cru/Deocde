package org.firstinspires.ftc.teamcode.blucru.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class LLObeliskTagTest extends BluLinearOpMode {

    public void initialize(){
        addLLTagDetector();
    }

    public void periodic(){
        Pose2d llBotPose = llTagDetector.getLLBotPose();
        telemetry.addData("Pattern", ShooterMotifCoordinator.getMotif());
        telemetry.addData("LL BotPose", "X: " + llBotPose.getX() + ",Y: " + llBotPose.getY() + ",H: " + llBotPose.getH());
    }

}
