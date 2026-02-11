package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp
public class LimelightRelocalizationTest extends BluLinearOpMode {

    public void initialize(){
        addLLTagDetector();
        addSixWheel();
        sixWheel.reset();
    }
    public void onStart(){
        sixWheel.setPosition(new Pose2d(0,0,0));
    }
    public void periodic(){
        llTagDetector.read();
        if (gamepad1.x){
            Pose2d llPose = llTagDetector.getLLBotPosePoseHistory();
            if (llPose != null){
                telemetry.addData("Unoffseted pose", llTagDetector.getLLBotPose());
                gamepad1.rumble(1000);
                sixWheel.setPosition(llPose);
            }
        }
        telemetry.addData("Pattern Detected?", llTagDetector.detectedPattern());
        telemetry.addData("Pattern", ShooterMotifCoordinator.getMotif());
        telemetry.addData("Pipeline", llTagDetector.getPipeline());
        sixWheel.teleDrive(gamepad1, 0.1);
    }


}
