package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

import java.util.Arrays;
import java.util.List;

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
