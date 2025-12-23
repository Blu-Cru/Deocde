package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;


//@TeleOp
public class SetCurrentPositionTest extends BluLinearOpMode {

    @Override
    public void initialize() {
        robot.clear();
        addDrivetrain();
    }

    @Override
    public void onStart() {
        drivetrain.setCurrentPose(new Pose2d(0,0,Math.PI/2));
    }

    @Override
    public void periodic(){
        if (gamepad1.a){
            drivetrain.setCurrentPose(new Pose2d(0,10,Math.PI/2));
        }
        if (gamepad1.b){
            drivetrain.setCurrentPose(new Pose2d(0,0,Math.PI/2));
        }
        if (gamepad1.x){
            drivetrain.setCurrentPose(new Pose2d(0,0,0));
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("Pose here", "x: "+ drivetrain.getCurrPose().getX() + "y: " + drivetrain.getCurrPose().getY() + "h: " + drivetrain.getCurrPose().getH());
    }
}
