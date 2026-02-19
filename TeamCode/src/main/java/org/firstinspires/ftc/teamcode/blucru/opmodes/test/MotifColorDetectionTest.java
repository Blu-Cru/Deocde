package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp
public class MotifColorDetectionTest extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addElevator();
        addLLTagDetector();
    }

    public void periodic(){
        if (gamepad1.a){
            intake.setIn();
        }
        if (gamepad1.y){
            intake.stop();
        }
        if(gamepad1.left_bumper){
            llTagDetector.switchToMotif();
        }
        if(gamepad1.right_bumper){
            ShooterMotifCoordinator.clear();
        }
        elevator.updateLeftBallColor();
        elevator.updateMiddleBallColor();
        elevator.updateRightBallColor();
        telemetry.addData("Left Ball Color", ShooterMotifCoordinator.getLeftColor());
        telemetry.addData("Middle Ball Color", ShooterMotifCoordinator.getMiddleColor());
        telemetry.addData("Right Ball Color", ShooterMotifCoordinator.getRightColor());
        telemetry.addData("Motif", ShooterMotifCoordinator.getMotif());
        telemetry.addData("LL State", llTagDetector.getPipeline());
    }

}
