package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class kickerTest extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addTransfer();
    }

    public void periodic(){
        if(gamepad1.dpad_left){
            transfer.leftSetUp();
        }
        if(gamepad1.dpad_down){
            transfer.middleSetUp();
        }

        if(gamepad1.dpad_right){
            transfer.rightSetUp();
        }
        if(gamepad1.left_bumper){
            transfer.setAllUp();
        }
        if(gamepad1.right_bumper){
            transfer.setAllDown();
        }
    }

}
