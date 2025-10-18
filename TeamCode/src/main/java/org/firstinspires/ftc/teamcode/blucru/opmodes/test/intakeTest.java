package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class intakeTest extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addIntake();
    }

    public void periodic(){
        if(gamepad1.left_trigger > 0.2){
            intake.setIn();
        }
        if(gamepad1.right_trigger > 0.2){
            intake.setOut();
        }
    }

}
