package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp
public class IntakePidTuning extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addIntake();
    }

    public void periodic(){
        if (gamepad1.a){
            intake.setIn();
        }
        if (gamepad1.b){
            intake.stop();
        }
        if (gamepad1.x){
            intake.setPID();
        }

    }

}
