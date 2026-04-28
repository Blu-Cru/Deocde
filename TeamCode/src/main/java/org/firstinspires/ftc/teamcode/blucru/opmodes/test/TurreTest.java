package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class TurreTest extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addTurret();
    }

    public void periodic(){
        turret.setPower(gamepad1.left_stick_x);
    }

}
