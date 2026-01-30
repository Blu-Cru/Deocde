package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp
public class TurretTest extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addTurret();
    }

    public void periodic(){
        //positive power is left
        turret.setPower(-gamepad1.left_stick_y);
    }

    public void telemetry(){
        turret.telemetry(telemetry);
    }

}
