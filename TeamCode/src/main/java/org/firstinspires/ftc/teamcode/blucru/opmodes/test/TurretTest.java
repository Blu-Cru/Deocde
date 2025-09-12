package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

public class TurretTest extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addTurret();
    }

    public void periodic(){
        //positive power is left
        turret.moveWithPower(-gamepad1.left_stick_y);
    }

    public void telemetry(){
        turret.telemetry(telemetry);
    }

}
