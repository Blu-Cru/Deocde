package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class TurretEncoderTest extends BluLinearOpMode {

    @Override
    public void initialize(){
        robot.clear();
        addTurret();
    }

    public void periodic(){
        if (gamepad1.a){
            turret.reset();
        }
    }

    @Override
    public void telemetry(){
        telemetry.addData("Turret Encoder", turret.getEncoderPos());
        telemetry.addData("Turret Angle", turret.getAngle());
    }

}
