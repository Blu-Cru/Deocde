package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
//@TeleOp(group = "test")
public class BluMotorWithEncoderComparisonTest extends BluLinearOpMode {
    BluMotorWithEncoder bluMotor;
    DcMotorEx dcMotor;
    @Override
    public void initialize(){
        bluMotor = new BluMotorWithEncoder("br");
        dcMotor = hardwareMap.get(DcMotorEx.class, "bl");
        bluMotor.setPower(0);
        bluMotor.write();
        dcMotor.setPower(0);
    }

    public void periodic(){
        //motors should both head the same way
        bluMotor.read();
        bluMotor.setPower(0.2);
        bluMotor.write();
        dcMotor.setPower(0.2);
    }

    public void telemetry(){
        telemetry.addData("Blu Motor encoder", bluMotor.getCurrentPos());
        telemetry.addData("Blu Motor Direction", bluMotor.getDirection());
        telemetry.addData("DcMotor encoder", dcMotor.getCurrentPosition());
        telemetry.addData("Blu Motor Power", "DcMotor val: " + bluMotor.getDcMotorPower() + ", Set Power: " + bluMotor.getPower());
        telemetry.addData("DcMotor Power", dcMotor.getPower());
    }
}
