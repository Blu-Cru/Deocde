package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@TeleOp()
public class MotorDirectionFinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.hwMap = hardwareMap;
        BluMotorWithEncoder FL = new BluMotorWithEncoder("FL");
        BluMotorWithEncoder FR = new BluMotorWithEncoder("FR");
        BluMotorWithEncoder BL = new BluMotorWithEncoder("BL");
        BluMotorWithEncoder BR = new BluMotorWithEncoder("BR");

        waitForStart();

        while (opModeIsActive()){
            FL.read();
            FR.read();
            BL.read();
            BR.read();
            if (gamepad1.a){
                telemetry.addLine("here");
                FL.setPower(1);
            } else {
                FL.setPower(0);
            }

            if (gamepad1.y){
                FR.setPower(-1);
                telemetry.addLine("here");
            } else {
                FR.setPower(0);
            }

            if (gamepad1.left_bumper){
                BL.setPower(1);
                telemetry.addLine("here");
            } else {
                BL.setPower(0);
            }

            if (gamepad1.right_bumper){
                BR.setPower(-1);
                telemetry.addLine("here");
            } else {
                BR.setPower(0);
            }
            FL.write();
            FR.write();
            BL.write();
            BR.write();
        }
    }
}
