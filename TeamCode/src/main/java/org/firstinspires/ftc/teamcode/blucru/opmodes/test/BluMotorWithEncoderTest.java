package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
public class BluMotorWithEncoderTest extends BluLinearOpMode {
    BluMotorWithEncoder motor;
    public void initialize(){
        motor = new BluMotorWithEncoder(Globals.frMotorName);
        motor.read();
    }

    public void periodic(){
        motor.read();
    }

    public void telemetry(){
        telemetry.addData("Encoder pos", motor.getCurrentPos());
    }

}
