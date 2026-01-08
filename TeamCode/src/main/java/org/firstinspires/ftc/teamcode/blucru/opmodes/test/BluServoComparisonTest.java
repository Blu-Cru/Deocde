package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
//@TeleOp(group = "test")
//@Config
public class BluServoComparisonTest extends BluLinearOpMode {

    Servo servo;
    BluServo bluServo;

    public static String regServoName = "";
    public static String bluServoName = "";

    public void onStart(){
        //initing on start so that names can be changed in init
        servo = hardwareMap.get(Servo.class, regServoName);
        bluServo = new BluServo(bluServoName);
    }

    public void periodic(){
        if (driver1.pressedA()){
            servo.setPosition(0.5);
            bluServo.setPos(0.5);
        }

        if (driver1.pressedB()){
            //servos should both head the same way
            servo.setPosition(0);
            bluServo.setPos(0);
        }
    }

}
