package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
@Config
public class BluServoTest extends BluLinearOpMode {
    BluServo servo;
    public static String name = "elevator";
    public static double pos;
    public void initialize(){
        servo = new BluServo(name);
    }
    public void initializePeriodic(){
        //name updating
        if (driver1.pressedA()){
            servo = new BluServo(name);
        }
    }

    public void periodic(){
        servo.read();

        if (driver1.pressedA()){
            servo.setPos(pos);
        }

        if (driver1.pressedB()){
            if (servo.enabled()){
                servo.disable();
            } else {
                servo.enable();
            }
        }

        servo.write();
        telemetry.addData("Servo pos", servo.getPos());

        telemetry.addData("Servo name", name);
    }
}
