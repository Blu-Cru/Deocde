package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

public class Hood {

    static final double ZERO_ANGLE = 26;
    static final double TOP_ANGLE = 50;
    double BOTTOM_ANGLE_POS = 0;
    double TOP_ANGLE_POS = 1;
    double ANGLE_DELTA = TOP_ANGLE - ZERO_ANGLE;
    double SERVO_POS_DELTA = TOP_ANGLE_POS - BOTTOM_ANGLE_POS;
    double SLOPE = SERVO_POS_DELTA/ANGLE_DELTA;
    private BluServo servo;
    public Hood(String servoName){
        servo = new BluServo(servoName);
    }
    public void init(){servo.init();}

    public void read(){
        servo.read();
    }

    public void write(){
        servo.write();
    }

    public double shooterAngleToPos(double angle){
        return SLOPE * angle - ZERO_ANGLE * SLOPE + BOTTOM_ANGLE_POS;
    }

    public void setShooterAngle(double angle){
        servo.setPos(shooterAngleToPos(angle));
    }

    public double getHoodAngle(){
        return shooterAngleToPos(servo.getPos());
    }

    public void reset(){

    }

    public void telemetry(){
        servo.telemetry();
    }


}
