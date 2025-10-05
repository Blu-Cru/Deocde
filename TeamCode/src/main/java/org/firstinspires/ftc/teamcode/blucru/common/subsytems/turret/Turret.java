package org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluPIDServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class Turret implements BluSubsystem {
    BluPIDServo servo1;
    BluPIDServo servo2;
    public Turret(BluPIDServo servo1, BluPIDServo servo2){
        this.servo1 = servo1;
        this.servo2 = servo2;
    }

    private enum State{
        POWER_MODE,
        PID_MODE,
        GOAL_LOCK_MODE;

    }

    @Override
    public void init() {
        servo1.init();
        servo2.init();
    }

    @Override
    public void read() {
        servo1.read();
        servo2.read();
    }

    @Override
    public void write() {
        servo1.write();
        servo2.write();
    }

    public void moveToPos(double pos){
        servo1.setPosition(pos);
        servo2.setPosition(1-pos);
    }

    public void moveToAngle(double angle){
        double servoPos = Globals.convertServoPosToAngle(255,angle);
        moveToPos(servoPos);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        servo1.telemetry();
        servo2.telemetry();
    }

    @Override
    public void reset() {
    }
}
