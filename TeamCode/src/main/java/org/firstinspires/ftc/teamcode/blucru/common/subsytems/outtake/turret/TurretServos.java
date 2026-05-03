package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluCRServo;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

public class TurretServos implements BluHardwareDevice {
    private BluCRServo servoLeft;
    private BluCRServo servoRight;

    private BluCRServo servoCenter;

    /**
     *
     * class assumes that positions need to be inverted(ie if one servo needs to go to
     *
     * */
    public TurretServos(BluCRServo servoLeft, BluCRServo servoRight, BluCRServo servoCenter){
        this.servoLeft = servoLeft;
        this.servoRight = servoRight;
        this.servoCenter = servoCenter;
    }

    @Override
    public void init() {
        servoLeft.init();
        servoRight.init();
        servoCenter.init();
    }

    @Override
    public void read() {
        servoLeft.read();
        servoRight.read();
        servoCenter.read();
    }

    @Override
    public void write() {
        servoLeft.write();
        servoRight.write();
        servoCenter.write();
    }

    public void setPower(double power){
        servoLeft.setPower(power);
        servoRight.setPower(power);
        servoCenter.setPower(power);
    }

    public double getPower(){
        return servoLeft.getPower();
    }

    @Override
    public void telemetry() {
        servoLeft.telemetry();
        servoRight.telemetry();
        servoCenter.telemetry();
    }
}
