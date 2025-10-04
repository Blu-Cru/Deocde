package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

public abstract class TransferServo extends BluServo {
    public TransferServo(String name) {
        super(name);
    }
    void setAngle(double degrees){
        double pos = (degrees - 90.0)*getTicksPerDeg() + getVerticalPos();
        setPosition(pos);
    }
    abstract double getTicksPerDeg();
    abstract double getVerticalPos();
}
