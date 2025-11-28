package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;

public abstract class TransferServo extends BluServo {
    public TransferServo(String name) {
        super(name);
    }
    void setBottom(){
        setPos(getBottomPos());
    }
    void setVertical(){
        setPos(getVerticalPos());
    }
    abstract double getVerticalPos();
    abstract double getBottomPos();
}
