package org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer;

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

    void setMiddle(){
        setPos(getMiddlePos());
    }
    abstract double getVerticalPos();
    abstract double getBottomPos();
    abstract double getMiddlePos();
}
