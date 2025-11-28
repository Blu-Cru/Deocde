package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

public class MiddleTransferServo extends TransferServo{
    public MiddleTransferServo(){
        super("kickerMiddle");
    }
    @Override
    double getVerticalPos() {
        return 0.8;
    }

    @Override
    double getBottomPos() {
        return 0.3;
    }
}
