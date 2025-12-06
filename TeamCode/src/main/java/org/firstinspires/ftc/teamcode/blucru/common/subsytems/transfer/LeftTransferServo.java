package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

public class LeftTransferServo extends TransferServo{
    public LeftTransferServo(){
        super("kickerLeft");
    }

    @Override
    double getVerticalPos() {
        return 0.3;
    }

    @Override
    double getBottomPos() {
        return 0.71;
    }
}
