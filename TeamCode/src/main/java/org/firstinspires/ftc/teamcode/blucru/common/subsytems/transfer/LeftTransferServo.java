package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

public class LeftTransferServo extends TransferServo{
    public LeftTransferServo(){
        super("kickerLeft");
    }

    @Override
    double getVerticalPos() {
        return 0.26;
    }

    @Override
    double getBottomPos() {
        return 0.87;
    }

    @Override
    double getMiddlePos() {
        return 0.5;
    }
}
