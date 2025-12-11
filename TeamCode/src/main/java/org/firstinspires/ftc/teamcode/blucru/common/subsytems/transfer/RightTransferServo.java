package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

public class RightTransferServo extends TransferServo{
    public RightTransferServo(){
        super("kickerRight");
    }


    @Override
    double getVerticalPos() {
        return 0.72;
    }

    @Override
    double getBottomPos() {
        return 0.13;
    }

    @Override
    double getMiddlePos() {
        return 0.5;
    }
}
