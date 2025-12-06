package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

public class RightTransferServo extends TransferServo{
    public RightTransferServo(){
        super("kickerRight");
    }


    @Override
    double getVerticalPos() {
        return 0.75;
    }

    @Override
    double getBottomPos() {
        return 0.27;
    }
}
