package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

public class MiddleTransferServo extends TransferServo{
    public MiddleTransferServo(){
        super("transfer middle");
    }

    @Override
    // TODO: find tick delta for 90 degrees REMEMBER THIS IS +/-   ticks of 90 degrees - ticks of 0 degrees / 90 degrees
    double getTicksPerDeg() {
        return 0;
    }

    @Override
    double getVerticalPos() {
        return 0;
    }
}
