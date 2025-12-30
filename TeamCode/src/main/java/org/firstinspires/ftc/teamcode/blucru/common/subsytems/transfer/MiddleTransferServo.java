package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MiddleTransferServo extends TransferServo{
    public static double middlePos = 0.6;
    public MiddleTransferServo(){
        super("kickerMiddle");
    }
    @Override
    double getVerticalPos() {
        return 0.78;
    }

    @Override
    double getBottomPos() {
        return 0.15;
    }

    @Override
    double getMiddlePos() {
        return middlePos;
    }
}
