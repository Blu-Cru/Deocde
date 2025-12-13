package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MiddleTransferServo extends TransferServo{
    public static double middlePos = 0.5;
    public MiddleTransferServo(){
        super("kickerMiddle");
    }
    @Override
    double getVerticalPos() {
        return 0.76;
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
