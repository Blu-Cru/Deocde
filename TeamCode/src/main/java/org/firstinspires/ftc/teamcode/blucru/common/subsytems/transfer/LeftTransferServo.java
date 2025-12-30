package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LeftTransferServo extends TransferServo{
    public static double middlePos = 0.3;
    public LeftTransferServo(){
        super("kickerLeft");
    }

    @Override
    double getVerticalPos() {
        return 0.2;
    }

    @Override
    double getBottomPos() {
        return 0.87;
    }

    @Override
    double getMiddlePos() {
        return middlePos;
    }
}
