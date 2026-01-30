package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LeftTransferServo extends TransferServo{
    public static double middlePos = 0.46;
    public LeftTransferServo(){
        super("kickerLeft");
    }

    @Override
    double getVerticalPos() {
        return 0.72;
    }

    @Override
    double getBottomPos() {
        return 0.3;
    }

    @Override
    double getMiddlePos() {
        return middlePos;
    }
}
