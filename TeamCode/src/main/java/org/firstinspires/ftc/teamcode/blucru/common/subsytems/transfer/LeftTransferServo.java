package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LeftTransferServo extends TransferServo{
    public static double middlePos = 0.44;
    public LeftTransferServo(){
        super("kickerLeft");
    }

    @Override
    double getVerticalPos() {
        return 0.75;
    }

    @Override
    double getBottomPos() {
        return 0.23;
    }

    @Override
    double getMiddlePos() {
        return middlePos;
    }
}
