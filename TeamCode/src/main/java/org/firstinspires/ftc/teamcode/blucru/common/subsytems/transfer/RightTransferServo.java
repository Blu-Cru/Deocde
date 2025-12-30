package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RightTransferServo extends TransferServo{
    public static double middlePos = 0.7;
    public RightTransferServo(){
        super("kickerRight");
    }


    @Override
    double getVerticalPos() {
        return 0.74;
    }

    @Override
    double getBottomPos() {
        return 0.13;
    }

    @Override
    double getMiddlePos() {
        return middlePos;
    }
}
