package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RightTransferServo extends TransferServo{
    public static double middlePos = 0.52;
    public RightTransferServo(){
        super("kickerRight");
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
