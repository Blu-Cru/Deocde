package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

public abstract class AutoConfig {
    public enum AUTOS {
        RED_CLOSE,
        RED_FAR,
        BLUE_CLOSE,
        BLUE_FAR
    }
    public void initialize(AUTOS givenauto) {
        if (givenauto == AUTOS.RED_CLOSE) {
            PPCloseAUTO.init();
        } else if (givenauto == AUTOS.RED_FAR) {
            PPFarAUTO.init();
        }
    }
}
