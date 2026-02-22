package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

public abstract class AutoConfig {
    public enum AUTOS {
        RED_CLOSE,
        RED_FAR,
        BLUE_CLOSE,
        BLUE_FAR
    }

    public static BaseAuto getAutoInstance(AUTOS givenauto) {
        if (givenauto == AUTOS.BLUE_FAR) {
            return new farBLUEautoFSM();
        }
        return null;
    }

    public static boolean InitBusy() {
        return false;
    }
}
