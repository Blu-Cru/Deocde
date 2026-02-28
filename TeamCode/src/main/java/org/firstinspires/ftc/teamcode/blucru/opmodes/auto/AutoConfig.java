package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

public abstract class AutoConfig {
    public enum AUTOS {
        RED_CLOSE,
        RED_FAR,
        BLUE_CLOSE,
        BLUE_FAR,
        RED_CLOSE_MOTIF,
        BLUE_CLOSE_MOTIF,
        ROOT_NEGATIVE_ONE
    }

    private static boolean isInitBusy = false;

    public static BaseAuto getAutoInstance(AUTOS givenauto) {
        isInitBusy = true;
        BaseAuto autoToReturn = null;
        
        if (givenauto == AUTOS.BLUE_FAR) {
            autoToReturn = new farBLUEautoFSM();
        } else if (givenauto == AUTOS.BLUE_CLOSE) {
            autoToReturn = new PPCloseBlueAuto();
        } else if (givenauto == AUTOS.RED_FAR) {
            autoToReturn = new farREDauto();
        } else if (givenauto == AUTOS.RED_CLOSE) {
            autoToReturn = new PPCloseRedAuto();
        } else if (givenauto == AUTOS.RED_CLOSE_MOTIF) {
            autoToReturn = new PPCloseRedAutoMotif();
        } else if (givenauto == AUTOS.BLUE_CLOSE_MOTIF) {
            autoToReturn = new PPCloseBlueAutoMotif();
        } else if (givenauto == AUTOS.ROOT_NEGATIVE_ONE){
            autoToReturn = new RootNegativeOneFSM();
        }

        // Simulating the end of synchronous instance creation for path generation.
        // Once this returns, it's immediately ready in Auto.java, so we mark it done.
        isInitBusy = false;
        
        return autoToReturn;
    }

    public static boolean InitBusy() {
        return isInitBusy;
    }
}
