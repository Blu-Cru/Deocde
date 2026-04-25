package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

public abstract class AutoConfig {
    public enum AUTOS {
        BLUE_CLOSE_AUTO,
        FAR_BLUE_AUTO,
        RED_CLOSE_AUTO,
        FAR_RED_AUTO
    }

    private static boolean isInitBusy = false;

    public static BaseAuto getAutoInstance(AUTOS givenauto) {
        isInitBusy = true;
        BaseAuto autoToReturn = null;
        
        if (givenauto == AUTOS.BLUE_CLOSE_AUTO){
            autoToReturn = new BlueCloseAuto();
        } else if (givenauto == AUTOS.FAR_BLUE_AUTO) {
            autoToReturn = new FarBlueAuto();
        } else if (givenauto == AUTOS.RED_CLOSE_AUTO) {
            autoToReturn = new CloseRedAuto();
        } else if (givenauto == AUTOS.FAR_RED_AUTO) {
            autoToReturn = new FarRedAuto();
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
