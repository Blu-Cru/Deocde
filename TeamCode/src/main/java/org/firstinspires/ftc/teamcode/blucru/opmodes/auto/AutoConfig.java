package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

public abstract class AutoConfig {
    public enum AUTOS {
        CLOSE_BLUE,
        FAR_BLUE,
        FAR_BLUE_SWEEP,
        CLOSE_RED,
        FAR_RED,
        FAR_RED_SWEEP
    }

    private static boolean isInitBusy = false;

    public static BaseAuto getAutoInstance(AUTOS givenauto) {
        isInitBusy = true;
        BaseAuto autoToReturn = null;
        
        if (givenauto == AUTOS.CLOSE_BLUE){
            autoToReturn = new CloseBlueAuto();
        } else if (givenauto == AUTOS.FAR_BLUE) {
            autoToReturn = new FarBlueAuto();
        } else if (givenauto == AUTOS.FAR_BLUE_SWEEP) {
            autoToReturn = new FarBlueAutoSweep();
        } else if (givenauto == AUTOS.CLOSE_RED) {
            autoToReturn = new CloseRedAuto();
        } else if (givenauto == AUTOS.FAR_RED) {
            autoToReturn = new FarRedAuto();
        } else if (givenauto == AUTOS.FAR_RED_SWEEP) {
            autoToReturn = new FarRedAutoSweep();
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
