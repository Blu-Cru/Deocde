package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

public abstract class AutoConfig {
    public enum AUTOS {
        CLOSE_AUTO,
        FAR_BLUE_AUTO
    }

    private static boolean isInitBusy = false;

    public static BaseAuto getAutoInstance(AUTOS givenauto) {
        isInitBusy = true;
        BaseAuto autoToReturn = null;
        
        if (givenauto == AUTOS.CLOSE_AUTO){
            autoToReturn = new CloseAuto();
        } else if (givenauto == AUTOS.FAR_BLUE_AUTO) {
            autoToReturn = new FarBlueAuto();
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
