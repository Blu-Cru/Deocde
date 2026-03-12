package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter;

import com.qualcomm.robotcore.util.Range;

public class ShooterAutoAimInterpolation {

    private static final double[] leftDists = {
            16.5, 24, 37, 44.5, 49, 55
    };
    private static final double[] leftVelocities = {
            1000,1000,1000,1000,1000,1000
    };

    private static final double[] middleDists = {
            16.5, 24, 37, 44.5, 49, 55
    };

    private static final double[] middleVelocities = {
            960, 980, 1000, 1040, 1080, 1120, 1180, 1200, 1280, 1280,
            1300, 1340, 1380, 1400, 1420, 1440, 1460, 1480, 1520, 1540,
            1560, 1580, 1600
    };

    private static final double[] rightDists = {
            16.5, 24, 37, 44.5, 49, 55, 60, 65, 70, 75,
            80, 85, 90, 95, 100, 105, 110, 115, 120, 125,
            130, 135, 140
    };

    private static final double[] rightVelocities = {
            960, 980, 1000, 1040, 1080, 1120, 1160, 1200, 1260, 1260,
            1320, 1320, 1360, 1380, 1400, 1400, 1420, 1440, 1480, 1500,
            1560, 1580, 1580
    };

    private static final double[] hoodDists = {10,20,30,40};
    private static final double[] hoodAngle = {28,32,35,37};


    private static final double[] middleShooterVels = {
            1000, 1200, 1250, 1400
    };

    private static final double[] middleShooterExitVels = {
            6, 9, 10, 11
    };


    public static double interpolateLeft(double dist){
        // clamp to range to prevent hardware damage or crashes
        dist = Range.clip(dist, leftDists[0], leftDists[leftDists.length - 1]);

        int i;
        for (i = 0; i < leftDists.length - 2; i++){
            if (leftDists[i+1] > dist){
                break;
            }
        }

        // calculate interpolation fraction
        double t = (dist - leftDists[i]) / (leftDists[i+1] - leftDists[i]);

        // linear interpolation
        double vel = lerp(leftVelocities[i], leftVelocities[i+1], t);

        return vel;
    }

    public static double interpolateMiddle(double dist){
        // clamp to range to prevent hardware damage or crashes
        dist = Range.clip(dist, middleDists[0], middleDists[middleDists.length - 1]);

        int i;
        for (i = 0; i < middleDists.length - 2; i++){
            if (middleDists[i+1] > dist){
                break;
            }
        }

        // calculate interpolation fraction
        double t = (dist - middleDists[i]) / (middleDists[i+1] - middleDists[i]);

        // linear interpolation
        double vel = lerp(middleVelocities[i], middleVelocities[i+1], t);

        return vel;
    }

    public static double interpolateRight(double dist){
        // clamp to range to prevent hardware damage or crashes
        dist = Range.clip(dist, rightDists[0], rightDists[rightDists.length - 1]);

        int i;
        for (i = 0; i < rightDists.length - 2; i++){
            if (rightDists[i+1] > dist){
                break;
            }
        }

        // calculate interpolation fraction
        double t = (dist - rightDists[i]) / (rightDists[i+1] - rightDists[i]);

        // linear interpolation
        double vel = lerp(rightVelocities[i], rightVelocities[i+1], t);

        return vel;
    }

    public static double interpolateHood(double dist){
        // clamp to range to prevent hardware damage or crashes
        dist = Range.clip(dist, hoodDists[0], hoodDists[hoodDists.length - 1]);

        int i;
        for (i = 0; i < hoodDists.length - 2; i++){
            if (hoodDists[i+1] > dist){
                break;
            }
        }

        // calculate interpolation fraction
        double t = (dist - hoodDists[i]) / (hoodDists[i+1] - hoodDists[i]);

        // linear interpolation
        double angle = lerp(hoodDists[i], hoodDists[i+1], t);

        return angle;
    }

    private static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }

    public static double getBallExitVel(double vel) {
        // clamp to range to prevent hardware damage or crashes
        vel = Range.clip(vel, middleShooterVels[0], middleShooterVels[middleShooterVels.length - 1]);

        int i;
        for (i = 0; i < middleShooterVels.length - 2; i++){
            if (middleShooterVels[i+1] > vel){
                break;
            }
        }

        // calculate interpolation fraction
        double t = (vel - middleShooterVels[i]) / (middleShooterVels[i+1] - middleShooterVels[i]);

        // linear interpolation
        double exitVel = lerp(middleShooterExitVels[i], middleShooterExitVels[i+1], t);

        return exitVel;
    }
}
