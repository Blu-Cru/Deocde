package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter;

import com.qualcomm.robotcore.util.Range;

public class ShooterAutoAimInterpolation {

    private static final double[] leftDists = {
            32.7, 29, 38.2, 42.2, 47.3, 53.8, 60.3, 65, 69.3, 74.7, 78.4, 83.4, 89, 93.6, 97.75, 104, 108, 115, 119, 127
    };
    private static final double[] leftVelocities = {
            1020, 990, 1020, 1020, 1050, 1080, 1130, 1160, 1210, 1240, 1260, 1290, 1310, 1340, 1380, 1430, 1460, 1480, 1500, 1520
    };

    private static final double[] middleDists = {
            32.7, 29, 38.2, 42.2, 47.3, 53.8, 60.3, 65, 69.3, 74.7, 78.4, 83.4, 89, 93.6, 97.75, 104, 108, 115, 119, 127
    };

    private static final double[] middleVelocities = {
            1020, 990, 1020, 1040, 1080, 1110, 1160, 1190, 1240, 1270, 1290, 1330, 1360, 1380, 1420, 1440, 1460, 1485, 1520, 1545
    };
    private static final double[] rightDists = {
            32.7, 29, 38.2, 42.2, 47.3, 53.8, 60.3, 65, 69.3, 74.7, 78.4, 83.4, 89, 93.6, 97.75, 104, 108, 115, 119, 127
    };

    private static final double[] rightVelocities = {
            1020, 990, 1020, 1040, 1080, 1110, 1160, 1190, 1240, 1270, 1290, 1325, 1345, 1380, 1415, 1440, 1460, 1500, 1540, 1560
    };

    private static final double[] hoodDists = {32.7, 29, 38.2, 42.2, 47.3, 53.8, 60.3, 65, 69.3, 74.7, 78.4, 83.4, 89, 93.6, 97.75, 104, 108, 115, 119, 127};
    private static final double[] hoodAngle = {29, 27, 34, 36, 37, 39, 41, 43, 44, 45, 46, 46.5, 48, 49, 47, 47, 48, 48, 48, 48};


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
        double angle = lerp(hoodAngle[i], hoodAngle[i+1], t);

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
