package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.qualcomm.robotcore.util.Range;

public class ShooterAutoAimInterpolation {

    private static final double[] leftDists = {
            16.5, 24, 37, 44.5, 49, 55, 60, 65, 70, 75,
            80, 85, 90, 95, 100, 105, 110, 115, 120, 125,
            130, 135, 140
    };

    private static final double[] leftAngles = {
            26, 26, 27, 30, 31, 33, 36, 39, 42, 42,
            43, 43, 45,
            50, 49, 49, 48, 48, 47, 46, 47, 46, 47
    };

    private static final double[] leftVelocities = {
            960, 980, 1000, 1040, 1080, 1120, 1160, 1200, 1260, 1260,
            1280, 1320, 1360, 1380, 1400, 1400, 1420, 1440, 1500, 1520,
            1560, 1580, 1600
    };

    private static final double[] middleDists = {
            16.5, 24, 37, 44.5, 49, 55, 60, 65, 70, 75,
            80, 85, 90, 95, 100, 105, 110, 115, 120, 125,
            130, 135, 140
    };

    private static final double[] middleAngles = {
            26, 26, 27, 30, 31, 33, 36, 39, 41, 41,
            42, 42, 44,
            49, 48, 48, 47, 47, 46, 46, 46, 45, 46
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

    private static final double[] rightAngles = {
            26, 26, 27, 30, 31, 33, 36, 39, 42, 42,
            43, 43, 45,
            50, 49, 49, 48, 48, 47, 46, 47, 46, 47
    };

    private static final double[] rightVelocities = {
            960, 980, 1000, 1040, 1080, 1120, 1160, 1200, 1260, 1260,
            1320, 1320, 1360, 1380, 1400, 1400, 1420, 1440, 1480, 1500,
            1560, 1580, 1580
    };

    public static double[] interpolateLeft(double dist){
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
        double shooterAngle = lerp(leftAngles[i], leftAngles[i+1], t);

        return new double[]{vel, shooterAngle};
    }

    public static double[] interpolateMiddle(double dist){
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
        double shooterAngle = lerp(middleAngles[i], middleAngles[i+1], t);

        return new double[]{vel, shooterAngle};
    }

    public static double[] interpolateRight(double dist){
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
        double shooterAngle = lerp(rightAngles[i], rightAngles[i+1], t);

        return new double[]{vel, shooterAngle};
    }

    private static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }
}