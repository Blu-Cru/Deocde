package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.qualcomm.robotcore.util.Range;

public class ShooterAutoAimInterpolation {

    private static final double[] leftDists = {
            12.5, 22, 34.5, 42, 48, 59, 73.5, 76, 83.5, 98, 109.5, 118.5, 130, 142.5
    };
    private static final double[] leftAngles = {
            26, 26, 26, 28, 36, 37, 38, 42, 44, 45, 45, 47, 47, 45
    };
    private static final double[] leftVelocities = {
            1000, 1000, 1050, 1050, 1150, 1200, 1250, 1300, 1380, 1440, 1500, 1560, 1660
    };

    private static final double[] middleDists = {
            12.5, 22, 34.5, 42, 48, 59, 73.5, 76, 83.5, 98, 109.5, 118.5, 130, 142.5
    };
    private static final double[] middleAngles = {
            26, 26, 26, 26, 35, 35, 36, 39, 43, 43, 43, 45, 46, 42
    };
    private static final double[] middleVelocities = {
            1000, 1000, 1050, 1050, 1150, 1200, 1250, 1300, 1380, 1420, 1480, 1560, 1640
    };

    private static final double[] rightDists = {
            12.5, 22, 34.5, 42, 48, 54, 73.5, 76, 83.5, 98, 109.5, 118.5, 130, 142.5
    };
    private static final double[] rightAngles = {
            26, 26, 26, 28, 36, 37, 39, 42, 45, 46, 46, 49, 49, 47
    };
    private static final double[] rightVelocities = {
            1000, 1000, 1050, 1050, 1150, 1200, 1250, 1300, 1380, 1440, 1500, 1560, 1640
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