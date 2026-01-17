package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.qualcomm.robotcore.util.Range;

public class ShooterAutoAimInterpolation {

    private static final double[] leftDists = {
            24.5, 30.5, 36.5, 46, 51, 57.5, 65, 71, 78, 85.5, 90, 98, 105, 118, 126.5, 134.25
    };
    private static final double[] leftAngles = {
            26, 27, 29, 32, 36, 36, 36, 35.5, 38, 41, 39, 39, 38, 38, 37, 37
    };
    private static final double[] leftVelocities = {
            880, 900, 930, 960, 1000, 1030, 1070, 1100, 1170, 1210, 1230, 1290, 1330, 1390, 1470, 1530
    };

    private static final double[] middleDists = {
            24.5, 30.5, 36.5, 46, 51, 57.5, 65, 71, 78, 85.5, 90, 98, 105, 118, 126.5, 134.25
    };
    private static final double[] middleAngles = {
            26, 27, 29, 32, 36, 36, 36, 35.5, 38, 41, 39, 39, 38, 38, 37, 37
    };
    private static final double[] middleVelocities = {
            880, 900, 930, 960, 1000, 1030, 1070, 1100, 1170, 1210, 1230, 1290, 1330, 1390, 1470, 1530
    };

    private static final double[] rightDists = {
            24.5, 30.5, 36.5, 46, 51, 57.5, 65, 71, 78, 85.5, 90, 98, 105, 118, 126.5, 134.25
    };
    private static final double[] rightAngles = {
            26, 27, 29, 32, 36, 36, 36, 35.5, 38, 41, 39, 39, 38, 38, 37, 37
    };
    private static final double[] rightVelocities = {
            880, 900, 930, 960, 1000, 1030, 1070, 1100, 1170, 1210, 1230, 1290, 1330, 1390, 1470, 1530
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