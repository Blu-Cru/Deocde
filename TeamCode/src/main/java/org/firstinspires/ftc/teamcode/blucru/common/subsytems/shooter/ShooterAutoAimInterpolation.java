package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.qualcomm.robotcore.util.Range;

public class ShooterAutoAimInterpolation {

    private static final double[] dists = {
            24.5, 30.5, 36.5, 46, 51, 57.5, 65, 71, 78, 85.5, 90, 98, 105, 118, 126.5, 134.25
    };

    // Arrays populated with your data
    private static final double[] leftAngles = {26, 27, 29, 32, 36, 36, 36, 35.5, 38, 41, 39, 39, 38, 38, 37, 37};
    private static final double[] middleAngles = {26, 27, 28, 31, 37, 39, 40, 42, 46, 48, 46, 47, 38, 39, 38, 38.5};
    private static final double[] rightAngles = {26, 27, 28, 32, 38, 38, 38, 39, 40, 42.5, 43, 44, 39, 39, 39, 39.5};
    private static final double[] velocities = {880, 900, 930, 960, 1000, 1030, 1070, 1100, 1170, 1210, 1230, 1290, 1330, 1390, 1470, 1530};

    public static double[] interpolate(double dist){
        // clamp to range to prevent hardware damage or crashes
        dist = Range.clip(dist, dists[0], dists[dists.length - 1]);

        int i;
        for (i = 0; i < dists.length - 2; i++){
            if (dists[i+1] > dist){
                break;
            }
        }

        // calculate interpolation fraction
        double t = (dist - dists[i]) / (dists[i+1] - dists[i]);

        // linear interpolation
        double vel = lerp(velocities[i], velocities[i+1], t);
        double leftShooterAngle = lerp(leftAngles[i], leftAngles[i+1], t);
        double middleShooterAngle = lerp(middleAngles[i], middleAngles[i+1], t);
        double rightShooterAngle = lerp(rightAngles[i], rightAngles[i+1], t);

        return new double[]{leftShooterAngle, middleShooterAngle, rightShooterAngle, vel};
    }

    private static double lerp(double start, double end, double t) {
        return start + (end - start) * t;
    }
}