package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.qualcomm.robotcore.util.Range;

public class ShooterAutoAimInterpolation {

    private static final double[] dists = {
            27.5, 33.0, 36.75, 42.0, 49.5, 55.25, 62.0, 72.5, 83.0, 93.5, 113.5
    };

    // Arrays populated with your data
    private static final double[] leftAngles = {26, 26, 32, 34, 39, 41, 42, 42, 45, 46, 50};
    private static final double[] middleAngles = {28, 28, 42, 44, 47, 48, 49, 50, 50, 50, 50};
    private static final double[] rightAngles = {26, 26, 36, 38, 43, 45, 46, 47, 48, 50, 50};
    private static final double[] velocities = {850, 850, 900, 1000, 1030, 1070, 1100, 1150, 1170, 1230, 1300};

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