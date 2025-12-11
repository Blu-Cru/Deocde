package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

public class ShooterAutoAimInterpolation {

    private static double[] dists = {
            27.5, 33.0, 36.75, 42.0, 49.5, 55.25, 62.0, 72.5, 83.0, 93.5
    };
    private static double[] leftAngles = {};
    private static double[] middleAngles = {};
    private static double[] rightAngles = {};
    private static double[] velocities = {850, 850, 900, 1000, 1030, 1070, 1100, 1150, 1170, 1230};

    public static double[] interpolate(double dist){
        int i;

        for (i = 0; i< velocities.length-1; i++){
            if (dists[i+1] > dist){
                //past point, so break
                break;
            }
        }
        double beforePointLeftAngle = leftAngles[i];
        double beforePointMiddleAngle = middleAngles[i];
        double beforePointRightAngle = rightAngles[i];
        double beforePointVel = velocities[i];

        double afterPointLeftAngle = leftAngles[i+1];
        double afterPointMiddleAngle = middleAngles[i+1];
        double afterPointRightAngle = rightAngles[i+1];
        double afterPointVel = velocities[i+1];

        //now do a linear interpolation
        double vel = (afterPointVel - beforePointVel)/(dists[i+1]-dists[i]) * (dist - dists[i]) + beforePointVel;
        double leftShooterAngle = (afterPointLeftAngle - beforePointLeftAngle)/(dists[i+1]-dists[i]) * (dist - dists[i]) + beforePointLeftAngle;
        double rightShooterAngle = (afterPointRightAngle - beforePointRightAngle)/(dists[i+1]-dists[i]) * (dist - dists[i]) + beforePointRightAngle;
        double middleShooterAngle = (afterPointMiddleAngle - beforePointMiddleAngle)/(dists[i+1]-dists[i]) * (dist - dists[i]) + beforePointMiddleAngle;

        return new double[]{leftShooterAngle, middleShooterAngle,rightShooterAngle,vel};
    }

}
