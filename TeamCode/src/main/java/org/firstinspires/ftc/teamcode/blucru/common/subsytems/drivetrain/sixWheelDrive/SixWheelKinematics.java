package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive;

public class SixWheelKinematics {

    public static double[] getPowers(double x, double rot){
        double left = x+rot;
        double right = x-rot;

        double max = Math.max(Math.abs(left), Math.abs(right));

        if (max>0){
            left /= max;
            right /= max;
        }

        return new double[]{left, right};
    }

}
