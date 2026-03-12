package org.firstinspires.ftc.teamcode.blucru.common.util;

public class KalmanFilter {
    static double x = 0; // your initial state
    static double Q = 0.1; // your model covariance
    static double R = 0.4; // your sensor covariance
    static double p = 1; // your initial covariance guess
    static double K = 1; // your initial Kalman gain guess

    static double x_previous = x;
    static double p_previous = p;
    static double u = 0;
    static double z = 0;
    static double lastinput = 0;

    public KalmanFilter(double x, double Q, double R, double p, double K) {
        this.x = x;
        this.Q = Q;
        this.R = R;
    }
    public void update(double input) {
        double predictedinput = input-lastinput;
        u = predictedinput; // The delta
        x = x_previous + u;

        p = p_previous + Q;

        K = p/(p + R);

        z = input; // The known current input

        x = x + K * (z - x);

        p = (1 - K) * p;

        x_previous = x;
        p_previous = p;
        lastinput = input;
    }

    public double get() {
        return x;
    }
    public double getUncertainty() {
        return p;
    }
}
