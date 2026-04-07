package org.firstinspires.ftc.teamcode.blucru.common.util;

public class KalmanFilter {
    double x = 0; // your initial state
    double Q = 0.1; // your model covariance
    double R = 0.4; // your sensor covariance
    double P = 1; // your initial covariance guess
    double K = 1; // your initial Kalman gain guess

    double x_previous = x;
    double p_previous = P;
    double u = 0;
    double z = 0;
    double lastinput = 0;

    public KalmanFilter(double x, double Q, double R, double P, double K) {
        this.x = x;
        this.Q = Q;
        this.R = R;
        this.P = P;
        this.K = K;
    }
    public void update(double firstInput, double secondInput) {
        double predictedinput = firstInput-lastinput;
        u = predictedinput; // The delta
        x = x_previous + u;

        P = p_previous + Q;

        K = P/(P + R);

        z = secondInput; // The known current input

        x = x + K * (z - x);

        P = (1 - K) * P;

        x_previous = x;
        p_previous = P;
        lastinput = firstInput;
    }

    /*public void update(double input, double givenlastinput) {
        double predictedinput = input-givenlastinput;
        u = predictedinput; // The delta
        x = x_previous + u;

        P = p_previous + Q;

        K = P/(P + R);

        z = input; // The known current input

        x = x + K * (z - x);

        P = (1 - K) * P;

        x_previous = x;
        p_previous = P;
        lastinput = input;
    }*/
    public void setVal(double val){
        this.x = val;
    }

    public double get() {
        return x;
    }
    public double getUncertainty() {
        return P;
    }
}
