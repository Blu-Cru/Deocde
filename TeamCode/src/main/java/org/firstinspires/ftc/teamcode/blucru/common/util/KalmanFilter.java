package org.firstinspires.ftc.teamcode.blucru.common.util;

/**
 * 1D scalar Kalman filter, Per loop call:
 *   filter.update(u, z);
 *   double estimate = filter.get();
 *
 *   u = control input / predicted change in state since the last update (delta)
 *   z = sensor measurement of the state
 *
 * For pure smoothing with no model, pass u = 0.
 */
public class KalmanFilter {
    private double Q; // process (model) noise covariance
    private double R; // sensor noise covariance
    private double P; // state covariance estimate
    private double K; // last Kalman gain
    private double x; // state estimate

    public KalmanFilter(double Q, double R) {
        this(0, Q, R, 1);
    }

    public KalmanFilter(double x, double Q, double R, double P) {
        this.x = x;
        this.Q = Q;
        this.R = R;
        this.P = P;
        this.K = 1;
    }

    public double update(double u, double z) {
        // Predict
        x = x + u;
        P = P + Q;

        // Correct
        K = P / (P + R);
        x = x + K * (z - x);
        P = (1 - K) * P;

        return x;
    }

    public double update(double z) {
        return update(0, z);
    }

    public void setVal(double val) {
        this.x = val;
    }

    public void setQ(double Q) { this.Q = Q; }
    public void setR(double R) { this.R = R; }

    public double get() { return x; }
    public double getUncertainty() { return P; }
    public double getGain() { return K; }
}
