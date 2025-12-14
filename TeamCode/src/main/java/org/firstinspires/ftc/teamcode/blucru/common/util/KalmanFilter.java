package org.firstinspires.ftc.teamcode.blucru.common.util;

/**
 * A simple 1D Recursive Kalman Filter implementation.
 * Can be used to smooth noisy sensor data.
 */
public class KalmanFilter {
    private double Q; // Process noise covariance
    private double R; // Measurement noise covariance
    private double P; // Estimation error covariance
    private double K; // Kalman gain
    private double x; // Value estimate

    private double innovation; // Difference between measurement and estimate

    /**
     * @param Q Process noise covariance (trust in the model/prediction). Higher = faster response, less smoothing.
     * @param R Measurement noise covariance (trust in the measurement). Higher = more smoothing, slower response.
     */
    public KalmanFilter(double Q, double R) {
        this.Q = Q;
        this.R = R;
        this.P = 1; // Initial estimate error (can be anything > 0)
        this.x = 0; // Initial value
    }

    /**
     * Updates the filter with a new measurement.
     * @param measurement The new noisy value from sensor
     * @return The filtered (smoothed) value
     */
    public double update(double measurement) {
        // Prediction update
        // P = P + Q; // P(k|k-1) = P(k-1|k-1) + Q
        // x = x;     // x(k|k-1) = x(k-1|k-1) (assuming constant model)

        P = P + Q;

        // Measurement update
        K = P / (P + R);
        innovation = measurement - x;
        x = x + K * innovation;
        P = (1 - K) * P;

        return x;
    }

    public void setEstimate(double estimate) {
        this.x = estimate;
    }

    public double getEstimate() {
        return x;
    }
    
    /**
     * Returns the last innovation (measurement - predicted state).
     * High innovation indicates the measurement was far from the prediction.
     */
    public double getInnovation() {
        return innovation;
    }
}
