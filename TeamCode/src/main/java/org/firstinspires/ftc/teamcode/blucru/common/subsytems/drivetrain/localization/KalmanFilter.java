package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization;

import com.acmerobotics.dashboard.config.Config;

/**
 * Simple Linear Kalman Filter for 2D robot pose (x, y, heading)
 *
 * No Jacobians needed - just simple addition and multiplication!
 *
 * The Kalman Filter has two steps:
 * 1. PREDICT: Use odometry to estimate where we are
 * 2. UPDATE: Use vision to correct our estimate
 *
 * Think of it like this:
 * - Odometry is fast but drifts over time (like dead reckoning)
 * - Vision is accurate but only works when tags are visible
 * - Kalman Filter blends both based on their uncertainty
 */
@Config
public class KalmanFilter {

    // ========== STATE ==========
    private double x;          // Robot X position (inches)
    private double y;          // Robot Y position (inches)
    private double heading;    // Robot heading (radians)

    // ========== UNCERTAINTY (Covariance) ==========
    // How uncertain are we about our position?
    private double P_x;        // Uncertainty in X
    private double P_y;        // Uncertainty in Y
    private double P_h;        // Uncertainty in heading

    // ========== TUNING PARAMETERS ==========

    // Process noise: How much do we trust odometry?
    // Higher = trust odometry less (more responsive to vision)
    public static double Q_X = 0.1;           // Odometry X noise
    public static double Q_Y = 0.1;           // Odometry Y noise
    public static double Q_HEADING = 0.05;    // Odometry heading noise

    // Measurement noise: How much do we trust vision?
    // Higher = trust vision less (smoother corrections)
    public static double R_X = 0.5;           // Vision X noise
    public static double R_Y = 0.5;           // Vision Y noise
    public static double R_HEADING = 0.1;     // Vision heading noise

    /**
     * Initialize filter with starting position
     */
    public KalmanFilter(double startX, double startY, double startHeading) {
        this.x = startX;
        this.y = startY;
        this.heading = startHeading;

        // Start with high uncertainty
        this.P_x = 1.0;
        this.P_y = 1.0;
        this.P_h = 0.5;
    }

    /**
     * STEP 1: PREDICT
     * Use odometry to predict where we are
     *
     * @param dx Change in X from odometry (inches)
     * @param dy Change in Y from odometry (inches)
     * @param dHeading Change in heading from odometry (radians)
     */
    public void predict(double dx, double dy, double dHeading) {
        // Update position estimate (simple addition!)
        x += dx;
        y += dy;
        heading += dHeading;

        // Keep heading in range [-pi, pi]
        heading = normalizeAngle(heading);

        // Increase uncertainty (we're less sure after moving)
        P_x += Q_X;
        P_y += Q_Y;
        P_h += Q_HEADING;
    }

    /**
     * STEP 2: UPDATE (Correct)
     * Use vision measurement to correct our estimate
     *
     * @param measuredX Vision-measured X (inches)
     * @param measuredY Vision-measured Y (inches)
     * @param measuredHeading Vision-measured heading (radians)
     */
    public void update(double measuredX, double measuredY, double measuredHeading) {
        // How different is vision from our prediction?
        double innovationX = measuredX - x;
        double innovationY = measuredY - y;
        double innovationH = normalizeAngle(measuredHeading - heading);

        // Calculate Kalman gain for each dimension
        // Kalman gain = how much to trust the measurement
        // K = P / (P + R)
        // If P is large (uncertain), K is large (trust measurement more)
        // If R is large (noisy measurement), K is small (trust measurement less)
        double K_x = P_x / (P_x + R_X);
        double K_y = P_y / (P_y + R_Y);
        double K_h = P_h / (P_h + R_HEADING);

        // Update estimate: blend prediction and measurement
        x += K_x * innovationX;
        y += K_y * innovationY;
        heading += K_h * innovationH;
        heading = normalizeAngle(heading);

        // Decrease uncertainty (we're more sure after vision correction)
        // P = (1 - K) * P
        P_x = (1 - K_x) * P_x;
        P_y = (1 - K_y) * P_y;
        P_h = (1 - K_h) * P_h;
    }

    /**
     * Get current position estimate
     */
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    /**
     * Get uncertainty (standard deviation)
     */
    public double getPositionUncertainty() {
        return Math.sqrt(P_x * P_x + P_y * P_y);
    }

    public double getHeadingUncertainty() {
        return Math.sqrt(P_h);
    }

    /**
     * Reset filter to new position
     */
    public void reset(double newX, double newY, double newHeading) {
        this.x = newX;
        this.y = newY;
        this.heading = newHeading;

        // Reset to high uncertainty
        this.P_x = 1.0;
        this.P_y = 1.0;
        this.P_h = 0.5;
    }

    /**
     * Keep angle between -pi and pi
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    // ========== DEBUGGING HELPERS ==========

    /**
     * Get the Kalman gain for X (0 to 1)
     * Tells you how much the filter trusts vision vs odometry
     * - Close to 0 = trusting odometry more
     * - Close to 1 = trusting vision more
     */
    public double getKalmanGainX() {
        return P_x / (P_x + R_X);
    }

    public double getKalmanGainY() {
        return P_y / (P_y + R_Y);
    }

    public double getKalmanGainHeading() {
        return P_h / (P_h + R_HEADING);
    }

    /**
     * Get raw uncertainty values
     */
    public double getUncertaintyX() {
        return P_x;
    }

    public double getUncertaintyY() {
        return P_y;
    }

    public double getUncertaintyHeading() {
        return P_h;
    }
}
