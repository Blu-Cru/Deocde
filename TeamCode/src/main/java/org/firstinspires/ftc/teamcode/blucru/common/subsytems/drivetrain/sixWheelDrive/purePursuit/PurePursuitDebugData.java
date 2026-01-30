package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit;

/**
 * Data structure to hold all PurePursuit debug information for a single time point
 * Used for logging and analysis of path following behavior
 */
public class PurePursuitDebugData {
    // Timestamp
    public long timestamp;

    // Robot state
    public double robotX;
    public double robotY;
    public double robotHeadingDeg;
    public double velX;
    public double velY;
    public double angularVel;

    // Path following
    public double targetX;
    public double targetY;
    public double distanceRemaining;
    public int currentSegmentIndex;
    public double lookAheadDistance;

    // Linear control (PD)
    public double linearError;
    public double linearPTerm;
    public double linearDTerm;
    public double linearOutput;

    // Heading control (PD)
    public double headingCurrentDeg;
    public double headingTargetDeg;
    public double headingErrorDeg;
    public double headingPTerm;
    public double headingDTerm;
    public double headingOutput;

    // State
    public boolean isDrivingBackwards;

    /**
     * Get CSV header row
     */
    public static String toCSVHeader() {
        return "timestamp_ms,robot_x,robot_y,robot_heading_deg,vel_x,vel_y,angular_vel," +
               "target_x,target_y,dist_remaining,segment_idx," +
               "linear_error,linear_p,linear_d,linear_out," +
               "heading_current_deg,heading_target_deg,heading_err_deg,heading_p,heading_d,heading_out," +
               "backwards,lookahead";
    }

    /**
     * Convert this data point to a CSV row
     */
    public String toCSVRow() {
        return String.format("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f," +
                           "%.3f,%.3f,%.3f,%d," +
                           "%.3f,%.3f,%.3f,%.3f," +
                           "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f," +
                           "%b,%.3f",
                timestamp, robotX, robotY, robotHeadingDeg, velX, velY, angularVel,
                targetX, targetY, distanceRemaining, currentSegmentIndex,
                linearError, linearPTerm, linearDTerm, linearOutput,
                headingCurrentDeg, headingTargetDeg, headingErrorDeg, headingPTerm, headingDTerm, headingOutput,
                isDrivingBackwards, lookAheadDistance);
    }

    /**
     * Convert to human-readable string for telemetry
     */
    @Override
    public String toString() {
        return String.format(
            "Time: %dms\n" +
            "Robot: (%.2f, %.2f, %.2f°)\n" +
            "Vel: (%.2f, %.2f, %.2f°/s)\n" +
            "Target: (%.2f, %.2f)\n" +
            "Dist: %.2f, Seg: %d\n" +
            "Linear: err=%.2f, P=%.3f, D=%.3f, out=%.3f\n" +
            "Heading: cur=%.2f°, tgt=%.2f°, err=%.2f°, P=%.3f, D=%.3f, out=%.3f\n" +
            "Backwards: %b, Lookahead: %.2f",
            timestamp,
            robotX, robotY, robotHeadingDeg,
            velX, velY, angularVel,
            targetX, targetY,
            distanceRemaining, currentSegmentIndex,
            linearError, linearPTerm, linearDTerm, linearOutput,
            headingCurrentDeg, headingTargetDeg, headingErrorDeg, headingPTerm, headingDTerm, headingOutput,
            isDrivingBackwards, lookAheadDistance);
    }
}
