package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

@Config
public class SixWheelPID {

    private PDController xy;
    private PDController r;
    private PDController rTurnTo;
    private PDController xyLineTo;

    // Stop linear movement when this close to goal to prevent oscillation
    public static double STOP_DISTANCE = 2;

    // Heading-based speed scaling
    public static boolean ENABLE_HEADING_SPEED_SCALING = true;  // Enable/disable cosine scaling
    public static double MIN_SPEED_MULTIPLIER = 0.1;  // Minimum speed when heading error is 90°

    // PID gains - adjust these via FTC Dashboard for tuning
    public static double pXY = 0.04, dXY = 0.005;
    public static double pR = 0.025, dR = 0.1;
    public static double pRTurnTo = 0.02, dRTurnTo = 0.1, ffTurnTo = 0.03;
    public static double pXYLineTo = 0.09, dXYLineTo = 0.015;

    // Track previous backwards driving state for hysteresis
    private boolean wasDriverBackwards = false;
    // Hysteresis thresholds to prevent rapid toggling
    public static double BACKWARDS_THRESHOLD = 100.0; // Switch to backwards
    public static double FORWARDS_THRESHOLD = 80.0;   // Switch back to forwards

    // Debug tracking - stores last calculation values
    private static double lastLinearError = 0;
    private static double lastLinearPTerm = 0;
    private static double lastLinearDTerm = 0;
    private static double lastLinearOutput = 0;

    private static double lastHeadingError = 0;
    private static double lastHeadingTarget = 0;
    private static double lastHeadingPTerm = 0;
    private static double lastHeadingDTerm = 0;
    private static double lastHeadingOutput = 0;

    private static boolean lastBackwardsState = false;

    public SixWheelPID(){
        xy = new PDController(pXY, dXY);
        r = new PDController(pR, dR);
        rTurnTo = new PDController(pRTurnTo, dRTurnTo);
        xyLineTo = new PDController(pXYLineTo, dXYLineTo);
    }

    public void resetBackwardsDrivingState() {
        wasDriverBackwards = false;
    }

    public double getLinearVel(double dist, Pose2d robotVel, boolean isDrivingBackwards, double headingErrorDeg){

        // Stop moving when very close to goal to prevent oscillation
        // The D term can overpower the P term at small distances
        if (dist < STOP_DISTANCE) {
            lastLinearError = dist;
            lastLinearPTerm = 0;
            lastLinearDTerm = 0;
            lastLinearOutput = 0;
            return 0;
        }

        double robotVelXY = Math.sqrt(robotVel.getX() * robotVel.getX() + robotVel.getY() * robotVel.getY());

        double error = dist;

        double linearVel = xy.calculate(error, -robotVelXY);

        linearVel = Math.min(1, linearVel);

        // Apply heading-based speed scaling (cosine scaling)
        if (ENABLE_HEADING_SPEED_SCALING) {
            // Use cosine of heading error to scale speed
            // 0° error → 1.0x speed, 90° error → 0.0x speed
            double cosineMultiplier = Math.abs(Math.cos(Math.toRadians(headingErrorDeg)));
            // Ensure minimum speed multiplier
            double speedMultiplier = Math.max(MIN_SPEED_MULTIPLIER, cosineMultiplier);
            linearVel *= speedMultiplier;
        }

        // Store debug values
        lastLinearError = error;
        lastLinearPTerm = pXY * error;
        lastLinearDTerm = dXY * (-robotVelXY);
        lastLinearOutput = linearVel;

        // If driving backwards, negate the linear velocity
        if (isDrivingBackwards) {
            linearVel = -linearVel;
            lastLinearOutput = linearVel; // Update output after negation
        }

        return linearVel;
    }


    /**
     * Calculate heading velocity to point toward a goal
     * @param robotPose Current robot pose
     * @param goalPoint Target point to face
     * @param angleVel Current angular velocity
     * @param isDrivingBackwards Whether robot is driving backwards (passed from shouldDriveBackwards)
     * @return Required angular velocity
     */
    public double getHeadingVel(Pose2d robotPose, Point2d goalPoint, double angleVel, boolean isDrivingBackwards){
        double robotHeading = Math.toDegrees(robotPose.getH());

        //get turn req
        double turnReq = Math.toDegrees(Math.atan2(goalPoint.getY() - robotPose.getY(), goalPoint.getX() - robotPose.getX()));

        double deltaAngle = turnReq - robotHeading;

        //make delta angle be between -180 and 180
        while (deltaAngle > 180){
            deltaAngle -= 360;
        }
        while (deltaAngle <= -180){
            deltaAngle += 360;
        }

        // If driving backwards, adjust the target angle to face the opposite direction
        if (isDrivingBackwards) {
            if (deltaAngle > 0) {
                deltaAngle -= 180;
            } else {
                deltaAngle += 180;
            }
        }

        // Store debug values
        lastBackwardsState = isDrivingBackwards;
        lastHeadingError = deltaAngle;
        lastHeadingTarget = robotHeading + deltaAngle; // Calculate target heading
        lastHeadingPTerm = pR * deltaAngle;
        lastHeadingDTerm = dR * (-angleVel);
        double headingVel = r.calculate(deltaAngle, -angleVel);
        lastHeadingOutput = headingVel;

        Globals.telemetry.addData("Robot Heading", robotHeading);
        Globals.telemetry.addData("Turn Req", turnReq);
        Globals.telemetry.addData("Delta Angle", deltaAngle);

        return headingVel;
    }

    public boolean shouldDriveBackwards(Pose2d robotPose, Point2d goalPoint) {
        double robotHeading = Math.toDegrees(robotPose.getH());
        double turnReq = Math.toDegrees(Math.atan2(goalPoint.getY() - robotPose.getY(), goalPoint.getX() - robotPose.getX()));

        double deltaAngle = turnReq - robotHeading;

        //make delta angle be between -180 and 180
        while (deltaAngle > 180){
            deltaAngle -= 360;
        }
        while (deltaAngle <= -180){
            deltaAngle += 360;
        }

        double absDeltaAngle = Math.abs(deltaAngle);

        // Hysteresis: use different thresholds based on previous state
        // This prevents rapid toggling around the threshold
        if (wasDriverBackwards) {
            // Currently backwards: only switch to forwards if angle is small enough
            if (absDeltaAngle < FORWARDS_THRESHOLD) {
                wasDriverBackwards = false;
                return false;
            }
            return true;
        } else {
            // Currently forwards: only switch to backwards if angle is large enough
            if (absDeltaAngle > BACKWARDS_THRESHOLD) {
                wasDriverBackwards = true;
                return true;
            }
            return false;
        }
    }

    /**
     * Calculate angular velocity to reach a specific target heading
     * @param robotPose Current robot pose
     * @param targetHeadingDegrees Desired heading in degrees
     * @param angleVel Current angular velocity
     * @return Required angular velocity to reach target heading
     */
    public double getHeadingVelToTargetTurnTo(Pose2d robotPose, double targetHeadingDegrees, double angleVel) {
        double robotHeading = Math.toDegrees(robotPose.getH());
        double deltaAngle = targetHeadingDegrees - robotHeading;

        // Normalize to [-180, 180]
        while (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        while (deltaAngle <= -180) {
            deltaAngle += 360;
        }

        Globals.telemetry.addData("Target Heading Control", targetHeadingDegrees);
        Globals.telemetry.addData("Delta Angle to Target", deltaAngle);

        return rTurnTo.calculate(deltaAngle, -angleVel);
    }
    public void updatePID(){
        xy.setPD(pXY, dXY);
        r.setPD(pR, dR);
    }

    public double lineToX(double targetX, Pose2d robotPose, Pose2d robotVel){
        double dist = targetX - robotPose.getX();
        return xyLineTo.calculate(dist, -robotVel.getX());
    }

    // Debug getters - provide access to last calculation
    public static double getLastLinearError() { return lastLinearError; }
    public static double getLastLinearPTerm() { return lastLinearPTerm; }
    public static double getLastLinearDTerm() { return lastLinearDTerm; }
    public static double getLastLinearOutput() { return lastLinearOutput; }

    public static double getLastHeadingError() { return lastHeadingError; }
    public static double getLastHeadingTarget() { return lastHeadingTarget; }
    public static double getLastHeadingPTerm() { return lastHeadingPTerm; }
    public static double getLastHeadingDTerm() { return lastHeadingDTerm; }
    public static double getLastHeadingOutput() { return lastHeadingOutput; }

    public static boolean getLastBackwardsState() { return lastBackwardsState; }

}
