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
    private double pXY = 0.05, dXY = 0.075;
    private double pR = 0.01, dR = 0.01;

    public SixWheelPID(){
        xy = new PDController(pXY, dXY);
        r = new PDController(pR, dR);
    }

    public double getLinearVel(double dist, Pose2d robotVel, boolean isDrivingBackwards){

        double robotVelXY = Math.sqrt(robotVel.getX() * robotVel.getX() + robotVel.getY() * robotVel.getY());


        double error = dist;

        double linearVel = xy.calculate(error, -robotVelXY);

        // If driving backwards, negate the linear velocity
        if (isDrivingBackwards) {
            linearVel = -linearVel;
        }

        return linearVel;
    }


    public double getHeadingVel(Pose2d robotPose, Point2d goalPoint, double angleVel){
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

        // Check if we should drive backwards instead of turning around
        // If the angle error is > 90 degrees, it's more efficient to drive backwards
        boolean shouldDriveBackwards = Math.abs(deltaAngle) > 90;

        if (shouldDriveBackwards) {
            // Adjust the target angle to face the opposite direction
            // This way we drive backwards toward the goal
            if (deltaAngle > 0) {
                deltaAngle -= 180;
            } else {
                deltaAngle += 180;
            }
        }

        Globals.telemetry.addData("Robot Heading", robotHeading);
        Globals.telemetry.addData("Turn Req", turnReq);
        Globals.telemetry.addData("Delta Angle", deltaAngle);
        Globals.telemetry.addData("Driving Backwards", shouldDriveBackwards);

        return r.calculate(deltaAngle, -angleVel);
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

        return Math.abs(deltaAngle) > 90;
    }

    /**
     * Calculate angular velocity to reach a specific target heading
     * @param robotPose Current robot pose
     * @param targetHeadingDegrees Desired heading in degrees
     * @param angleVel Current angular velocity
     * @return Required angular velocity to reach target heading
     */
    public double getHeadingVelToTarget(Pose2d robotPose, double targetHeadingDegrees, double angleVel) {
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

        return r.calculate(deltaAngle, -angleVel);
    }

}
