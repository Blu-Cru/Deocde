package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.mecanumDrivetrain.control;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

public class TurnToPointKinematics {

    public Vector2d point;

    public TurnToPointKinematics(Vector2d point){this.point = point;}

    public double getHeadingTowardsPoint(Pose2d curr){
        return Math.atan2(point.getY() - curr.getY(),
                point.getX() - curr.getX());
    }

    public Vector2d getHeadingStateTowardsPoint(Pose2d currPose, Pose2d currVel) {
        double heading = getHeadingTowardsPoint(currPose);

        double xDelta = point.getX() - currPose.getX();
        double yDelta = point.getY() - currPose.getY();

        double xVel = currVel.getX();
        double yVel = currVel.getY();

        //to get vel, derive angle over pos, which is the derivative of atan(yDelta/xDelta) with respect to t
        double headingVel = (xDelta * yVel - yDelta * xVel) / (xDelta * xDelta + yDelta * yDelta);

        return new Vector2d(heading, headingVel);
    }
}
