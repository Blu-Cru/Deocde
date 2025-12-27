package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

@Config
public class SixWheelPID {

    private PDController xy;
    private PDController r;
    private double kXY = 0.05, dXY = 0;

    private double kR = 0.01, dR = 0;
    private double kV = 0.035, kVel = 0.01; // Tunable values for Feedforward and Velocity P

    public SixWheelPID() {
        xy = new PDController(kXY, dXY);
        r = new PDController(kR, dR);
    }

    public double getLinearVel(Pose2d robotPose, double dist, Pose2d robotVel) {

        double robotVelXY = Math.sqrt(robotVel.getX() * robotVel.getX() + robotVel.getY() * robotVel.getY());

        double error = dist;

        return xy.calculate(error, -robotVelXY);
    }

    public double getProfiledLinearVel(double targetVel, double currentVel) {
        double error = targetVel - currentVel;
        return (targetVel * kV) + (error * kVel);
    }

    public double getHeadingVel(Pose2d robotPose, Point2d goalPoint, double angleVel) {
        double robotHeading = Math.toDegrees(robotPose.getH());

        // get turn req
        double turnReq = Math
                .toDegrees(Math.atan2(goalPoint.getY() - robotPose.getY(), goalPoint.getX() - robotPose.getX()));

        double deltaAngle = turnReq - robotHeading;

        // make delta angle be between -180 and 180
        while (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        while (deltaAngle <= -180) {
            deltaAngle += 360;
        }
        Globals.telemetry.addData("Robot Heading", robotHeading);
        Globals.telemetry.addData("Turn Req", turnReq);
        Globals.telemetry.addData("Delta Angle", deltaAngle);

        return r.calculate(deltaAngle, -angleVel);
    }

}
