package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

public class SixWheelPID {

    private PDController xy;
    private PDController r;
    private double kXY = 0, dXY = 0;
    private double kR = 0, dR = 0;

    public SixWheelPID(){
        xy = new PDController(kXY, dXY);
        r = new PDController(kR, dR);
    }

    public double getLinearVel(Pose2d robotPose, Point2d targetPose, Pose2d robotVel){

        double robotVelXY = Math.sqrt(robotVel.getX() * robotVel.getX() + robotVel.getY() * robotVel.getY());


        double dx = robotPose.getX() - targetPose.getX();
        double dy = robotPose.getY() - targetPose.getY();
        double error = Math.sqrt(dx * dx + dy * dy);

        return xy.calculate(error, -robotVelXY);
    }


    public double getHeadingVel(Pose2d robotPose, Point2d goalPoint, double angleVel){
        double robotHeading = Math.toDegrees(Globals.normalize(robotPose.getH()));

        //get turn req
        double turnReq = Math.toDegrees(Globals.normalize(Math.atan2(goalPoint.getX() - robotPose.getX(), goalPoint.getX() - robotPose.getY())));

        double deltaAngle = Globals.normalize(turnReq - robotHeading);

        //make delta angle be between -180 and 180
        if (deltaAngle > 180){
            deltaAngle -= 180;
        } else  if (deltaAngle < -180){
            deltaAngle += 180;
        }

        return r.calculate(deltaAngle, -angleVel);
    }

}
