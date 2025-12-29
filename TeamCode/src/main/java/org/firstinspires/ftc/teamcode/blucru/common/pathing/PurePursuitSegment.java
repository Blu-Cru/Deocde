package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class PurePursuitSegment implements PathSegment{

    Point2d[] path;
    Double targetHeading; // Target heading in degrees, null if no heading control

    double startTime;
    double maxTime;

    public PurePursuitSegment(Point2d[] path, double maxTime){
        this.path = path;
        this.maxTime = maxTime;
        this.targetHeading = null; // No heading control by default
    }

    public PurePursuitSegment(Point2d[] path, double targetHeading, double maxTime){
        this.path = path;
        this.targetHeading = targetHeading;
        this.maxTime = maxTime;
    }

    public Double getTargetHeading() {
        return targetHeading;
    }
    @Override
    public boolean isDone() {
        Globals.telemetry.addData("Last Point", path[path.length-1]);
        Pose2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos();
        double dist = robotPose.getDistTo(new Pose2d(path[path.length-1].getX(), path[path.length-1].getY(), 0));
        Globals.telemetry.addData("Dist", dist);

        // If no target heading specified, only check position
        if (targetHeading == null) {
            return dist < 2;
        }

        // Check both position and heading
        double robotHeadingDeg = Math.toDegrees(robotPose.getH());
        double headingError = targetHeading - robotHeadingDeg;

        // Normalize heading error to [-180, 180]
        while (headingError > 180) {
            headingError -= 360;
        }
        while (headingError <= -180) {
            headingError += 360;
        }
        headingError = Math.abs(headingError);

        Globals.telemetry.addData("Target Heading", targetHeading);
        Globals.telemetry.addData("Robot Heading", robotHeadingDeg);
        Globals.telemetry.addData("Heading Error", headingError);

        boolean positionReached = dist < 2;
        boolean headingReached = headingError < 5; // within 5 degrees

        return positionReached && headingReached;
    }

    @Override
    public void startSegment() {
        startTime = System.currentTimeMillis();
        if (targetHeading != null) {
            Robot.getInstance().sixWheelDrivetrain.followPath(path, targetHeading);
        } else {
            Robot.getInstance().sixWheelDrivetrain.followPath(path);
        }
    }

    @Override
    public boolean failed() {
        return System.currentTimeMillis() - startTime > maxTime;
    }

    @Override
    public Pose2d getPose() {
        return Robot.getInstance().sixWheelDrivetrain.getPos();
    }

    @Override
    public void runSegment() {
        // Path following continues automatically in the drivetrain's periodic update
    }
}
