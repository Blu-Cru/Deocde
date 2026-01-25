package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class PurePursuitSegment implements PathSegment{

    Point2d[] path;
    double startTime;
    double maxTime;

    public PurePursuitSegment(Point2d[] path, double maxTime){
        this.path = path;
        this.maxTime = maxTime;
    }

    @Override
    public boolean isDone() {
        Globals.telemetry.addData("Last Point", path[path.length-1]);
        Pose2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos();
        double dist = robotPose.getDistTo(new Pose2d(path[path.length-1].getX(), path[path.length-1].getY(), 0));
        Globals.telemetry.addData("Delta X", robotPose.getX() - path[path.length - 1].getX());
        Globals.telemetry.addData("Delta Y", robotPose.getY() - path[path.length - 1].getY());
        Globals.telemetry.addData("Dist", dist);

        return dist < 2;
    }

    @Override
    public void startSegment() {
        startTime = System.currentTimeMillis();
        Robot.getInstance().sixWheelDrivetrain.followPath(path);
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
