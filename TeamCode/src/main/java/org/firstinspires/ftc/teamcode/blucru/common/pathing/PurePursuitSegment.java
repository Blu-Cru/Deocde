package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
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
        return (Robot.getInstance().sixWheelDrivetrain.getPos().
                getDistTo(new Pose2d(path[path.length-1].getX(), path[path.length-1].getY(), 0))) < 1;
    }

    @Override
    public void startSegment() {
        startTime = System.currentTimeMillis();
        Robot.getInstance().sixWheelDrivetrain.resetPurePursuit();
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
        Robot.getInstance().sixWheelDrivetrain.followPath(path);
    }
}
