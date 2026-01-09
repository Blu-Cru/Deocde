package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class LineToXSegment implements PathSegment{
    double targetX;
    double maxTime;
    double startTime;
    public LineToXSegment(double x, double maxTime){
        targetX = x;
        this.maxTime = maxTime;
    }

    @Override
    public boolean isDone() {
        //1 inch tol
        return Math.abs(targetX - Robot.getInstance().sixWheelDrivetrain.getPos().getX()) < 2;
    }

    @Override
    public void startSegment() {
        startTime = System.currentTimeMillis();
        Robot.getInstance().sixWheelDrivetrain.lineToX(targetX);
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

    }
}
