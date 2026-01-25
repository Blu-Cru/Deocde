package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import static org.firstinspires.ftc.teamcode.blucru.common.util.Globals.telemetry;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class TurnToSegment implements PathSegment{
    double heading;
    double startTime;
    double maxTime;
    public TurnToSegment(double heading, double maxTime){
        this.heading = heading;
        this.maxTime = maxTime;
    }
    @Override
    public boolean isDone() {
        return Robot.getInstance().sixWheelDrivetrain.isTurnComplete();
    }

    @Override
    public void startSegment() {
        startTime = System.currentTimeMillis();
        Robot.getInstance().sixWheelDrivetrain.turnTo(heading);
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
        //dont do anything, segment already started
        heading = Robot.getInstance().sixWheelDrivetrain.getPos().getH();

        telemetry.addData("heading", heading);
    }
}
