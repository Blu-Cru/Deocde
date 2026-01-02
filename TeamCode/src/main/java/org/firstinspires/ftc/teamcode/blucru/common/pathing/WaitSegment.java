package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class WaitSegment implements PathSegment{
    private final double waitTime;
    private double startTime;
    private Pose2d waitPose;  // Store pose where we started waiting

    public WaitSegment(double waitTime){
        this.waitTime = waitTime;
    }

    @Override
    public boolean isDone() {
        return System.currentTimeMillis() - startTime >= waitTime;
    }

    @Override
    public void startSegment() {
        startTime = System.currentTimeMillis();
        // Capture current pose when wait starts
        waitPose = Robot.getInstance().sixWheelDrivetrain.getPos();
    }

    @Override
    public boolean failed() {
        return false;
    }

    @Override
    public Pose2d getPose() {
        return waitPose;  // Return the pose where we started waiting
    }

    @Override
    public void runSegment() {
        // Do nothing - just wait. Drivetrain is already in IDLE mode
        // from endSixWheel() being called when previous segment completed
    }
}
