package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

public class TurnToPointSegment implements PathSegment{
    Vector2d drivePose, turnPose;
    double tol;
    boolean stopReq;
    double startTime;
    public TurnToPointSegment(Vector2d drivePose, Vector2d turnPose, double tol, boolean stopReq){
        this.drivePose = drivePose;
        this.turnPose = turnPose;
        this.tol = tol;
        this.stopReq = stopReq;
    }
    public TurnToPointSegment(Vector2d drivePose, Vector2d turnPose, double tol){
        this(drivePose, turnPose, tol, false);
    }
    public TurnToPointSegment(Vector2d drivePose, Vector2d turnPose){
        this(drivePose, turnPose, 1.5);
    }
    @Override
    public boolean isDone() {
        boolean velSatisfied = !stopReq ||
                Robot.getInstance().mecanumDrivetrain.vel.vec().getMag() < 4.0;


        return Robot.getInstance().mecanumDrivetrain.inRangeTurnToPoint(drivePose, turnPose, tol, tol*0.1)
                && velSatisfied;
    }

    @Override
    public void startSegment() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean failed() {
        return false;
    }

    @Override
    public Pose2d getPose() {
        Pose2d currPose = Robot.getInstance().mecanumDrivetrain.currPose;
        return new Pose2d(currPose.vec(), Math.atan2(
                turnPose.getY() - currPose.getY(),
                turnPose.getX() - currPose.getX()
        ));
    }

    @Override
    public void runSegment() {
        Robot.getInstance().mecanumDrivetrain.pidTurnToPos(drivePose, turnPose);
    }
}
