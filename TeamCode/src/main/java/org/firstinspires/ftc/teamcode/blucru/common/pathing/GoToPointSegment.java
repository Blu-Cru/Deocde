package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
@Config
public class GoToPointSegment implements PathSegment{
    //CHANGE IF NECESSARY
    static double MAX_TIME = 5000;
    static double STOP_VEL =4.0;
    Pose2d targetPose;
    double xyTol;
    double startTime;
    boolean stopRequiredToEnd;

    public GoToPointSegment(Pose2d targetPose, double xyTol, boolean stopRequiredToEnd){
        this.targetPose = targetPose;
        this.xyTol = xyTol;
        this.stopRequiredToEnd = stopRequiredToEnd;
    }
    public GoToPointSegment(Pose2d targetPose, double xyTol){
        this(targetPose, xyTol, false);
    }
    public GoToPointSegment(Pose2d targetPose, boolean stopRequiredToEnd){
        this(targetPose, Globals.defaultXYTol, stopRequiredToEnd);
    }
    public GoToPointSegment(Pose2d targetPose){
        this(targetPose, Globals.defaultXYTol, false);
    }

    @Override
    public boolean isDone() {

        boolean velGood = !stopRequiredToEnd ||
                (Robot.getInstance().mecanumDrivetrain.vel.vec().getMag() < STOP_VEL);

        return Robot.getInstance().mecanumDrivetrain.inRange(xyTol, xyTol * 0.07)
                && velGood;
    }

    @Override
    public void startSegment() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean failed() {
        return System.currentTimeMillis() - startTime > MAX_TIME;
    }

    @Override
    public Pose2d getPose() {
        return targetPose;
    }
    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    @Override
    public void runSegment() {
        Robot.getInstance().mecanumDrivetrain.pidTo(targetPose);
    }
}
