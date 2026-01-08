package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.control;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

@Config
public class DrivePID {

    public static double
        kPxy=0.15, kDxy=0.01,
        kPh = 1.7, kDh = 0.035;

    public PDController xyController, headingController;

    public Pose2d targetPose;

    public DrivePID(){
        xyController = new PDController(kPxy, kDxy);
        headingController = new PDController(kPh, kDh);
        targetPose = new Pose2d(0,0,0);
    }

    /**
     * setting power in inches
     * */
    public Pose2d calculate(Vector2d xState, Vector2d yState, double heading){

        //get the current position and velocity

        Vector2d currPose = new Vector2d(xState.getX(), yState.getX());

        Vector2d currVel = new Vector2d(xState.getY(), yState.getY());

        //get the error vector

        Vector2d errorVector = targetPose.vec().subtractNotInPlace(currPose);


        //make the error vector linear, so only values should be x and y
        double vectorHeading = errorVector.getHeading();

        Vector2d linearError = errorVector.rotate(-vectorHeading);

        double error = linearError.getMag();

        //get mag
        //vel doesnt need to rotate bc the mag doesnt change based on angle of vec
        double mag = xyController.calculate(error, -currVel.getMag());

        double headingPower = getRotate(heading);

        return new Pose2d(Vector2d.polarToCartesian(mag, vectorHeading), headingPower);
    }

    public Vector2d getXY(Vector2d xState, Vector2d yState){
        //get the current position and velocity

        Vector2d currPose = new Vector2d(xState.getX(), yState.getX());

        Vector2d currVel = new Vector2d(xState.getY(), yState.getY());

        //get the error vector

        Vector2d errorVector = targetPose.vec().subtractNotInPlace(currPose);


        //make the error vector linear, so only values should be x and y
        double vectorHeading = errorVector.getHeading();

        Vector2d linearError = errorVector.rotate(-vectorHeading);

        double error = linearError.getMag();

        //get percentage of mag
        //vel doesnt need to rotate bc the mag doesnt change based on angle of vec
        double mag = xyController.calculate(error, currVel.getMag());

        return Vector2d.polarToCartesian(mag, vectorHeading);
    }

    public void setTargetPose(Vector2d targetPose){
        this.targetPose = new Pose2d(targetPose);
        xyController.setSetPoint(targetPose.getMag());
    }

    public void setTargetPose(Pose2d pose){
        this.targetPose = pose;
        xyController.setSetPoint(pose.vec().getMag());
        headingController.setSetPoint(pose.getH());
    }

    public void updatePID(){
        xyController.setPID(kPxy, 0,kDxy);
        headingController.setPID(kPh, 0, kDh);
    }

    public void setTargetHeading(double targetHeading){
        headingController.setSetPoint(targetHeading);
    }

    public double getRotate(double heading){
        // wrap heading
        if (heading - headingController.getSetPoint() < -Math.PI){
            heading += 2 * Math.PI;
        } else if (heading - headingController.getSetPoint() > Math.PI){
            heading -= 2 * Math.PI;
        }
        return Range.clip(headingController.calculate(heading), -1, 1);
    }

    public double getRotate(Vector2d curr, Vector2d end){
        double deltaAngle = curr.getX() - end.getY();
        //wrapping to be between -pi and pi
        if (deltaAngle < -Math.PI){
            curr = new Vector2d(deltaAngle + 2*Math.PI, curr.getY());
        } else if (deltaAngle > Math.PI){
            curr = new Vector2d(deltaAngle - 2*Math.PI, curr.getY());
        }

        return headingController.calculate(curr, end);
    }

    public double getRotate(Vector2d curr){
       return getRotate(curr, new Vector2d( headingController.getSetPoint(), 0));
    }

    public boolean inRange(Pose2d currPose, double xTolerance, double yTolerance, double headingTolerance){

        Vector2d target = targetPose.vec();
        Vector2d curr = currPose.vec();

        boolean inRangeX = Math.abs(target.getX() - curr.getX()) < xTolerance;
        boolean inRangeY = Math.abs(target.getY() - curr.getY()) < yTolerance;
        boolean inRangeH = Math.abs(targetPose.getH() - currPose.getH()) < headingTolerance;

        return inRangeX && inRangeY && inRangeH;

    }

    public boolean inRange(Pose2d currPose, double xyTolerance, double headingTolerance){

        Vector2d target = targetPose.vec();
        Vector2d curr = currPose.vec();

        boolean inRangeXY = target.subtractNotInPlace(curr).getMag() < xyTolerance;

        boolean inRangeH = Math.abs(targetPose.getH() - currPose.getH()) < headingTolerance;

        return inRangeXY && inRangeH;

    }
}
