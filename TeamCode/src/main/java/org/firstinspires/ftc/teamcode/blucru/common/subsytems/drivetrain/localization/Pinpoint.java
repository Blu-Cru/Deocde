package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
@Config
public class Pinpoint implements RobotLocalizer{
    //TODO TUNE PER ROBOT
    public static double parallelYOffset = 138.5, perpXOffset = 94.05;
    private GoBildaPinpointDriver pinpoint;
    private double headingOffset;
    private Pose2d pinpointPose;
    private double xyAccel;
    private double hAccel;
    private Pose2d lastLoopVel;
    private double lastLoopTime;
    private Pose2d vel;

    public Pinpoint(String name){
        this(Globals.hwMap.get(GoBildaPinpointDriver.class, name));
    }

    public Pinpoint(GoBildaPinpointDriver pinpoint){
        this.pinpoint = pinpoint;
        headingOffset = 0;
        //TODO UPDATE PER ROBOT
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(parallelYOffset, perpXOffset, DistanceUnit.MM);
        pinpointPose = new Pose2d(pinpoint.getPosition().getX(DistanceUnit.INCH), pinpoint.getPosition().getY(DistanceUnit.INCH), pinpoint.getPosition().getHeading(AngleUnit.RADIANS));
        lastLoopVel = new Pose2d(0,0,0);
        lastLoopTime = 0;

    }

    @Override
    public void read() {
        pinpoint.update();
        Pose2D pinpointPose2D = pinpoint.getPosition();
        pinpointPose = new Pose2d(pinpointPose2D.getX(DistanceUnit.INCH), pinpointPose2D.getY(DistanceUnit.INCH), pinpointPose2D.getHeading(AngleUnit.RADIANS));

        Pose2D velPose2D = pinpoint.getVelocity();

        vel = new Pose2d(velPose2D.getX(DistanceUnit.INCH), velPose2D.getX(DistanceUnit.INCH), velPose2D.getHeading(AngleUnit.RADIANS));
        double currTime = Globals.matchTime.milliseconds();
        double xAccel = (vel.getX() - lastLoopVel.getX())/(currTime - lastLoopTime) * 1000.0;
        double yAccel = (vel.getY() - lastLoopVel.getY())/(currTime - lastLoopTime) * 1000.0;
        xyAccel = Math.hypot(xAccel, yAccel);
        hAccel = (vel.getH() - lastLoopVel.getH())/(currTime - lastLoopTime) * 1000.0;

        Robot.getInstance().positionHistory.add(pinpointPose, vel);

    }

    /**
     * returns pose in x,y,h in inch,inch,radian
     * */
    @Override
    public Pose2d getPose() {
        return pinpointPose;
    }

    /**
     * in inches
     * */
    @Override
    public double getX() {
        return pinpointPose.getX();
    }

    /**
     * in inches
     * */
    @Override
    public double getY() {
        return pinpointPose.getY();
    }

    /**
     * in radians
     * */
    @Override
    public double getHeading() {
        return pinpointPose.getH() - headingOffset;
    }

    /**
     * for setting offsets, inch, inch, radian
     * */
    @Override
    public void setOffset(double x, double y, double h) {
        pinpoint.setOffsets(x,y,DistanceUnit.INCH);
        headingOffset = h;
    }

    /**
     * inch, inch, radian
     * */
    public void setPosition(double x, double y, double h){
        pinpoint.setPosX(x, DistanceUnit.INCH);
        pinpoint.setPosY(y, DistanceUnit.INCH);
        pinpoint.setHeading(h, AngleUnit.RADIANS);
        read();
    }

    @Override
    public void setPosition(Pose2d pose) {


        Globals.telemetry.addData("Pose", "X: " + pose.getX() + ",Y: " + pose.getY() + ",H: " + pose.getH());

        pinpoint.setPosX(pose.getX(), DistanceUnit.INCH);
        pinpoint.setPosY(pose.getY(), DistanceUnit.INCH);
        pinpoint.setHeading(pose.getH(), AngleUnit.RADIANS);
        read();
    }

    @Override
    public Pose2d getVel() {
        return vel;
    }

    @Override
    public void setHeading(double heading) {
        pinpoint.setHeading(heading, AngleUnit.RADIANS);
    }

    @Override
    public void telemetry(Telemetry telemetry) {

        telemetry.addData("Pinpoint heading", pinpoint.getHeading(AngleUnit.RADIANS));

    }

    public double getXyAccel(){
        return xyAccel;
    }

    public double gethAccel(){
        return hAccel;
    }

    public void reset(){
        pinpoint.resetPosAndIMU();
    }
}
