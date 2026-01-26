package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
@Config
public class Pinpoint implements RobotLocalizer{
    //TODO TUNE PER ROBOT
    public static double parallelYOffset = 138.5, perpXOffset = 94.05;
    private GoBildaPinpointDriver pinpoint;
    private double headingOffset;

    private Pose2d pinpointPose;

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

    }

    @Override
    public void read() {
        pinpoint.update();
        Pose2D pinpointPose2D = pinpoint.getPosition();
        pinpointPose = new Pose2d(pinpointPose2D.getX(DistanceUnit.INCH), pinpointPose2D.getY(DistanceUnit.INCH), pinpointPose2D.getHeading(AngleUnit.RADIANS));

        Pose2D vel = pinpoint.getVelocity();
        Robot.getInstance().positionHistory.add(pinpointPose, new Pose2d(vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), vel.getHeading(AngleUnit.RADIANS)));
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
        return new Pose2d(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }

    @Override
    public void setHeading(double heading) {
        pinpoint.setHeading(heading, AngleUnit.RADIANS);
    }

    @Override
    public void telemetry(Telemetry telemetry) {

        telemetry.addData("Pinpoint heading", pinpoint.getHeading(AngleUnit.RADIANS));

    }

    public void reset(){
        pinpoint.resetPosAndIMU();
    }
}
