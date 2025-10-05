package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class Pinpoint implements RobotLocalizer{
    //TODO TUNE PER ROBOT
    public static double parallelYOffset = -144.675, perpXOffset = -70;
    private GoBildaPinpointDriver pinpoint;
    private double headingOffset;

    private Pose2D pinpointPose;

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

        pinpoint.resetPosAndIMU();
        pinpointPose = pinpoint.getPosition();
    }

    @Override
    public void read() {
        pinpoint.update();
        pinpointPose = pinpoint.getPosition();
    }

    /**
     * returns pose in x,y,h in inch,inch,radian
     * */
    @Override
    public Pose2d getPose() {

        //Pose2D is different from Pose2d, Pose2D is ftc's Pose where Pose2d is the custom pose
        return new Pose2d(pinpointPose.getX(DistanceUnit.INCH), pinpointPose.getY(DistanceUnit.INCH), pinpointPose.getHeading(AngleUnit.RADIANS));
    }

    /**
     * in inches
     * */
    @Override
    public double getX() {
        return pinpointPose.getX(DistanceUnit.INCH);
    }

    /**
     * in inches
     * */
    @Override
    public double getY() {
        return pinpointPose.getY(DistanceUnit.INCH);
    }

    /**
     * in radians
     * */
    @Override
    public double getHeading() {
        return pinpointPose.getHeading(AngleUnit.RADIANS) - headingOffset;
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
}
