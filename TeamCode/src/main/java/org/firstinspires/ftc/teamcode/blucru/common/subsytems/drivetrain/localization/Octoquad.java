package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
@Config
public class Octoquad implements RobotLocalizer{
    //TODO TUNE PER ROBOT
    public static double parallelYOffset = -138.5, perpXOffset = -94.05;
    private OctoQuad octoquad;
    private double headingOffset;

    private Pose2d pose;
    private Pose2d vel;

    public Octoquad(String name){
        this(Globals.hwMap.get(OctoQuad.class, name));
    }

    public Octoquad(OctoQuad octoquad){
        this.octoquad = octoquad;
        headingOffset = 0;
        //TODO UPDATE PER ROBOT
        //x encoder
        //TODO TUNE HEADING SCALAR

        // Set encoder directions (must be set before resetLocalizerAndCalibrateIMU)
        octoquad.setSingleEncoderDirection(5, OctoQuad.EncoderDirection.REVERSE);
        octoquad.setSingleEncoderDirection(6, OctoQuad.EncoderDirection.REVERSE);

        octoquad.setAllLocalizerParameters(5,6,(float) (2000/(32 * Math.PI)),(float) (2000/(32 * Math.PI)),(float) perpXOffset, (float) parallelYOffset, 1.02F,20);

        OctoQuad.LocalizerDataBlock info = octoquad.readLocalizerData();
        pose = new Pose2d(info.posX_mm, info.posY_mm, info.heading_rad);
        vel = new Pose2d(info.velX_mmS, info.velY_mmS, info.velHeading_radS);

    }

    @Override
    public void read() {
        OctoQuad.LocalizerDataBlock info = octoquad.readLocalizerData();
        Globals.telemetry.addData("Info x", info.posX_mm);
        Globals.telemetry.addData("Info y", info.posY_mm);
        //dividing for unit conversion
        pose = new Pose2d(info.posX_mm / 25.4, info.posY_mm / 25.4, info.heading_rad);
        vel = new Pose2d(info.velX_mmS / 25.4, info.velY_mmS / 25.4, info.velHeading_radS);
    }
    /**
     * returns pose in x,y,h in inch,inch,radian
     * */
    @Override
    public Pose2d getPose() {
        return pose;
    }

    /**
     * in inches
     * */
    @Override
    public double getX() {
        return pose.getX();
    }

    /**
     * in inches
     * */
    @Override
    public double getY() {
        return pose.getY();
    }

    /**
     * in radians
     * */
    @Override
    public double getHeading() {
        return pose.getH() - headingOffset;
    }

    /**
     * for setting offsets, inch, inch, radian
     * */
    @Override
    public void setOffset(double x, double y, double h) {
        octoquad.setLocalizerTcpOffsetMM_X((float) x);
        octoquad.setLocalizerTcpOffsetMM_Y((float) y);
        headingOffset = h;
    }

    /**
     * inch, inch, radian
     * */
    public void setPosition(double x, double y, double h){
        Globals.telemetry.addLine("Setting Pose");
        octoquad.setLocalizerPose((int) (x * 25.4),(int) (y * 25.4),(float) h);
        read();
    }

    @Override
    public void setPosition(Pose2d pose) {
        Globals.telemetry.addData("Pose", "X: " + pose.getX() + ",Y: " + pose.getY() + ",H: " + pose.getH());
        setPosition(pose.getX(), pose.getY(), pose.getH());
    }

    @Override
    public Pose2d getVel() {
        return vel;
    }

    @Override
    public void setHeading(double heading) {
        octoquad.setLocalizerPose((int) (pose.getX() * 25.4), (int) (pose.getY() * 25.4), (float) heading);
    }

    @Override
    public void telemetry(Telemetry telemetry) {

        telemetry.addData("Pinpoint heading", pose.getH());

    }

    public void reset(){
        octoquad.resetLocalizerAndCalibrateIMU();
    }
}
