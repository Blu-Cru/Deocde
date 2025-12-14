package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.control.DriveKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.LimelightLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.RobotLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

public class DriveBase implements BluSubsystem {
    RobotLocalizer localizer;

    BluMotor fl, fr, bl, br;

    BluMotor[] motors;

    public Pose2d currPose, vel;
    public double heading, headingVel;

    public Vector2d xState, yState, headingState;

    public DriveBase() {
        // With IMU fusion for better accuracy
        Pinpoint pinpoint = new Pinpoint("pinpoint");
        localizer = new LimelightLocalizer("limelight", pinpoint);

        // TODO MAKE SURE THAT THE DIRECTION IS RIGTH
        fl = new BluMotor(Globals.flMotorName, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        fr = new BluMotor(Globals.frMotorName, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        bl = new BluMotor(Globals.blMotorName, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        br = new BluMotor(Globals.brMotorName, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        xState = new Vector2d(0, 0);
        yState = new Vector2d(0, 0);
        headingState = new Vector2d(0, 0);
        heading = 0;
        headingVel = 0;

        motors = new BluMotor[] { fl, fr, bl, br };
    }

    @Override
    public void init() {

        for (BluMotor motor : motors) {
            motor.init();
        }

        read();
    }

    @Override
    public void read() {

        localizer.read();

        currPose = localizer.getPose();
        vel = localizer.getVel();
        heading = localizer.getHeading();
        headingVel = localizer.getVel().getH();

        xState = new Vector2d(currPose.getX(), vel.getX());
        yState = new Vector2d(currPose.getY(), vel.getY());
        headingState = new Vector2d(heading, headingVel);

        for (BluMotor motor : motors) {
            motor.read();
        }
    }

    @Override
    public void write() {

        for (BluMotor motor : motors) {
            motor.write();
        }

    }

    public void drive(Pose2d vel) {
        vel = DriveKinematics.staticFriction(vel);

        double[] powers = DriveKinematics.getDriveMotorPowers(vel);

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    public void drive(Vector2d xy, double h) {
        drive(new Pose2d(xy, h));
    }

    public void driveFieldCentric(Pose2d fieldCentric) {
        Pose2d robotCentricPose = new Pose2d(fieldCentric.vec().rotate(-localizer.getHeading()), fieldCentric.getH());
        drive(robotCentricPose);
    }

    public void driveFieldCentric(Pose2d fieldCentric, Alliance alliance) {
        if (alliance == Alliance.RED) {
            // normal case, no rotation necessary
            // field setup is based off red
            driveFieldCentric(fieldCentric);

            // get out
            return;
        }

        // otherwise just do the same thing but rotated 180
        Pose2d robotDriveVel = new Pose2d(fieldCentric.vec().rotate(-localizer.getHeading() + Math.PI),
                fieldCentric.getH());
        drive(robotDriveVel);
    }

    public void driveFieldCentric(Vector2d translation, double heading) {
        driveFieldCentric(new Pose2d(translation, heading));
    }

    public void driveFieldCentric(Vector2d translation, double heading, Alliance alliance) {
        driveFieldCentric(new Pose2d(translation, heading), alliance);
    }

    public Pose2d getCurrPose() {
        return localizer.getPose();
    }

    public void setCurrentPose(Pose2d pose) {
        Globals.telemetry.addData("Pose", "X: " + pose.getX() + ",Y: " + pose.getY() + ",H: " + pose.getH());
        localizer.setPosition(pose);
    }

    public void setHeading(double heading) {
        localizer.setHeading(heading);
    }

    public void setAllianceSpecificHeading(double heading, Alliance alliance) {
        if (alliance == Alliance.RED) {
            // normal case, do nothing special
            setHeading(heading);

            // get out of func
            return;
        }

        // not normal case, need to rotate by 180 deg
        setHeading(Globals.normalize(heading + Math.PI));
    }

    public double getHeading() {
        return heading;
    }

    public Pose2d getVel() {
        return localizer.getVel();
    }

    public void drawPose() {
        Globals.drawPose(getCurrPose());
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        localizer.telemetry(telemetry);
    }

    @Override
    public void reset() {
        // cant reset drive encoders bc there are no encoders
    }
}
