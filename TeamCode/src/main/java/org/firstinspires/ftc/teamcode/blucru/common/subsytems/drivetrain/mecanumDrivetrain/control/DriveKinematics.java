package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.control;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;
@Config
public class DriveKinematics {

    //TODO SET THESE VALUES WHEN USING THIS ARCHITECTURE
    public static double
            AXIAL_DECEL = 0.5, LATERAL_DECEL = 0.5,HEADING_DECEL = 8,
            LATERAL_MULT = 1.2,
            STRAFE_STATIC_FRICTION = 0.02, FORWARD_STATIC_FRICTION = 0.01, //how much power needed to move the robot
            MAX_DRIVE_ACCEL_DELTA = 0.5, MAX_DRIVE_DECEL_DELTA = 0.5;

    public static double getDistToPoint(Pose2d curr, Pose2d target){
        return Math.sqrt(Math.pow(curr.getX() - target.getX(), 2) + Math.pow(curr.getY() - target.getY(),2));
    }

    public static Pose2d getStopPose(Pose2d pose, Pose2d fieldVel){
        //convert field-centric velocity to robot centric velocity
        Pose2d robotVel = new Pose2d(fieldVel.vec().rotate(-pose.getH()), fieldVel.getH());

        //get change in x and y
        double robotDeltaX = robotVel.getX() * Math.abs(robotVel.getX()) / (2.0 * AXIAL_DECEL);
        double robotDeltaY = robotVel.getY() * Math.abs(robotVel.getY()) / (2.0 * LATERAL_DECEL);

        Vector2d robotDeltaPose = new Vector2d(robotDeltaX, robotDeltaY);

        return new Pose2d(pose.vec().addNotInPlace(robotDeltaPose.rotate(pose.getH())), getHeadingDecel(pose.getH(), fieldVel.getH()));
    }

    public static double getHeadingDecel(double heading, double headingVel){
        double headingDelta = headingVel * Math.abs(headingVel) /(2.0 * HEADING_DECEL);

        //normalizes angle
        return Globals.normalize(heading + headingDelta);
    }

    public static Pose2d staticFriction(Pose2d drivePose){
        Vector2d driveVector = drivePose.vec();

        if (driveVector.getMag() != 0){
            double angle = driveVector.getHeading();

            double minMagOfStaticFriction = STRAFE_STATIC_FRICTION * FORWARD_STATIC_FRICTION /
                    Math.hypot(STRAFE_STATIC_FRICTION * Math.cos(angle) , FORWARD_STATIC_FRICTION * Math.sin(angle));

            double newDriveMag = minMagOfStaticFriction + (1-minMagOfStaticFriction) * driveVector.getMag();
            return new Pose2d(driveVector.getX() * newDriveMag / driveVector.getMag(),
                    driveVector.getY() * newDriveMag / driveVector.getMag(),
                    drivePose.getH());
        } else {
            //no static friction
            return drivePose;
        }
    }

    public static double[] getDriveMotorPowers(Pose2d drivePose){

        //make pose have values between -1 and 1
        drivePose = clip(drivePose, 1);

        double[] powers = new double[4];

        powers[0] = drivePose.getX() - drivePose.getY() * LATERAL_MULT - drivePose.getH();
        powers[1] = drivePose.getX() + drivePose.getY() * LATERAL_MULT + drivePose.getH();
        powers[2] = drivePose.getX() + drivePose.getY() * LATERAL_MULT - drivePose.getH();
        powers[3] = drivePose.getX() - drivePose.getY() * LATERAL_MULT + drivePose.getH();

        double maxPower = Math.max(Math.abs(powers[0]),
                Math.max(Math.abs(powers[1]),
                        Math.max(Math.abs(powers[2]),
                                Math.abs(powers[3]))));

        if (maxPower > 1){
            for (int i = 0; i < 4; i++){
                powers[i] /= maxPower;
            }
        }

        return powers;

    }

    public static Pose2d clip(Pose2d pose, double max){
        return new Pose2d(Range.clip(pose.getX(), -max, max),
                Range.clip(pose.getY(), -max, max),
                Range.clip(pose.getH(), -max, max));
    }





}
