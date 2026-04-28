package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@Config
@TeleOp(group = "test")
public class TeleDriveToPointTest extends BluLinearOpMode {
    double initAmountOfSubsystems;
    public static double dx = -72;
    public static double dy = 72;

    public void initialize(){;
        robot.clear();
        initAmountOfSubsystems = robot.getAmountOfSubsystems();
        addDrivetrain();
        robot.read();
        reportTelemetry = true;
    }

    public void periodic(){
        if (driver1.pressedA()){
            //go 10 inches to the right from 0,0, dt heading
            drivetrain.pidTo(new Pose2d(dx,0,drivetrain.getHeading()));
        }
        if (driver1.pressedB()){
            //go 10 inches forward from 0,0,dt heading
            drivetrain.pidTo(new Pose2d(0,dy,drivetrain.getHeading()));
        }
        if (driver1.pressedX()){
            //rotate 90 degrees to the left from 0,0,0
            drivetrain.pidTo(new Pose2d(0,0,Math.PI/2));
        }
        if(driver1.pressedY()){
            //back to 0,0,dt heading
            drivetrain.pidTo(new Pose2d(0,0,drivetrain.getHeading()));
        }
        if(driver1.pressedDpadUp()){
            //back to 0,0,0
            drivetrain.pidTo(new Pose2d(0,0,0));
        }
        if (driver1.pressedLeftBumper()){
            //idle drivetrain if something bad happens
            drivetrain.switchToIdle();
        }

        //reset heading
        if (driver1.pressedRightBumper()){
            drivetrain.setHeadingTele(Math.PI/2);
        }

        if (driver1.pressedDpadDown()){
            drivetrain.updatePID();
        }


    }

    public void telemetry(){
        telemetry.addData("Pinpoint pose", "X: " + drivetrain.currPose.getX() + ",Y: " + drivetrain.currPose.getY() + ",H: " +drivetrain.currPose.getH());
        /*telemetry.addData("curr pose heading", drivetrain.currPose.getH());
        telemetry.addData("Target Pid Pose Heading", drivetrain.pid.targetPose.getH());
        telemetry.addData("Heading getRotate", drivetrain.pid.getRotate(drivetrain.headingState));*/
    }

}
