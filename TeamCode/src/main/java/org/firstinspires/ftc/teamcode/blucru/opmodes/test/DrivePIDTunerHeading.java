package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.control.DrivePID;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group =  "tuner")
public class DrivePIDTunerHeading extends BluLinearOpMode {
    double p = 0.001, d = 0;
    @Override
    public void initialize(){
        enableDash();
        addDrivetrain();
        drivetrain.setCurrentPose(new Pose2d(0,0,0));
    }

    public void periodic(){
        if (driver1.pressedA()){
            //turn 180 degrees
            drivetrain.pidTo(new Pose2d(0, 0, Math.PI));
        }
        if (driver1.pressedB()) {
            //go back to 0,0,0
            drivetrain.pidTo(new Pose2d(0,0,0));
        }
        if (driver1.pressedRightBumper()){
            //double p
            p *= 2;
            DrivePID.kPh = p;
            drivetrain.updatePID();
        }
        if (driver1.pressedLeftBumper()){
            //double d
            if(d == 0){
                d = 0.000000001;
            }
            d *= 2;
            DrivePID.kDh = d;
            drivetrain.updatePID();
        }

    }

    public void telemetry(){
        Globals.telemetry.addData("P: ", p);
        Globals.telemetry.addData("D: ", d);
    }
}
