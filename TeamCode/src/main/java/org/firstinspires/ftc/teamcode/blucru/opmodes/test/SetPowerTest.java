package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import org.firstinspires.ftc.teamcode.blucru.common.pathing.MecanumDrivetrainSetPowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

public class SetPowerTest extends BluLinearOpMode {

    @Override
    public void initialize(){
        addDrivetrain();
        drivetrain.setCurrentPose(new Pose2d(0,0,0));
        new MecanumDrivetrainSetPowerCommand(0.9).schedule();
    }

    public void periodic(){
        drivetrain.teleOpDrive(gamepad1);
        if (gamepad1.left_bumper){
            new MecanumDrivetrainSetPowerCommand(0.5).schedule();
        }
        if (gamepad1.right_bumper){
            new MecanumDrivetrainSetPowerCommand(0.9).schedule();
        }
    }

}
