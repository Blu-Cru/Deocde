package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.control.DriveKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp
public class DriveKinematicsTest extends BluLinearOpMode {
    public void initialize(){

    }

    public void telemetry(){
        double[] powers = DriveKinematics.getDriveMotorPowers(new Pose2d(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x));
        telemetry.addData("fl power", powers[0]);
        telemetry.addData("fr power", powers[1]);
        telemetry.addData("bl power", powers[2]);
        telemetry.addData("br power", powers[3]);
    }
}
