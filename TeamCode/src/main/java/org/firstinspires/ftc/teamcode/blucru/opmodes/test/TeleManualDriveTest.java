package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group =  "test")
public class TeleManualDriveTest extends BluLinearOpMode {
    @Override
    public void initialize(){
        enableDash();
        addDrivetrain();
    }
    @Override
    public void periodic(){
        drivetrain.teleOpDrive(gamepad1);

        if (driver1.pressedA()){
            drivetrain.setHeadingTele(Math.PI/2);
        }

        if (driver1.pressedB()){
            drivetrain.driveToHeading(0,1);
        }
    }
}
