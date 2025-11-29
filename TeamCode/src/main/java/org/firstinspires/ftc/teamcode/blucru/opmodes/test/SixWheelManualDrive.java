package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp
public class SixWheelManualDrive extends BluLinearOpMode {
    @Override
    public void initialize() {
        robot.clear();
        addSixWheel();
    }

    @Override
    public void periodic() {
        sixWheel.setDrivePower(0.5);
        sixWheel.teleDrive(gamepad1,0.001);
    }
}
