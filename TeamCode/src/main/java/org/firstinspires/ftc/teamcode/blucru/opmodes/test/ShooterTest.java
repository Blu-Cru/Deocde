package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
public class ShooterTest extends BluLinearOpMode {
    double hoodAngle;
    public void initialize(){
        robot.clear();
        addShooter();
        hoodAngle = 0.5;
    }

    public void periodic(){
        shooter.shoot(-gamepad1.left_stick_y);

        hoodAngle += -gamepad1.right_stick_y * 2.0;
        shooter.setHoodAngle(hoodAngle);
    }

}
