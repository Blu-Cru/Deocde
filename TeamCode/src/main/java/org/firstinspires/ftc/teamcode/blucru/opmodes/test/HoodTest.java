package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp
public class HoodTest extends BluLinearOpMode {
    double pos;

    @Override
    public void initialize() {
        robot.clear();
        addShooter();
    }

    @Override
    public void periodic() {
        if (driver1.pressedX()){
            shooter.setHoodAngle(26);
            pos = 26;
        }
        if (gamepad1.x){
            shooter.setHoodAngle(50);
            pos = 50;
        }
        if (gamepad1.a){
            shooter.setHoodAngle(38);
            pos = 38;
        }
    }

    @Override
    public void telemetry(){
        telemetry.addData("Servo dir", shooter.hoodLeft.getDir());
        telemetry.addData("Servo pos", shooter.hoodLeft.getServoPos());
        telemetry.addData("Thought pos", shooter.hoodLeft.getPos());
        telemetry.addData("Angle", pos);
    }
}
