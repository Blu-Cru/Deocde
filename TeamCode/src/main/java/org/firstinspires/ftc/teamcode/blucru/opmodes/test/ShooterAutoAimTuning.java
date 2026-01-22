package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@Config
@TeleOp(group = "test")
public class ShooterAutoAimTuning extends BluLinearOpMode {
    public static double leftAngle = 0;
    public static double rightAngle = 0;
    public static double middleAngle = 0;

    public static double turretAngle = 20;

    public static double vel = 0;

    public void initialize(){
        addShooter();
        addSixWheel();
        //addLLTagDetector();
        addTransfer();
        addTurret();
        turret.resetEncoder();
    }

    public void periodic(){
        if (gamepad1.b){
            turret.setAngle(0);
        } else {
            turret.setAngle(turretAngle);
        }

        if (driver1.pressedA()){
            shooter.setLeftHoodAngle(leftAngle);
            shooter.setMiddleHoodAngle(middleAngle);
            shooter.setRightHoodAngle(rightAngle);
        }

        if (driver1.pressedY()){
            shooter.shootWithVelocity(vel);
        }

        if (gamepad1.x){
            telemetry.addLine("here");
            shooter.shoot(0);
        }


        if (driver1.pressedDpadDown()){
            transfer.middleSetUp();
        }
        if (driver1.pressedDpadLeft()) {
            transfer.leftSetUp();
        }
        if (gamepad1.dpad_right){
            transfer.rightSetUp();
        }
        if (driver1.pressedLeftBumper()){
            transfer.setAllUp();
        }

        if (driver1.pressedDpadUp()){
            transfer.setAllMiddle();
        }
    }

}
