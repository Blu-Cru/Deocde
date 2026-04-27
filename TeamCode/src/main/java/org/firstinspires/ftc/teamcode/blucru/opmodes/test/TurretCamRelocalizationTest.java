package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp
public class TurretCamRelocalizationTest extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        robot.addTurretCam();
        addTurret();
        addSixWheel();
    }

    public void periodic(){
        if (gamepad1.a) {
            sixWheel.setPosition(new Pose2d(0, 0, 0));
            turret.resetEncoder();
        }
        telemetry.addData("Turret Cam Pose Raw", robot.turretCam.getBotpose());
        telemetry.addData("Turret Cam Pose Kalman", robot.turretCam.getBotpose());
        try{
            wait(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        };
        telemetry.addData("Turret Cam Pose Pose History", robot.turretCam.getBotPosePoseHistory());
    }

}
