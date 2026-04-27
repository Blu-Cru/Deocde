package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp
public class TurretCamRelocalizationTest extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addSixWheel();
        addTurret();
        robot.addTurretCam();
        Globals.setAlliance(Alliance.BLUE);
    }

    public void periodic(){
        if (gamepad1.a) {
            sixWheel.setPosition(new Pose2d(0, 0, 0));
            turret.resetEncoder();
        }
        telemetry.addData("Turret Cam Pose Raw", robot.turretCam.getBotpose());
        telemetry.addData("Turret Cam Pose Kalman", robot.turretCam.getKalmanFilteredBotpose());
        telemetry.addData("Turret Cam Pose Pose History", robot.turretCam.getBotPosePoseHistory());
        telemetry.addData("Pose History Length", robot.positionHistory.size());
    }

}
