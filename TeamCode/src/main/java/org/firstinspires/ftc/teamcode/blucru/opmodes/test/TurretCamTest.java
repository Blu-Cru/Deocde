package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

public class TurretCamTest extends BluLinearOpMode {
    public void initialize(){
        robot.clear();
        robot.addTurretCam();
        addSixWheel();
        addTurret();
    }

    public void periodic(){
        telemetry.addData("Robot Pos Cam", Robot.getInstance().turretCam.getBotPosePoseHistory());
        telemetry.addData("Robot Pos Loc", sixWheel.getPos());
    }
}
