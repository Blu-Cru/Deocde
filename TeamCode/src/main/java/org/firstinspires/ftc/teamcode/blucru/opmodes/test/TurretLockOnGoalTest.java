package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
public class TurretLockOnGoalTest extends BluLinearOpMode {
    public void initialize(){
        robot.clear();
        addSixWheel();
        robot.addTurretCam();
        //addLLTagDetector();
        addIntake();
        addElevator();
        addTransfer();
        addShooter();
        addTurret();
        addTilt();
        Globals.multiTelemetry = new MultipleTelemetry(telemetry);
    }

    public void periodic() throws InterruptedException {
        if (gamepad1.a){
            turret.lockOnGoal();
        }
        if (gamepad1.y){
            turret.toggleManual();
            turret.setPower(0);
        }
        if (gamepad1.dpad_up){
            sixWheel.setPosition(new Pose2d(0,0,0));
        }
        if (driver1.pressedRightBumper()){
            Globals.setAlliance(Alliance.RED);
        }
        if (driver1.pressedLeftBumper()){
            Globals.setAlliance(Alliance.BLUE);
        }
        //Thread.sleep(10);
    }
}
