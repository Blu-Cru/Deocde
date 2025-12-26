package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
public class TurretLockOnGoalTest extends BluLinearOpMode {
    public void initialize(){
        robot.clear();
        addSixWheel();
        addTurret();
        addShooter();
    }

    public void periodic(){
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
    }
}
