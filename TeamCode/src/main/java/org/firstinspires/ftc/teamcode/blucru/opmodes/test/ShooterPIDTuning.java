package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp
@Config
public class ShooterPIDTuning extends BluLinearOpMode {

    public static double vel = 0;

    public void initialize(){
        robot.clear();
        addShooter();
        enableDash();
        vel = 0;
    }

    public void periodic(){
        if (gamepad1.a) {
           shooter.shootWithVelocity(vel);
        }
        if (gamepad1.x){
            shooter.leftShooter.setPower(0.5);
        }

        if (gamepad1.y){
            shooter.middleShooter.setPower(0.5);
        }

        if (gamepad1.b){
            shooter.rightShooter.setPower(0.5);
        }

    }

    public void telemetry(){
        Globals.telemetry.addData("shooter left vel", shooter.getLeftVel());
        Globals.telemetry.addData("shooter middle vel", shooter.getMiddleVel());
        Globals.telemetry.addData("shooter right vel", shooter.getRightVel());
        Globals.telemetry.addData("target vel", vel);
        Globals.telemetry.update();
    }

}
