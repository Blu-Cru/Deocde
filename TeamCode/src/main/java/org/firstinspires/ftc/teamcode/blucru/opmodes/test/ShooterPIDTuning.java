package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

//@TeleOp
//@Config
public class ShooterPIDTuning extends BluLinearOpMode {

    public static double vel = 0;

    public void initialize(){
        robot.clear();
        addShooter();
        Globals.multiTelemetry = new MultipleTelemetry(telemetry
                , FtcDashboard.getInstance().getTelemetry());
        vel = 0;
    }

    public void periodic(){
        if (gamepad1.a) {
           shooter.shootWithVelocity(vel);
        }

    }

    public void telemetry(){
        Globals.multiTelemetry.addData("shooter vel", shooter.getLeftVel());
        Globals.multiTelemetry.addData("target vel", vel);
        Globals.multiTelemetry.update();
    }

}
