package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp
@Config
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

        if (gamepad1.x){
            shooter.updatePID();
            telemetry.addLine("updated");
            telemetry.addData("New PID Constants", Shooter.p + ", " + Shooter.d);
        }
    }

    public void telemetry(){
        Globals.multiTelemetry.addData("shooter vel", shooter.getVel());
        Globals.multiTelemetry.addData("target vel", vel);
        Globals.multiTelemetry.update();
    }

}
