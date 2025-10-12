package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp
public class ShooterPIDTuning extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addShooter();
        Globals.multiTelemetry = new MultipleTelemetry(telemetry
                , FtcDashboard.getInstance().getTelemetry());
    }

    public void periodic(){
        if (gamepad1.a){
            new ShootWithVelocityCommand(2500).schedule();
        }

        if (gamepad1.b){
            new ShootWithVelocityCommand(0).schedule();
        }

        if (gamepad1.x){
            shooter.updatePID();
            telemetry.addLine("updated");
            telemetry.addData("New PID Constants", Shooter.p + ", " + Shooter.d);
        }
    }

    public void telemetry(){
        Globals.multiTelemetry.addData("shooter vel", shooter.getVel());
        telemetry.addData("Shooter power", shooter.getPower());
        telemetry.addData("g1x", gamepad1.x);
    }

}
