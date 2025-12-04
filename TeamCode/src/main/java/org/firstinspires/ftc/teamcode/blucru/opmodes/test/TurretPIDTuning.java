package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@Config
@TeleOp(group = "test")

public class TurretPIDTuning extends BluLinearOpMode {

    public static double angle = 0;


    @Override
    public void initialize() {
        super.reportTelemetry = false;
        robot.clear();
        addTurret();
        Globals.multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void periodic(){
        if (gamepad1.a){
            turret.setAngle(angle);
        }

        if (gamepad1.x){
            turret.updatePD();
        }

        Globals.multiTelemetry.addData("Turret Pos", turret.getAngle());
        Globals.multiTelemetry.addData("Target Pos", angle);
        Globals.multiTelemetry.update();
    }
}
