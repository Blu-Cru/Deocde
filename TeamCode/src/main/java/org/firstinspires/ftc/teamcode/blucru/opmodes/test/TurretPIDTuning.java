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
    public static double power = 0;
    public static double state = 0;


    @Override
    public void initialize() {
        super.reportTelemetry = true;
        robot.clear();
        addTurret();
        Globals.multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void periodic() throws InterruptedException {
        Thread.sleep(34);

        if (state == 0) {
            turret.setAngle(angle);

            turret.updatePID();
        }else{
            turret.setPower(power);
        }

        Globals.multiTelemetry.addData("Turret Pos", -turret.getAngle());
        Globals.multiTelemetry.addData("Target Pos", angle);
        Globals.multiTelemetry.addData("Target Power", turret.getPower());
        //Globals.multiTelemetry.addData("Error", turret.getRotateError(turret.getAngle(), angle));
        turret.telemetry(telemetry);
        Globals.multiTelemetry.update();
    }
}
