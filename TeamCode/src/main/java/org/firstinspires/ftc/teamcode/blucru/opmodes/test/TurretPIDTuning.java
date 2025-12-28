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


    public void initialize() {
        enableDash();
        robot.clear();
        addTurret();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void periodic(){
        if (state == 0) {
            turret.setAngle(angle);
        } else {
            turret.setPower(power);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("Turret Pos", turret.getAngle());
        telemetry.addData("Target Pos", angle);
        telemetry.addData("Target Power", turret.getPower());
        telemetry.addData("Error", turret.getRotateError(turret.getAngle(), angle));
    }
}
