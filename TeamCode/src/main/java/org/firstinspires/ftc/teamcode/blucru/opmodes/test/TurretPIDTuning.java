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

    // Dashboard tunables
    public static double angle = 0;
    public static double power = 0;

    public static int state = 0;

    @Override
    public void initialize() {
        super.reportTelemetry = true;
        robot.clear();
        addTurret();

        Globals.multiTelemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
    }

    @Override
    public void periodic() {

        // Live PID tuning
        turret.updatePID();

        if (state == 0) {
            turret.setAngle(angle);
        } else {
            turret.setPower(power);
        }

        double currentAngle = turret.getAngle();
        double error = angle - currentAngle;

        Globals.multiTelemetry.addData("Turret Angle", currentAngle);
        Globals.multiTelemetry.addData("Target Angle", angle);
        Globals.multiTelemetry.addData("Error", error);
        Globals.multiTelemetry.addData("Mode", state == 0 ? "PID" : "MANUAL");

        turret.telemetry(telemetry);
        Globals.multiTelemetry.update();
    }
}