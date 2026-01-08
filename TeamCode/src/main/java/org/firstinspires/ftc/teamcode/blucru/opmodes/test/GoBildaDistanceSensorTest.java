package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

//@TeleOp
public class GoBildaDistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel distSensor = hardwareMap.get(DigitalChannel.class, "aligner");
        boolean detected;
        waitForStart();

        while (opModeIsActive()){
            detected = distSensor.getState();
            telemetry.addData("Detected?", detected);
            telemetry.update();
        }
    }
}
