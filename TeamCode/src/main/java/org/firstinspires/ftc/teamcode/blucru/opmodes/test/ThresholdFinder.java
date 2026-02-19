package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
@TeleOp
public class ThresholdFinder extends LinearOpMode {

    RevColorSensorV3 sensor;

    public void runOpMode(){
        sensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        waitForStart();

        while (opModeIsActive()){
            NormalizedRGBA colors = sensor.getNormalizedColors();
            telemetry.addData("Red", colors.red);
            telemetry.addData("Blue", colors.blue);
            telemetry.addData("Green", colors.green);
            telemetry.update();
        }
    }

}
