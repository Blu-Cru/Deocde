package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
public class ColorSensorFinder extends LinearOpMode {

    RevColorSensorV3 sensor1;
    RevColorSensorV3 sensor2;
    RevColorSensorV3 sensor3;
    RevColorSensorV3 sensor4;
    RevColorSensorV3 sensor5;
    RevColorSensorV3 sensor6;

    public void runOpMode() {
        sensor1 = hardwareMap.tryGet(RevColorSensorV3.class, "sensor1"); //middle left
        sensor2 = hardwareMap.tryGet(RevColorSensorV3.class, "sensor2"); //left top
        sensor3 = hardwareMap.tryGet(RevColorSensorV3.class, "sensor3"); //left bottom
        sensor4 = hardwareMap.tryGet(RevColorSensorV3.class, "sensor4"); //middle right
        sensor5 = hardwareMap.tryGet(RevColorSensorV3.class, "sensor5"); //right bottom
        sensor6 = hardwareMap.tryGet(RevColorSensorV3.class, "sensor6"); //right top

        waitForStart();

        while (opModeIsActive()) {
            displaySensorData("Sensor 1", sensor1);
            displaySensorData("Sensor 2", sensor2);
            displaySensorData("Sensor 3", sensor3);
            displaySensorData("Sensor 4", sensor4);
            displaySensorData("Sensor 5", sensor5);
            displaySensorData("Sensor 6", sensor6);
            telemetry.update();
        }
    }

    private void displaySensorData(String name, RevColorSensorV3 sensor) {
        if (sensor == null) {
            telemetry.addData(name, "NOT CONNECTED");
        } else {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            telemetry.addData(name + " R", "%.3f", colors.red);
            telemetry.addData(name + " G", "%.3f", colors.green);
            telemetry.addData(name + " B", "%.3f", colors.blue);
        }
    }

}
