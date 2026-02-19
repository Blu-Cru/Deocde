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
        sensor1 = hardwareMap.tryGet(RevColorSensorV3.class, "middleColorSensorRight"); //middle left
        sensor2 = hardwareMap.tryGet(RevColorSensorV3.class, "middleColorSensorLeft"); //left top
        sensor3 = hardwareMap.tryGet(RevColorSensorV3.class, "rightColorSensorTop"); //left bottom
        sensor4 = hardwareMap.tryGet(RevColorSensorV3.class, "rightColorSensorBottom"); //middle right
        sensor5 = hardwareMap.tryGet(RevColorSensorV3.class, "leftColorSensorTop"); //right bottom
        sensor6 = hardwareMap.tryGet(RevColorSensorV3.class, "leftColorSensorBottom"); //right top

        waitForStart();

        while (opModeIsActive()) {
            displaySensorData("mR", sensor1);
            displaySensorData("mL", sensor2);
            displaySensorData("rT", sensor3);
            displaySensorData("rB", sensor4);
            displaySensorData("lT", sensor5);
            displaySensorData("lB", sensor6);
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
