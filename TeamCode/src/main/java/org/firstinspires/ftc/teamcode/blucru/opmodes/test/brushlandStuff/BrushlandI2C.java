package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(name = "Brushland I2C Read! :)")
public class BrushlandI2C extends BluLinearOpMode {
    RevColorSensorV3 colorSensorV3;

    public void initialize(){
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "Color");
        ((LynxI2cDeviceSynch) colorSensorV3.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    }

    @Override
    public void telemetry() {
        NormalizedRGBA colors = colorSensorV3.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];
        double distance = colorSensorV3.getDistance(DistanceUnit.MM);
        telemetry.addData("green : ", colors.green);
        telemetry.addData("red: ", colors.red);
        telemetry.addData(" hue: ", hue);
        telemetry.addData("Distance MM" , distance);
    }

}
