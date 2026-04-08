package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(name = "Brushland I2C Read! :)")
public class BrushlandI2C extends BluLinearOpMode {
    RevColorSensorV3 colorSensorV3;

    public void initialize(){
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "Color");
        ((LynxI2cDeviceSynch) colorSensorV3.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    }

    public void periodic(){
        NormalizedRGBA colors = colorSensorV3.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];
        telemetry.addData("rgb: ", colors.red + " " + colors.blue + " " + colors.green);
        telemetry.addData("hue: ", hue);
        telemetry.update();
    }

}
