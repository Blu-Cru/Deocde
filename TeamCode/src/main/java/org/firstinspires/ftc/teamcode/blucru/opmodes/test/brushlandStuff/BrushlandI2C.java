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
    RevColorSensorV3 colorSensorV32;

    public void initialize(){
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "Color1");
        colorSensorV32 = hardwareMap.get(RevColorSensorV3.class, "Color");
        ((LynxI2cDeviceSynch) colorSensorV32.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) colorSensorV3.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    }

    public void periodic(){
        NormalizedRGBA colors = colorSensorV3.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        float hue = hsvValues[0];
        double distance = colorSensorV3.getDistance(DistanceUnit.MM);
        NormalizedRGBA colors2 = colorSensorV32.getNormalizedColors();
        float[] hsvValues2 = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues2);
        float hue2 = hsvValues[0];
        double distance1 = colorSensorV32.getDistance(DistanceUnit.MM);
        telemetry.addData(" Bottom rgb: ", colors.red + " " + colors.blue + " " + colors.green);
        telemetry.addData(" Bottom hue: ", hue);
        telemetry.addData("Distance Botton MM" , distance);
        telemetry.addData(" Top rgb: ", colors2.red + " " + colors2.blue + " " + colors2.green);
        telemetry.addData(" Top hue: ", hue2);
        telemetry.addData("Distance Top", distance1);
        telemetry.update();
    }

}
