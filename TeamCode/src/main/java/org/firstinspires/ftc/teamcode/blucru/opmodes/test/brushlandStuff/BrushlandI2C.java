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
    RevColorSensorV3 colorSensorV33;

    public void initialize(){
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "ColorA");
        colorSensorV32 = hardwareMap.get(RevColorSensorV3.class, "ColorB");
        colorSensorV33 = hardwareMap.get(RevColorSensorV3.class, "ColorC");
        ((LynxI2cDeviceSynch) colorSensorV33.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
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
        NormalizedRGBA colors3 = colorSensorV32.getNormalizedColors();
        float[] hsvValues3 = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues2);
        float hue3 = hsvValues[0];
        double distance3 = colorSensorV32.getDistance(DistanceUnit.MM);
        telemetry.addData(" Color A rgb: ", colors.red + " " + colors.blue + " " + colors.green);
        telemetry.addData(" Color A hue: ", hue);
        telemetry.addData(" Distance Color A MM" , distance);
        telemetry.addData(" Color B rgb: ", colors2.red + " " + colors2.blue + " " + colors2.green);
        telemetry.addData(" Color B hue: ", hue2);
        telemetry.addData(" Color B Distance", distance1);
        telemetry.addData(" Color C rgb: ", colors3.red + " " + colors3.blue + " " + colors3.green);
        telemetry.addData(" Color C hue: ", hue3);
        telemetry.addData(" Color C Top", distance3);
        telemetry.update();
    }

}
