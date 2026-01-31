package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluColorSensor implements BluHardwareDevice{
    RevColorSensorV3 colorSensor;
    double red;
    double blue;
    double green;
    double[] purpleBottomThreshold, purpleTopThreshold, greenBottomThreshold, greenTopThreshold;
    public BluColorSensor(String name, double[][] thresholds){
        colorSensor = Globals.hwMap.get(RevColorSensorV3.class, name);
        purpleBottomThreshold = thresholds[0];
        purpleTopThreshold = thresholds[1];
        greenBottomThreshold = thresholds[2];
        greenTopThreshold = thresholds[3];
    }
    @Override
    public void init() {

    }

    @Override
    public void read() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        red = colors.red;
        blue = colors.blue;
        green = colors.green;
    }

    @Override
    public void write() {

    }

    @Override
    public void telemetry() {

    }

    public double getRed(){
        return red;
    }
    public double getBlue(){
        return blue;
    }
    public double getGreen(){
        return green;
    }

    public boolean isPurple(){
        boolean inRed = red < purpleTopThreshold[0]
                && red > purpleBottomThreshold[0];
        boolean inBlue = blue < purpleTopThreshold[2]
                && blue > purpleBottomThreshold[2];
        boolean inGreen = green < purpleTopThreshold[1]
                && green > purpleBottomThreshold[1];
        Globals.telemetry.addData("Sensor Name", colorSensor.getDeviceName());
        Globals.telemetry.addData("in red", inRed);
        Globals.telemetry.addData("in blue", inBlue);
        Globals.telemetry.addData("in green", inGreen);
        Globals.telemetry.addLine("");
        return (inRed && inBlue && inGreen);
    }
    public boolean isGreen(){
        boolean inRed = red < greenTopThreshold[0]
                && red > greenBottomThreshold[0];
        boolean inBlue = blue < greenTopThreshold[2]
                && blue > greenBottomThreshold[2];
        boolean inGreen = green < greenTopThreshold[1]
                && green > greenBottomThreshold[1];
        return (red < greenTopThreshold[0]
                && red > greenBottomThreshold[0]
                && green < greenTopThreshold[1]
                && green > greenBottomThreshold[1]
                && blue < greenTopThreshold[2]
                && blue > greenBottomThreshold[2]);
    }
}
