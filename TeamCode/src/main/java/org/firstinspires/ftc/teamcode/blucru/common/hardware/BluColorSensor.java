package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluColorSensor implements BluHardwareDevice{
    RevColorSensorV3 colorSensor;
    double red;
    double blue;
    double green;
    public BluColorSensor(String name){
        colorSensor = Globals.hwMap.get(RevColorSensorV3.class, name);
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
}
