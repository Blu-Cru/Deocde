package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluBrushlandLabsColorRangefinder implements BluHardwareDevice{

    public static final double GREEN_VOLTAGE_LOW = 1.2;
    public static final double GREEN_VOLTAGE_HIGH = 1.9;
    public static final double PURPLE_VOLTAGE_HIGH = 1.2;
    public static final double DISTANCE_THRESHOLD_MM = 60.0;

    private DigitalChannel pin0, pin1;
    private AnalogInput analogPin0;
    private boolean state1;
    private boolean state2;
    private double voltage;
    private boolean useAnalog;

    public BluBrushlandLabsColorRangefinder(String pin0, String pin1){
        this.pin0 = Globals.hwMap.get(DigitalChannel.class,pin0);
        this.pin1 = Globals.hwMap.get(DigitalChannel.class,pin1);
        this.useAnalog = false;
        configurePinsForInput();
    }

    public BluBrushlandLabsColorRangefinder(String pin0, String pin1, String analogPinName){
        this.pin0 = Globals.hwMap.get(DigitalChannel.class,pin0);
        this.pin1 = Globals.hwMap.get(DigitalChannel.class,pin1);
        this.analogPin0 = Globals.hwMap.get(AnalogInput.class, analogPinName);
        this.useAnalog = true;
        configurePinsForInput();
    }

    @Override
    public void init() {
        configurePinsForInput();
    }

    @Override
    public void read() {
        state1 = pin0.getState();
        state2 = pin1.getState();
        if (useAnalog) {
            voltage = analogPin0.getVoltage();
        }
    }

    @Override
    public void write() {
    }

    public double getVoltage() {
        return voltage;
    }

    public boolean hasArtifact() {
        return state2;
    }

    public boolean isGreen() {
        if (useAnalog) {
            return hasArtifact() && voltage > GREEN_VOLTAGE_LOW && voltage < GREEN_VOLTAGE_HIGH;
        }
        return state2;
    }

    public boolean isPurple() {
        if (useAnalog) {
            return hasArtifact() && voltage < PURPLE_VOLTAGE_HIGH;
        }
        return state1;
    }

    public boolean ballDetected(){
        if (useAnalog) {
            return hasArtifact();
        }
        return state1 || state2;
    }

    public boolean greenBall(){
        return isGreen();
    }

    public boolean purpleBall(){
        return isPurple();
    }

    public boolean getRawState1(){
        return state1;
    }

    public boolean getRawState2(){
        return state2;
    }

    private void configurePinsForInput() {
        pin0.setMode(DigitalChannel.Mode.INPUT);
        pin1.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void telemetry() {
    }
}
