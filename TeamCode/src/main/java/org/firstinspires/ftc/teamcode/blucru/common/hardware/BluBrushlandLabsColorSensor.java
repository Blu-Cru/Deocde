package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluBrushlandLabsColorSensor implements BluHardwareDevice{

    private DigitalChannel pin0, pin1;
    private boolean state1;
    private boolean state2;

    public BluBrushlandLabsColorSensor(String pin0, String pin1){
        this.pin0 = Globals.hwMap.get(DigitalChannel.class, pin0);
        this.pin1 = Globals.hwMap.get(DigitalChannel.class, pin1);
    }

    @Override
    public void init() {

    }

    @Override
    public void read() {
        state1 = pin0.getState();
        state2 = pin1.getState();
    }

    @Override
    public void write() {
    }

    public boolean ballDetected(){
        return state1 || state2;
    }

    @Override
    public void telemetry() {
    }

    public boolean getPin0State(){
        return state1;
    }

    public boolean getPin1State(){
        return state2;
    }
}
