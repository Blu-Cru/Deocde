package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluBrushlandColorSensor implements BluHardwareDevice{

    private DigitalChannel pin0, pin1;

    private boolean pin0State, pin1State;

    public BluBrushlandColorSensor(String pin0, String pin1){
        this.pin0 = Globals.hwMap.get(DigitalChannel.class, pin0);
        this.pin1 = Globals.hwMap.get(DigitalChannel.class, pin1);
    }

    @Override
    public void init() {

    }

    @Override
    public void read() {

        pin0State = pin0.getState();
        pin1State = pin1.getState();

    }

    @Override
    public void write() {

    }

    @Override
    public void telemetry() {
        Globals.telemetry.addData("pin 0 state", pin0State);
        Globals.telemetry.addData("pin 1 state", pin1State);
    }

    public boolean getPin0State(){
        return pin0State;
    }

    public boolean getPin1State(){
        return pin1State;
    }
}
