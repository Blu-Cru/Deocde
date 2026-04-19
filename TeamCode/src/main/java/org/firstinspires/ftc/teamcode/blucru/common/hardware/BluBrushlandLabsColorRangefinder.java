package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluBrushlandLabsColorRangefinder implements BluHardwareDevice{

    private BluDigitalChannel pin0, pin1;
    private boolean state1;
    private boolean state2;

    public BluBrushlandLabsColorRangefinder(String pin0, String pin1){
        this.pin0 = new BluDigitalChannel(pin0);
        this.pin1 = new BluDigitalChannel(pin1);
        configurePinsForInput();
    }

    @Override
    public void init() {
        configurePinsForInput();
    }

    @Override
    public void read() {
        pin0.read();
        pin1.read();
        state1 = pin0.getState();
        state2 = pin1.getState();
    }

    @Override
    public void write() {
    }

    public boolean ballDetected(){
        return state2;
    }
    public boolean greenBall(){
        return state2 && !purpleBall();
    }
    public boolean purpleBall(){
        return state1;
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
