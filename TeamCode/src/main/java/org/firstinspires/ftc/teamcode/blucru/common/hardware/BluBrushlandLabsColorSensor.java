package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluBrushlandLabsColorSensor implements BluHardwareDevice{

    private DigitalChannel pin0, pin1;
    private boolean state1;
    private boolean state2;

    public BluBrushlandLabsColorSensor(String pin0, String pin1){
        this.pin0 = Globals.hwMap.get(DigitalChannel.class,pin0);
        this.pin1 = Globals.hwMap.get(DigitalChannel.class,pin1);
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
    }

    @Override
    public void write() {
    }

    public boolean ballDetected(){
        return state1 || state2;
    }
    public boolean greenBall(){
        return state2;
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
