package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.Tele;

public class BluDigitalChannel implements BluHardwareDevice{
    DigitalChannel channel;
    boolean state;
    String name;
    public BluDigitalChannel(String name){
        this.name = name;
        this.channel = Globals.hwMap.get(DigitalChannel.class, name);
    }

    @Override
    public void init() {
        state = false;
    }

    @Override
    public void read() {
        state = channel.getState();
    }

    @Override
    public void write() {

    }

    @Override
    public void telemetry() {
        Globals.telemetry.addData(name + " state", state);
    }

    public boolean getState(){
        return state;
    }
}
