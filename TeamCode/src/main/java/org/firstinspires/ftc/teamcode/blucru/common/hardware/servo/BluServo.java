package org.firstinspires.ftc.teamcode.blucru.common.hardware.servo;

import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluServo extends ServoImpl implements BluHardwareDevice {
    String name;
    ServoController controller;
    double pos=0, lastPos=0;
    int direction;
    boolean enabled;
    public BluServo(String name){
        this(name, Direction.FORWARD);
    }
    public BluServo(String name, Direction direction){
        this(Globals.hwMap.get(ServoImpl.class, name), name, direction);
    }
    private BluServo(ServoImpl servo, String name, Direction direction){
        super(servo.getController(), servo.getPortNumber(), direction);
        super.setDirection(direction);
        this.name = name;
        this.controller = servo.getController();
        this.enabled = true;
        if (direction == Direction.FORWARD){
            this.direction = 1;
        } else{
            this.direction = 0;
        }
    }
    public void setPos(double pos){
        this.pos = Range.clip(pos,0,1);
    }
    public double getServoPos(){
        return super.getPosition();
    }
    public double getPos(){
        return pos;
    }
    public int getDir(){
        return direction;
    }


    /**
     * ONLY USE THESE FUNCTIONS WHEN GOING BETWEEN COMPLETE STOP STATES
     * */
    public void enable(){
        controller.pwmEnable();
        enabled = true;
    }
    public void disable(){
        controller.pwmDisable();
        enabled = false;
    }

    @Override
    public void init() {

    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        if (Math.abs(pos - lastPos) > 0.002){
            lastPos = pos;
            super.setPosition(pos);

        }
    }
    public String getName(){
        return name;
    }

    @Override
    public void telemetry() {
        try {
            Telemetry telemetry = Globals.telemetry;
            telemetry.addData(name + " Pos: ", pos);
        } catch (Exception ignored){}
    }

    public void addLine(String str){
        try{
            Telemetry telemetry = Globals.telemetry;
            telemetry.addLine(name + " " + str);
        }catch(Exception ignored){}
    }
    public boolean enabled(){
        return enabled;
    }
}
