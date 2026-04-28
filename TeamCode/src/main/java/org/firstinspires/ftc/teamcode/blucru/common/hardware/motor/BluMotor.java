package org.firstinspires.ftc.teamcode.blucru.common.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluMotor extends DcMotorImplEx implements BluHardwareDevice {
    String name;
    double power=0, lastPower=0, multiplier = 1;
    double current;
    public BluMotor(String name){
        this(name, Direction.FORWARD);
    }
    public BluMotor(String name, Direction direction){
        this(name, direction, ZeroPowerBehavior.FLOAT);
    }
    public BluMotor(String name, Direction direction, ZeroPowerBehavior zpb){
        this(Globals.hwMap.get(DcMotor.class, name), name, direction, zpb);
    }
    private BluMotor(DcMotor motor, String name, Direction direction, ZeroPowerBehavior zpb){
        super(motor.getController(), motor.getPortNumber(), Direction.FORWARD);
        this.name = name;
        if (direction == Direction.REVERSE){
            multiplier = -1;
        }
        super.setZeroPowerBehavior(zpb);

    }

    public void setDirection(Direction direction) {
        if (direction == Direction.FORWARD) {
            multiplier = 1;
        } else {
            multiplier = -1;
        }
    }

    public void setPower(double power){
        this.power = Range.clip(power,-1,1);
    }
    @Override
    public void init() {
        super.setPower(0);

    }

    @Override
    public void read() {
        //current = super.getCurrent(CurrentUnit.MILLIAMPS);
    }

    @Override
    public void write() {
        if (Math.abs(power - lastPower) > 0.005){
            //power has changed
            lastPower = power * multiplier;
            super.setPower(Globals.getCorrectPower(power * multiplier));
        }
    }

    @Override
    public void telemetry() {
        Telemetry telemetry = Globals.telemetry;
        telemetry.addData(name + " Power: ", power);
        telemetry.addData(name + " Last Power: ", lastPower);
    }

    public double getPower(){
        return power;
    }
    //public double getCurrent(){return current;}

    public double getDcMotorPower(){
        return super.getPower();
    }
}
