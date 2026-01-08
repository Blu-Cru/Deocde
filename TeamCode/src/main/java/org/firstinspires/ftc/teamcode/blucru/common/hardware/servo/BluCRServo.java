package org.firstinspires.ftc.teamcode.blucru.common.hardware.servo;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluHardwareDevice;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class BluCRServo extends CRServoImpl implements BluHardwareDevice {
    String name;
    double power = 0, lastPower = 0;
    public BluCRServo(String name){
        this(name, Direction.FORWARD);
    }
    public BluCRServo(String name, Direction direction){
        this(Globals.hwMap.get(CRServoImpl.class, name), name, direction);
    }
    private BluCRServo(CRServo servo, String name, Direction direction){
        super(servo.getController(), servo.getPortNumber(),direction);
        this.name = name;
        this.power = 0;
        this.lastPower = 0;
    }

    public void setPower(double power){
        this.power = Range.clip(power, -1, 1);
    }
    @Override
    public void init() {
        super.setPower(0);
        setPower(0);
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        if (Math.abs(power - lastPower) > 0.005){
            lastPower = power;
            super.setPower(power);
        }
    }

    @Override
    public void telemetry() {
        Telemetry telemetry = Globals.telemetry;
        telemetry.addLine(name + " Power: " + power);
    }
}
