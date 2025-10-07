package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer implements BluSubsystem, Subsystem {

    TransferServo[] transferServos;
    public enum State {
        DOWN,
        UP
    }
    private State state;
    public Transfer(HardwareMap hardwareMap) {
        transferServos =  new TransferServo[]{new LeftTransferServo(), new MiddleTransferServo(), new RightTransferServo()};
    }

    public void setDown() {
        state = State.DOWN;
        setAngle(-10); //TODO: find correct angle, -10 degrees is just a random number
    }

    public void setUp() {
        state = State.UP;
        setAngle(30); //TODO: find correct angle, 30 degrees is just a random number
    }

    public State getState() {
        return state;
    }

    @Override
    public void init() {
        for(TransferServo servo : transferServos){
            servo.init();
        }
    }

    @Override
    public void read() {
        for(TransferServo servo : transferServos){
            servo.read();
        }
    }

    @Override
    public void write() {
        for(TransferServo servo : transferServos){
            servo.write();
        }

    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Transfer state: ", state);
        for(TransferServo servo : transferServos){
            servo.telemetry();
        }
    }

    @Override
    public void reset() {
        setDown();
    }
    public void setAngle(double degrees){
        for(TransferServo servo:transferServos){
            servo.setAngle(degrees);
        }
    }
}
