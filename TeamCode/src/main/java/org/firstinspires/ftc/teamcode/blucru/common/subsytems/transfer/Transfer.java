package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer implements BluSubsystem, Subsystem {

    TransferServo[] transferServos;
    private static final double DOWN_POSITION = 0.0;//TODO: find positions
    private static final double UP_POSITION = 1.0;
    public enum State {
        DOWN,
        UP
    }
    private State leftState;
    private State middleState;
    private State rightState;
    public Transfer(HardwareMap hardwareMap) {
        transferServos =  new TransferServo[]{new LeftTransferServo(), new MiddleTransferServo(), new RightTransferServo()};
    }

    public void leftSetDown() {
        leftState = State.DOWN;
        transferServos[0].setBottom();
    }

    public void leftSetUp() {
        leftState = State.UP;
        transferServos[0].setVertical();
    }
    public void middleSetDown() {
        middleState = State.DOWN;
        transferServos[1].setBottom();
    }

    public void middleSetUp() {
        middleState = State.UP;
        transferServos[1].setVertical();
    }
    public void rightSetDown() {
        rightState = State.DOWN;
        transferServos[2].setBottom();
    }

    public void rightSetUp() {
        rightState = State.UP;
        transferServos[2].setVertical();
    }

    public State getLeftState() {
        return leftState;
    }

    public State getMiddleState() {
        return middleState;
    }

    public State getRightState() {
        return rightState;
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
        telemetry.addData("Left transfer state: ", leftState);
        telemetry.addData("Middle transfer state: ", middleState);
        telemetry.addData("Right transfer state: ", rightState);

        for(TransferServo servo : transferServos){
            servo.telemetry();
        }
    }

    @Override
    public void reset() {
        leftSetDown();
        middleSetDown();
        rightSetDown();
    }
    public void setAllDown(){
        leftSetDown();
        middleSetDown();
        rightSetDown();
    }
    public void setAllUp(){
        leftSetUp();
        middleSetUp();
        rightSetUp();
    }

}
