package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.seattlesolvers.solverslib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluColorSensor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer implements BluSubsystem, Subsystem {

    TransferServo[] transferServos;
    private static final double DOWN_POSITION = 0.0;//TODO: find positions
    private static final double UP_POSITION = 1.0;
    public enum State {
        DOWN,
        UP,
        MIDDLE
    }
    private State leftState;
    private State middleState;
    private State rightState;
    private BluColorSensor leftSensorBottom, leftSensorTop, middleSensorBottom, middleSensorTop, rightSensorBottom, rightSensorTop;
    public Transfer(HardwareMap hardwareMap) {
        transferServos =  new TransferServo[]{new LeftTransferServo(), new MiddleTransferServo(), new RightTransferServo()};

        setAllDown();
        write();
    }

    public void leftSetDown() {
        leftState = State.DOWN;
        transferServos[0].setBottom();
    }

    public void leftSetMiddle() {
        leftState = State.MIDDLE;
        transferServos[0].setMiddle();
    }

    public void leftSetUp() {
        leftState = State.UP;
        transferServos[0].setVertical();
    }
    public void middleSetDown() {
        middleState = State.DOWN;
        transferServos[1].setBottom();
    }

    public void middleSetMiddle() {
        middleState = State.MIDDLE;
        transferServos[1].setMiddle();
    }

    public void middleSetUp() {
        middleState = State.UP;
        transferServos[1].setVertical();
    }
    public void rightSetDown() {
        rightState = State.DOWN;
        transferServos[2].setBottom();
    }

    public void rightSetMiddle() {
        rightState = State.MIDDLE;
        transferServos[2].setMiddle();
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
//        telemetry.addData("Left transfer state: ", leftState);
//        telemetry.addData("Middle transfer state: ", middleState);
//        telemetry.addData("Right transfer state: ", rightState);

        for(TransferServo servo : transferServos){
//            servo.telemetry();
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
    public void setAllMiddle(){
        leftSetMiddle();
        middleSetMiddle();
        rightSetMiddle();
    }
    public void setAllUp(){
        leftSetUp();
        middleSetUp();
        rightSetUp();
    }
}
