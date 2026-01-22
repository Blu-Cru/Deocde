package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorSensor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;

public class Elevator implements BluSubsystem, Subsystem {
    private BluServo elevatorServoLeft;
    private BluServo elevatorServoRight;
    private BluBrushlandLabsColorSensor leftSensor, middleSensor, rightSensor;
    private static final double DOWN_POSITION_LEFT = 0.65;//TODO: find positions
    private static final double UP_POSITION_LEFT = 0.35;
    private static final double MIDDLE_POSITION_LEFT = 0.62;

    private static final double DOWN_POSITION_RIGHT = 0.35;//TODO: find positions
    private static final double UP_POSITION_RIGHT = 0.65;
    private static final double MIDDLE_POSITION_RIGHT = 0.38;

    public Elevator(){
        elevatorServoLeft = new BluServo("elevatorLeft");
        elevatorServoRight = new BluServo("elevatorRight");
        setDown();
        write();
    }

    public void setUp(){
        elevatorServoLeft.setPos(UP_POSITION_LEFT);
        elevatorServoRight.setPos(UP_POSITION_RIGHT);
    }

    public void setDown(){
        elevatorServoLeft.setPos(DOWN_POSITION_LEFT);
        elevatorServoRight.setPos(DOWN_POSITION_RIGHT);
    }
    public boolean ballInElevatorSlot(){
        return leftSensor.ballDetected() || middleSensor.ballDetected() || rightSensor.ballDetected();
    }
    public void turnOffElevatorServo(){
        elevatorServoLeft.disable();
        //always want to write after a disable
        elevatorServoLeft.write();
    }
    public void setMiddle(){
        elevatorServoLeft.setPos(MIDDLE_POSITION_LEFT);
        elevatorServoRight.setPos(MIDDLE_POSITION_RIGHT);
    }

    @Override
    public void init() {
        elevatorServoLeft.init();
    }

    @Override
    public void read() {
        elevatorServoLeft.read();
    }

    @Override
    public void write() {
        elevatorServoLeft.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        elevatorServoLeft.telemetry();
    }

    @Override
    public void reset() {
        setDown();
    }
}
